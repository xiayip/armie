import json
import socket
import threading
import time
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from armie.common import STATE_QOS
from armie.hmi import Hmi
from armie.perception import mock_scene
from armie.pet import Pet
from armie.vision import Vision


@contextmanager
def graph(classes, parameters=(), executor_factory=SingleThreadedExecutor):
    args = ['--ros-args']
    for value in parameters:
        args += ['-p', value]
    rclpy.init(args=args)
    nodes, executor = [], executor_factory()
    try:
        nodes = [factory() for factory in classes]
        probe = Node('armie_test_probe')
        nodes.append(probe)
        for node in nodes:
            executor.add_node(node)
        yield nodes, executor
    finally:
        # Finish action-server terminal-status work before destroying waitables.
        for node in nodes:
            for timer in node.timers:
                timer.cancel()
        deadline = time.monotonic() + .15
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        for node in nodes:
            executor.remove_node(node)
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


def until(executor, condition, timeout=4.):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.02)
    assert condition(), 'condition timed out'


def call(executor, client):
    until(executor, client.service_is_ready)
    future = client.call_async(Trigger.Request())
    until(executor, future.done)
    return future.result()


def test_pet_vision_hmi_mock_closed_loop_and_stop():
    with graph([Pet, Vision, Hmi]) as (nodes, executor):
        pet, vision, hmi, probe = nodes
        states, faces, scenes = [], [], []
        subscriptions = [
            probe.create_subscription(String, '/armie/state', lambda m: states.append(json.loads(m.data)), STATE_QOS),
            probe.create_subscription(String, '/armie/hmi/tx', lambda m: faces.append(json.loads(m.data)), STATE_QOS),
            probe.create_subscription(String, '/armie/perception', lambda m: scenes.append(json.loads(m.data)), STATE_QOS),
        ]
        command = probe.create_publisher(String, '/armie/command', 10)
        until(executor, lambda: command.get_subscription_count() > 0 and len(states) > 0)
        command.publish(String(data='greet'))
        until(executor, lambda: any(s['state'] == 'acting' for s in states))
        until(executor, lambda: not pet.motion.busy)
        command.publish(String(data='look'))
        until(executor, lambda: scenes and pet.state == 'idle')
        until(executor, lambda: any(f['expression'] == 'curious' for f in faces))
        assert scenes[-1]['source'] == 'mock'
        assert pet.motion.backend == 'mock' and not pet.motion.enabled
        stop = probe.create_client(Trigger, '/armie/stop')
        assert call(executor, stop).success
        command.publish(String(data='nod'))
        for _ in range(10):
            executor.spin_once(timeout_sec=.02)
        assert pet.state == 'stopped' and not pet.motion.busy
        wake = probe.create_client(Trigger, '/armie/wake')
        assert call(executor, wake).success
        assert pet.state == 'idle'


class VlmHandler(BaseHTTPRequestHandler):
    mode = 'valid'
    requests = []

    def log_message(self, *args):
        pass

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        type(self).requests.append(body)
        if self.mode == 'slow':
            time.sleep(.5)
        content = json.dumps(mock_scene()) if self.mode != 'invalid' else '{"joint_targets":[1]}'
        response = json.dumps({'choices': [{'message': {'content': content}}]}).encode()
        self.send_response(200)
        self.end_headers()
        try:
            self.wfile.write(response)
        except BrokenPipeError:
            pass


@pytest.mark.parametrize('mode,source', [('valid', 'http'), ('invalid', 'fallback'), ('slow', 'fallback')])
def test_camera_http_schema_and_timeout(mode, source):
    import cv2
    import numpy as np
    VlmHandler.mode, VlmHandler.requests = mode, []
    server = ThreadingHTTPServer(('127.0.0.1', 0), VlmHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with graph([Vision], ['vision_backend:=http', 'vlm_model:=test',
                             f'vlm_endpoint:=http://127.0.0.1:{server.server_port}/v1/chat/completions',
                             'vlm_timeout_sec:=0.15', 'look_cooldown_sec:=0.0']) as (nodes, executor):
            vision, probe = nodes
            scenes, snapshots = [], []
            sub = probe.create_subscription(String, '/armie/perception', lambda m: scenes.append(json.loads(m.data)), STATE_QOS)
            snap = probe.create_subscription(CompressedImage, '/armie/snapshot/compressed', snapshots.append, 1)
            pub = probe.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', qos_profile_sensor_data)
            until(executor, lambda: pub.get_subscription_count() > 0)
            _, jpeg = cv2.imencode('.jpg', np.zeros((64, 80, 3), dtype=np.uint8))
            pub.publish(CompressedImage(format='jpeg', data=jpeg.tobytes()))
            until(executor, lambda: vision.image is not None)
            client = probe.create_client(Trigger, '/armie/look')
            response = call(executor, client)
            assert response.success
            until(executor, lambda: len(scenes) == 1)
            assert scenes[0]['source'] == source
            assert scenes[0]['request_id'] == response.message
            assert VlmHandler.requests[0]['messages'][0]['content'][1]['image_url']['url'].startswith('data:image/jpeg;base64,')
            until(executor, lambda: len(snapshots) == 1)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.)


def test_no_camera_and_stale_camera_fall_back_without_http():
    with graph([Vision], ['vision_backend:=http', 'look_cooldown_sec:=0.0']) as (nodes, executor):
        vision, probe = nodes
        scenes = []
        sub = probe.create_subscription(String, '/armie/perception', lambda m: scenes.append(json.loads(m.data)), STATE_QOS)
        client = probe.create_client(Trigger, '/armie/look')
        assert call(executor, client).success
        until(executor, lambda: len(scenes) == 1)
        assert scenes[-1]['source'] == 'fallback'
        vision.image = CompressedImage(format='jpeg', data=b'invalid')
        vision.image_at = time.monotonic() - 10.
        assert call(executor, client).success
        until(executor, lambda: len(scenes) == 2)
        assert scenes[-1]['reason'] == 'no fresh camera image'


def test_udp_display_and_lost_heartbeat():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.settimeout(1.)
    try:
        with graph([Hmi], ['hmi_transport:=udp', f'hmi_udp_port:={receiver.getsockname()[1]}']) as (nodes, executor):
            hmi, probe = nodes
            pub = probe.create_publisher(String, '/armie/expression', STATE_QOS)
            until(executor, lambda: pub.get_subscription_count() > 0)
            pub.publish(String(data=json.dumps(dict(schema_version=1, sequence=1,
                                                   expression='happy', text='你好', ttl_ms=100))))
            until(executor, lambda: hmi.last_sequence == 1)
            assert json.loads(receiver.recv(4096))['expression'] == 'happy'
            until(executor, lambda: hmi.stale)
            assert json.loads(receiver.recv(4096))['expression'] == 'confused'
    finally:
        receiver.close()


def test_real_motion_is_disabled_and_bt_has_no_default_mapping():
    with graph([Pet], ['motion_backend:=trajectory']) as (nodes, executor):
        pet = nodes[0]
        ok, reason = pet.motion.start('nod')
        assert not ok and reason == 'motion_enabled is false'
        assert not pet.motion.busy
    with graph([Pet], ['motion_backend:=bt', 'motion_enabled:=true']) as (nodes, executor):
        ok, reason = nodes[0].motion.start('nod')
        assert not ok and 'no commissioned BT mapping' in reason
