"""Exercise the installed launch file and all three console entry points."""
import json
import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from armie.common import STATE_QOS
from test_ros import until


def test_installed_launch_mock_roundtrip_and_clean_shutdown(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID='188', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    logfile = tmp_path / 'launch.log'
    with logfile.open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'armie', 'legacy.launch.py'],
                                   env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        rclpy.init(domain_id=188)
        probe = Node('armie_launch_test')
        executor = SingleThreadedExecutor()
        executor.add_node(probe)
        try:
            states, scenes, faces = [], [], []
            subscriptions = [
                probe.create_subscription(String, '/armie/state', lambda m: states.append(json.loads(m.data)), STATE_QOS),
                probe.create_subscription(String, '/armie/perception', lambda m: scenes.append(json.loads(m.data)), STATE_QOS),
                probe.create_subscription(String, '/armie/hmi/tx', lambda m: faces.append(json.loads(m.data)), STATE_QOS),
            ]
            command = probe.create_publisher(String, '/armie/command', 10)
            look = probe.create_client(Trigger, '/armie/look')
            until(executor, lambda: states and faces and command.get_subscription_count() > 0
                  and look.service_is_ready(), timeout=8.)
            command.publish(String(data='look'))
            last_retry_sequence = [states[-1]['sequence']]
            def completed():
                # DDS discovery is per participant: this probe knowing the
                # service does not prove the newly launched Pet knows it yet.
                # Retry only its explicit pre-submission discovery rejection;
                # never duplicate an accepted request or hide other failures.
                latest = states[-1]
                if (latest.get('text') == '视觉节点尚未就绪。'
                        and latest['sequence'] > last_retry_sequence[0]):
                    last_retry_sequence[0] = latest['sequence']
                    command.publish(String(data='look'))
                return bool(scenes and any(f['expression'] == 'curious' for f in faces))
            try:
                until(executor, completed, timeout=8.)
            except AssertionError as exc:
                raise AssertionError(f'No look result. states={states}; log={logfile.read_text()}') from exc
            assert scenes[-1]['source'] == 'mock'
            assert process.poll() is None
        finally:
            executor.shutdown()
            probe.destroy_node()
            rclpy.shutdown()
            # Signal the launcher only; it forwards exactly once to its children.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.)
        assert process.returncode == 0, logfile.read_text()
    content = logfile.read_text()
    assert 'Traceback' not in content, content
    assert content.count('process has finished cleanly') == 3, content
