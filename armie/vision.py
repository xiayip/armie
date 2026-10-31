import time
import uuid
from concurrent.futures import ThreadPoolExecutor

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .common import STATE_QOS, finite, param, publish, run
from .perception import fallback_scene, infer, jpeg_snapshot, mock_scene


class Vision(Node):
    def __init__(self):
        super().__init__('armie_vision')
        self.backend = param(self, 'vision_backend', 'mock')
        if self.backend not in {'mock', 'http'}:
            raise ValueError('vision_backend must be mock or http')
        self.endpoint = param(self, 'vlm_endpoint', '')
        self.model = param(self, 'vlm_model', '')
        self.key_env = param(self, 'vlm_api_key_env', 'ARMIE_VLM_API_KEY')
        self.timeout = finite(param(self, 'vlm_timeout_sec', 8.), .1, 30., 'vlm timeout')
        self.max_age = finite(param(self, 'image_max_age_sec', 2.), .1, 10., 'image age')
        self.max_edge = int(finite(param(self, 'image_max_edge', 640), 64, 1280, 'image edge'))
        self.cooldown = finite(param(self, 'look_cooldown_sec', 1.), 0., 30., 'look cooldown')
        self.image, self.image_at, self.last_request = None, 0., -1e9
        self.sub = self.create_subscription(
            CompressedImage, param(self, 'image_topic', '/camera/color/image_raw/compressed'),
            self.on_image, qos_profile_sensor_data)
        self.result_pub = self.create_publisher(String, '/armie/perception', STATE_QOS)
        self.snapshot_pub = self.create_publisher(CompressedImage, '/armie/snapshot/compressed', 1)
        self.service = self.create_service(Trigger, '/armie/look', self.look)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='armie-vlm')
        self.future, self.request_id, self.started, self.delivered = None, '', 0., False
        self.timer = self.create_timer(.05, self.tick)

    def on_image(self, msg):
        if 0 < len(msg.data) <= 8 * 1024 * 1024:
            # Timestamp freshness is checked as well as local receipt freshness.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp > 0 and not -.2 <= age <= self.max_age:
                self.image, self.image_at = None, 0.
                return
            self.image, self.image_at = msg, time.monotonic()

    def emit(self, scene, source, reason=''):
        publish(self.result_pub, dict(schema_version=1, request_id=self.request_id,
                                     source=source, reason=reason, scene=scene))

    def look(self, request, response):
        now = time.monotonic()
        if self.future is not None or now - self.last_request < self.cooldown:
            response.success, response.message = False, 'busy or cooling down'
            return response
        self.request_id, self.started, self.last_request = uuid.uuid4().hex, now, now
        self.delivered = False
        response.success, response.message = True, self.request_id
        if self.backend == 'mock':
            # Deliver from timer to ensure the service response precedes the result.
            self.future = self.pool.submit(lambda: (mock_scene(), None))
        elif self.image is None or now - self.image_at > self.max_age:
            self.future = self.pool.submit(lambda: (fallback_scene(), 'no fresh camera image'))
        else:
            snapshot = self.image
            self.snapshot_pub.publish(snapshot)
            data = bytes(snapshot.data)
            self.future = self.pool.submit(self.analyze, data)
        return response

    def analyze(self, data):
        try:
            jpeg = jpeg_snapshot(data, self.max_edge)
            return infer(jpeg, self.endpoint, self.model, self.key_env, self.timeout), None
        except Exception as exc:
            # Do not expose server bodies, authorization headers or image content in logs.
            return fallback_scene(), 'VLM unavailable or invalid output: ' + type(exc).__name__

    def tick(self):
        if self.future is None:
            return
        if self.future.done():
            scene, error = self.future.result()
            if not self.delivered:
                self.emit(scene, 'fallback' if error else self.backend, error or '')
            self.future = None
        elif not self.delivered and time.monotonic() - self.started > self.timeout + 1.:
            self.emit(fallback_scene(), 'fallback', 'VLM deadline exceeded')
            self.delivered = True
            # Keep the worker slot occupied until it exits. Never queue stale images.

    def destroy_node(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main():
    run(Vision)
