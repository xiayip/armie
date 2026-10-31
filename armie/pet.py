import json
import time

from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .common import STATE_QOS, param, publish, run
from .motion import Motion
from .perception import validate_scene


class Pet(Node):
    def __init__(self):
        super().__init__('armie_pet')
        self.state, self.expression, self.text = 'idle', 'idle', '你好，我是 Armie。'
        self.sequence, self.changed_at = 0, time.monotonic()
        self.pending_look, self.look_id, self.look_deadline = False, None, 0.
        self.look_generation = 0
        self.last_perception_id = None
        self.perception_buffer = {}
        self.pub = self.create_publisher(String, '/armie/state', STATE_QOS)
        self.face_pub = self.create_publisher(String, '/armie/expression', STATE_QOS)
        self.motion = Motion(self, self.motion_done)
        self.look_client = self.create_client(Trigger, '/armie/look')
        self.sub = self.create_subscription(String, '/armie/command', self.command, 10)
        self.perception_sub = self.create_subscription(
            String, '/armie/perception', self.perception, STATE_QOS)
        self.stop_service = self.create_service(Trigger, '/armie/stop', self.stop)
        self.wake_service = self.create_service(Trigger, '/armie/wake', self.wake)
        self.idle_sleep = param(self, 'idle_sleep_sec', 60.)
        self.timer = self.create_timer(.1, self.tick)
        self.heartbeat = self.create_timer(1., self.publish_state)
        self.publish_state()

    def transition(self, state, expression, text):
        self.state, self.expression, self.text = state, expression, text
        self.changed_at = time.monotonic()
        self.publish_state()

    def publish_state(self):
        self.sequence += 1
        face = dict(schema_version=1, sequence=self.sequence, expression=self.expression,
                    text=self.text[:240], ttl_ms=2500)
        publish(self.face_pub, face)
        publish(self.pub, dict(**face, state=self.state, motion_backend=self.motion.backend,
                               motion_enabled=self.motion.enabled, motion_busy=self.motion.busy,
                               motion_phase=self.motion.phase))

    def command(self, msg):
        command = msg.data.strip().lower()
        if command == 'stop':
            self.stop(None, Trigger.Response())
        elif command == 'wake':
            self.wake(None, Trigger.Response())
        elif self.state == 'stopped':
            return
        elif command in {'greet', 'nod', 'tilt'}:
            if self.pending_look or self.state == 'thinking':
                return
            ok, message = self.motion.start(command)
            if ok:
                self.transition('acting', 'happy' if command != 'tilt' else 'curious', '我在呢！')
            elif not self.motion.busy:
                self.transition('idle', 'confused', message)
        elif command == 'look':
            if self.motion.busy or self.pending_look or self.state == 'thinking':
                return
            if not self.look_client.service_is_ready():
                self.transition('idle', 'confused', '视觉节点尚未就绪。')
                return
            self.pending_look, self.look_id = True, None
            self.look_generation += 1
            generation = self.look_generation
            self.look_deadline = time.monotonic() + 35.
            self.transition('thinking', 'thinking', '让我看一看。')
            self.look_client.call_async(Trigger.Request()).add_done_callback(
                lambda future: self.look_accepted(future, generation))
        elif command == 'sleep' and not self.motion.busy and not self.pending_look and self.state != 'thinking':
            self.transition('sleeping', 'sleepy', '我眯一会儿。')

    def look_accepted(self, future, generation):
        if not self.pending_look or generation != self.look_generation:
            return
        self.pending_look = False
        try:
            response = future.result()
            if not response.success:
                raise ValueError(response.message)
            self.look_id = response.message
            if self.look_id in self.perception_buffer:
                self.apply_perception(self.perception_buffer.pop(self.look_id))
        except Exception:
            if self.state != 'stopped':
                self.transition('idle', 'confused', '现在还看不了，稍后再试吧。')

    def perception(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or value.get('schema_version') != 1 or value.get('source') not in {'mock', 'http', 'fallback'}:
                return
            validate_scene(value['scene'])
            rid = value['request_id']
            if not isinstance(rid, str) or len(rid) > 64:
                return
            # ROS service and topic delivery order is not guaranteed.
            if self.pending_look:
                self.perception_buffer = {rid: value}
            elif rid == self.look_id:
                self.apply_perception(value)
        except (ValueError, TypeError, KeyError):
            pass

    def apply_perception(self, value):
        if self.state != 'thinking' or self.last_perception_id == value['request_id']:
            return
        self.last_perception_id, self.look_id = value['request_id'], None
        scene = value['scene']
        expression = {'neutral': 'idle', 'happy': 'happy', 'curious': 'curious',
                      'confused': 'confused'}[scene['mood']]
        # Perception can affect only a display. It never requests a gesture or BT.
        self.transition('idle', expression, scene['summary'])

    def motion_done(self, success, message):
        if self.state == 'stopped':
            self.publish_state()
        else:
            self.transition('idle', 'happy' if success else 'confused', message)

    def stop(self, request, response):
        self.pending_look, self.look_id = False, None
        self.look_generation += 1
        self.perception_buffer.clear()
        self.transition('stopped', 'stopped', '已暂停互动。')
        self.motion.stop()
        response.success, response.message = True, 'interaction stopped; owned motion cancellation requested'
        return response

    def wake(self, request, response):
        if self.motion.busy:
            response.success, response.message = False, 'waiting for terminal motion result'
        elif self.pending_look or self.state == 'thinking':
            response.success, response.message = False, 'vision request pending'
        else:
            self.transition('idle', 'idle', '我醒啦。')
            response.success, response.message = True, 'awake'
        return response

    def tick(self):
        self.motion.tick()
        if self.state == 'thinking' and time.monotonic() > self.look_deadline:
            self.pending_look, self.look_id = False, None
            self.transition('idle', 'confused', '刚才没看清楚。')
        if (self.state == 'idle' and self.idle_sleep > 0 and not self.motion.busy and
                time.monotonic() - self.changed_at > self.idle_sleep):
            self.transition('sleeping', 'sleepy', '我眯一会儿。')

    def prepare_shutdown(self):
        self.stop(None, Trigger.Response())

    def destroy_node(self):
        self.motion.destroy()
        return super().destroy_node()


def main():
    run(Pet)
