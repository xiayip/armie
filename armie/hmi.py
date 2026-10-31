"""Display-only adapter. Existing CAN ROS MODE v1 has no face command.

ROS JSON and optional UDP are an Armie display contract, not claimed compatible
with the current HMI firmware. No serial port, CAN socket or boot line is opened.
"""
import json
import socket
import time

from rclpy.node import Node
from std_msgs.msg import String

from .common import EXPRESSIONS, STATE_QOS, finite, param, publish, run

FACE_IDS = {name: i for i, name in enumerate(
    ['idle', 'happy', 'curious', 'thinking', 'sleepy', 'confused', 'stopped'])}


def display_packet(value):
    if not isinstance(value, dict) or value.get('schema_version') != 1:
        raise ValueError('invalid expression schema')
    expression = value.get('expression')
    if expression not in EXPRESSIONS or not isinstance(value.get('text'), str):
        raise ValueError('invalid expression')
    seq = value.get('sequence')
    if type(seq) is not int or seq < 0:
        raise ValueError('invalid sequence')
    ttl = int(finite(value.get('ttl_ms'), 100, 5000, 'ttl_ms'))
    return dict(protocol='armie-face', version=1, sequence=seq,
                face_id=FACE_IDS[expression], expression=expression,
                text=value['text'][:240], ttl_ms=ttl)


class Hmi(Node):
    def __init__(self):
        super().__init__('armie_hmi')
        self.transport = param(self, 'hmi_transport', 'mock')
        if self.transport not in {'mock', 'udp'}:
            raise ValueError('hmi_transport must be mock or udp')
        host = param(self, 'hmi_udp_host', '127.0.0.1')
        port = int(finite(param(self, 'hmi_udp_port', 8766), 1, 65535, 'UDP port'))
        self.address = host, port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.transport == 'udp' else None
        if self.socket:
            self.socket.setblocking(False)
        self.pub = self.create_publisher(String, '/armie/hmi/tx', STATE_QOS)
        self.status_pub = self.create_publisher(String, '/armie/hmi/status', STATE_QOS)
        self.sub = self.create_subscription(String, '/armie/expression', self.face, STATE_QOS)
        self.deadline, self.stale = time.monotonic() + 2.5, False
        self.last_sequence = 0
        self.timer = self.create_timer(.1, self.tick)

    def send(self, packet):
        publish(self.pub, packet)
        status = 'mock' if self.socket is None else 'sent_unacknowledged'
        if self.socket:
            try:
                self.socket.sendto(json.dumps(packet, ensure_ascii=False).encode(), self.address)
            except OSError:
                status = 'transport_error'
        publish(self.status_pub, dict(schema_version=1, transport=self.transport, status=status,
                                     firmware_support='not_implemented', stale=self.stale))

    def face(self, msg):
        try:
            packet = display_packet(json.loads(msg.data))
        except (ValueError, TypeError):
            return
        self.deadline = time.monotonic() + packet['ttl_ms'] / 1000.
        self.stale, self.last_sequence = False, packet['sequence']
        self.send(packet)

    def tick(self):
        if not self.stale and time.monotonic() > self.deadline:
            self.stale = True
            self.send(display_packet(dict(schema_version=1, sequence=self.last_sequence + 1,
                                          expression='confused', text='连接中断', ttl_ms=2500)))

    def destroy_node(self):
        if self.socket:
            self.socket.close()
        return super().destroy_node()


def main():
    run(Hmi)
