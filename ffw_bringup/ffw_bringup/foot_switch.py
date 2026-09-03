#!/usr/bin/env python3

import fcntl
import os
import select
import struct
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8


EVIOCGRAB = 0x40044590
EV_KEY = 0x01
INPUT_EVENT_FORMAT = 'llHHi'
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FORMAT)

KEY_LEFT = 30
KEY_MIDDLE = 48
KEY_RIGHT = 46


class FootSwitchReader(Node):
    def __init__(self):
        super().__init__('foot_switch_node')
        self.device = self.declare_parameter(
            'device',
            '/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd',
        ).value
        self.save_pose_id = self.declare_parameter('save_pose_id', 4).value
        self.debounce_sec = self.declare_parameter('debounce_sec', 0.5).value
        self.fd = None
        self.last_event_time = {}
        self.middle_pressed = False

        self.deadzone_client = self.create_client(
            SetParameters, '/leader/joystick_controller/set_parameters')
        self.left_command_pub = self.create_publisher(UInt8, '/leader/left_command', 1)
        self.right_command_pub = self.create_publisher(UInt8, '/leader/right_command', 1)
        self.middle_pedal_pub = self.create_publisher(
            Bool, '/leader/foot_switch/middle_pedal', 1)

    def open_device(self):
        while rclpy.ok():
            try:
                self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
                fcntl.ioctl(self.fd, EVIOCGRAB, 1)
                self.get_logger().info(f'Opened and grabbed foot pedal: {self.device}')
                return True
            except (FileNotFoundError, PermissionError, OSError) as error:
                if self.fd is not None:
                    os.close(self.fd)
                    self.fd = None
                self.get_logger().warning(
                    f'Foot pedal unavailable ({error}); retrying in 1 second')
                time.sleep(1.0)
        return False

    def close_device(self):
        if self.fd is None:
            return
        try:
            fcntl.ioctl(self.fd, EVIOCGRAB, 0)
        except OSError:
            pass
        os.close(self.fd)
        self.fd = None

    def is_debounced(self, code, value):
        key = (code, value)
        now = time.monotonic()
        if now - self.last_event_time.get(key, 0.0) < self.debounce_sec:
            return False
        self.last_event_time[key] = now
        return True

    def publish_home(self, publisher, side):
        msg = UInt8()
        msg.data = self.save_pose_id
        publisher.publish(msg)
        self.get_logger().info(f'{side} pedal requested home pose {self.save_pose_id}')

    def set_deadzone(self, value):
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='deadzone',
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=value,
            ),
        )]
        self.deadzone_client.call_async(request)

    def handle_event(self, code, value):
        if code not in (KEY_LEFT, KEY_MIDDLE, KEY_RIGHT):
            return
        if value not in (0, 1) or not self.is_debounced(code, value):
            return

        if code == KEY_LEFT and value == 1:
            self.publish_home(self.left_command_pub, 'Left')
        elif code == KEY_RIGHT and value == 1:
            self.publish_home(self.right_command_pub, 'Right')
        elif code == KEY_MIDDLE:
            self.middle_pressed = value == 1
            msg = Bool()
            msg.data = self.middle_pressed
            self.middle_pedal_pub.publish(msg)
            self.set_deadzone(0.95 if self.middle_pressed else 1.0)
            state = 'pressed' if self.middle_pressed else 'released'
            self.get_logger().info(f'Middle pedal {state}')

    def run(self):
        while rclpy.ok():
            readable, _, _ = select.select([self.fd], [], [], 0.2)
            if not readable:
                continue
            try:
                data = os.read(self.fd, INPUT_EVENT_SIZE)
            except BlockingIOError:
                continue
            if len(data) != INPUT_EVENT_SIZE:
                continue
            _, _, event_type, code, value = struct.unpack(INPUT_EVENT_FORMAT, data)
            if event_type == EV_KEY:
                self.handle_event(code, value)


def main(args=None):
    rclpy.init(args=args)
    node = FootSwitchReader()
    try:
        if node.open_device():
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_device()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
