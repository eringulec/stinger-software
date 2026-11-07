"""Controller logic checks with ROS doubles; no DDS/Gazebo verification implied."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch


class Twist:
    def __init__(self):
        self.linear = NS(x=0.0, y=0.0, z=0.0)
        self.angular = NS(x=0.0, y=0.0, z=0.0)


class WrenchStamped:
    def __init__(self):
        self.header = NS(stamp=0, frame_id='')
        self.wrench = NS(force=NS(x=0.0), torque=NS(z=0.0))


class Odometry:
    def __init__(self):
        self.header = NS(stamp=0, frame_id='odom')
        self.child_frame_id = 'base_link'
        self.twist = NS(twist=Twist())


class Time:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1e9)

    def __sub__(self, other):
        return NS(nanoseconds=self.nanoseconds - other.nanoseconds)


class Node:
    def __init__(self, name):
        self.seconds = 0.0
        self.published = []

    def get_clock(self):
        return NS(now=lambda: Time(self.seconds))

    def declare_parameter(self, name, value):
        return NS(value=value)

    def create_publisher(self, msg_type, topic, depth):
        return NS(msg_type=msg_type, topic_name=topic,
                  publish=self.published.append)

    def create_subscription(self, msg_type, topic, callback, depth):
        return NS(msg_type=msg_type, topic_name=topic, callback=callback)


def load_controller():
    modules = {name: ModuleType(name) for name in
               ('rclpy', 'rclpy.node', 'geometry_msgs', 'geometry_msgs.msg',
                'nav_msgs', 'nav_msgs.msg')}
    modules['rclpy.node'].Node = Node
    modules['geometry_msgs.msg'].Twist = Twist
    modules['geometry_msgs.msg'].WrenchStamped = WrenchStamped
    modules['nav_msgs.msg'].Odometry = Odometry
    path = Path(__file__).parents[1] / 'stinger_controller/velocity_controller.py'
    spec = importlib.util.spec_from_file_location('controller_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.VelocityController


VelocityController = load_controller()


class TestVelocityController(unittest.TestCase):
    def setUp(self):
        self.node = VelocityController()
        self.odom = Odometry()

    def step(self, seconds):
        self.node.seconds = seconds
        return self.node.odometry_callback(self.odom)

    def test_tutorial_interfaces(self):
        for attr, topic, msg in [('wrench_pub', '/cmd_wrench', WrenchStamped),
                                 ('cmd_vel_sub', '/cmd_vel', Twist),
                                 ('odom_sub', '/odometry/filtered', Odometry)]:
            endpoint = getattr(self.node, attr)
            self.assertEqual(endpoint.topic_name, topic)
            self.assertIs(endpoint.msg_type, msg)
        self.assertEqual(self.node.cmd_vel_sub.callback.__name__, 'cmd_vel_callback')
        self.assertEqual(self.node.odom_sub.callback.__name__, 'odometry_callback')

    def test_autograder_arithmetic_and_return_order(self):
        n = self.node
        n.prev_time = Time(0)
        n.prev_error_surge = 0.4
        n.Kp_surge, n.Ki_surge, n.Kd_surge = 3, 4, 5
        n.Kp_yaw, n.Ki_yaw, n.Kd_yaw = 6, 7, 8
        command = Twist()
        command.linear.x, command.angular.z = 1.0, 0.5
        n.cmd_vel_callback(command)
        self.odom.twist.twist.linear.x = 0.5
        self.odom.twist.twist.angular.z = 0.2
        error, p, i, d, yaw, out = self.step(0.1)
        for actual, expected in zip((error, p, i, d), (0.5, 1.5, 0.2, 5.0)):
            self.assertAlmostEqual(actual, expected)
        self.odom.twist.twist.linear.x = 0.1
        _, _, _, d, yaw, out = self.step(0.2)
        self.assertAlmostEqual(d, 20.0)
        self.assertAlmostEqual(yaw, 2.22)
        self.assertAlmostEqual(out.wrench.force.x, 23.26)
        self.assertAlmostEqual(out.wrench.torque.z, 2.22)
        self.assertEqual(out.header.frame_id, 'base_link')

    def test_first_sample_primes_derivative(self):
        self.node.cmd_vel.linear.x = 1.0
        self.assertIsNone(self.step(0.0))
        result = self.step(0.1)
        self.assertEqual(result[3], 0.0)

    def test_bad_time_resets_and_publishes_zero(self):
        for delta in (0.0, -0.1, 1e-9, 2.0):
            with self.subTest(delta=delta):
                self.node.prev_time = Time(5)
                self.node.I_surge_total = 5.0
                self.assertIsNone(self.step(5 + delta))
                self.assertIsNone(self.node.prev_time)
                self.assertEqual(self.node.I_surge_total, 0.0)
                self.assertEqual(self.node.published[-1].wrench.force.x, 0.0)

    def test_nonfinite_inputs_zero_and_recover(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            for target in (self.node.cmd_vel.linear, self.node.cmd_vel.angular,
                           self.odom.twist.twist.linear, self.odom.twist.twist.angular):
                attr = 'x' if hasattr(target, 'x') and target in (
                    self.node.cmd_vel.linear, self.odom.twist.twist.linear) else 'z'
                setattr(target, attr, value)
                self.assertIsNone(self.step(1.0))
                self.assertEqual(self.node.published[-1].wrench.torque.z, 0.0)
                setattr(target, attr, 0.0)
        self.step(2.0)
        self.assertIsNotNone(self.step(2.1))

    def test_signed_output_bounds_and_no_windup(self):
        for sign in (-1, 1):
            self.node = VelocityController()
            n = self.node
            n.cmd_vel.linear.x = sign * 100.0
            n.cmd_vel.angular.z = sign * 100.0
            self.step(0.0)
            for index in range(1, 101):
                out = self.step(index * 0.1)[-1]
                self.assertEqual(out.wrench.force.x, sign * n.max_force)
                self.assertEqual(out.wrench.torque.z, sign * n.max_torque)
            self.assertEqual(n.I_surge_total, 0.0)
            self.assertEqual(n.I_yaw_total, 0.0)

    def test_integral_clamp_unwind_and_disabled_gain(self):
        n = self.node
        self.assertEqual(n._integrate(9.9, 1, 1, 1, 0, 10, 60), 10)
        self.assertEqual(n._integrate(10, -1, 1, 1, 100, 10, 60), 9)
        self.assertEqual(n._integrate(-10, 1, 1, 1, -100, 10, 60), -9)
        self.assertEqual(n._integrate(10, 1, 1, 0, 0, 10, 60), 0)

    def test_zero_error_and_reverse_motion(self):
        self.node.cmd_vel.linear.x = -0.3
        self.node.cmd_vel.angular.z = -0.2
        self.odom.twist.twist.linear.x = -0.3
        self.odom.twist.twist.angular.z = -0.2
        self.step(0)
        out = self.step(0.1)[-1]
        self.assertEqual(out.wrench.force.x, 0)
        self.assertEqual(out.wrench.torque.z, 0)


if __name__ == '__main__':
    unittest.main()
