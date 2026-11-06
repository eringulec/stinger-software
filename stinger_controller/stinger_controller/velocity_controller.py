import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Completed: 5.1.a Velocity Controller Setup
        ### STUDENT CODE HERE
        self.wrench_pub = self.create_publisher(WrenchStamped, '/cmd_wrench', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odometry_callback, 10)

        ### END STUDENT CODE

        # Mock Values, Tune These
        self.Kp_surge = 1
        self.Ki_surge = 0
        self.Kd_surge = 0

        self.Kp_yaw = 1
        self.Ki_yaw = 0
        self.Kd_yaw = 0

        # Completed: 5.1.g Controller Tuning
        # You should be overriding the above values!
        ### STUDENT CODE HERE
        # Initial simulation gains; validate/tune on the actual boat model.
        for axis, gains in [('surge', (20.0, 5.0, 1.0)),
                            ('yaw', (10.0, 2.0, 0.5))]:
            for term, default in zip(('Kp', 'Ki', 'Kd'), gains):
                name = f'{term}_{axis}'
                value = float(self.declare_parameter(name, default).value)
                if not math.isfinite(value) or value < 0.0:
                    raise ValueError(f'{name} must be finite and nonnegative')
                setattr(self, name, value)
        # Independent controller limits; throttle_controller also limits each
        # thruster to 30 N. Combined surge/yaw can still saturate the allocator.
        for name, default in [('max_force', 60.0), ('max_torque', 30.0),
                              ('integral_limit_surge', 10.0),
                              ('integral_limit_yaw', 10.0), ('max_dt', 1.0)]:
            value = float(self.declare_parameter(name, default).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)

        ### END STUDENT CODE

        self.cmd_vel = Twist()
        self.prev_time = None
        self.prev_error_surge = 0
        self.prev_error_yaw = 0
        self.I_surge_total = 0
        self.I_yaw_total = 0

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def odometry_callback(self, msg: Odometry):
        now = self.get_clock().now()
        values = (self.cmd_vel.linear.x, self.cmd_vel.angular.z,
                  msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        # Finite inputs can still overflow when forming an extreme error.
        errors = (values[0] - values[2], values[1] - values[3])
        if not all(math.isfinite(value) for value in values + errors):
            self._reset()
            self._publish_zero(msg)
            return
        if self.prev_time is None:
            self.prev_time = now
            self.prev_error_surge = values[0] - values[2]
            self.prev_error_yaw = values[1] - values[3]
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        if not math.isfinite(dt) or dt < 1e-6 or dt > self.max_dt:
            # A paused/reset simulation clock must not generate a derivative
            # spike or integrate an entire interruption. Re-prime next sample.
            self._reset()
            self._publish_zero(msg)
            return

        output_force = WrenchStamped()
        output_force.header.stamp = msg.header.stamp
        # Twist (and hence force/torque) is expressed in the child/body frame.
        output_force.header.frame_id = msg.child_frame_id or 'base_link'

        error_surge = 0
        # Completed: 5.1.b Error Calculation
        ### STUDENT CODE HERE
        error_surge = self.cmd_vel.linear.x - msg.twist.twist.linear.x

        ### END STUDENT CODE

        P_surge = 0
        # Completed: 5.1.c Proportional Calculation
        ### STUDENT CODE HERE
        P_surge = self.Kp_surge * error_surge

        ### END STUDENT CODE

        I_surge = 0
        # Completed: 5.1.d Integral Calculation
        ### STUDENT CODE HERE
        derivative_surge = self.Kd_surge * (
            error_surge - self.prev_error_surge) / dt
        self.I_surge_total = self._integrate(
            self.I_surge_total, error_surge, dt, self.Ki_surge,
            P_surge + derivative_surge, self.integral_limit_surge, self.max_force)
        I_surge = self.Ki_surge * self.I_surge_total

        ### END STUDENT CODE

        D_surge = 0
        # Completed: 5.1.e Derivative Calculation
        ### STUDENT CODE HERE
        D_surge = derivative_surge
        self.prev_error_surge = error_surge

        ### END STUDENT CODE

        control_surge = self._clip(P_surge + I_surge + D_surge,
                                  self.max_force)
        output_force.wrench.force.x = control_surge

        # Completed: 5.1.f Yaw Control
        ### STUDENT CODE HERE
        error_yaw = self.cmd_vel.angular.z - msg.twist.twist.angular.z
        P_yaw = self.Kp_yaw * error_yaw
        D_yaw = self.Kd_yaw * (error_yaw - self.prev_error_yaw) / dt
        self.I_yaw_total = self._integrate(
            self.I_yaw_total, error_yaw, dt, self.Ki_yaw,
            P_yaw + D_yaw, self.integral_limit_yaw, self.max_torque)
        I_yaw = self.Ki_yaw * self.I_yaw_total
        control_yaw = self._clip(P_yaw + I_yaw + D_yaw, self.max_torque)
        output_force.wrench.torque.z = control_yaw
        self.prev_error_yaw = error_yaw

        ### END STUDENT CODE

        self.wrench_pub.publish(output_force)

        # Ignore this line, only for autograder purposes
        return error_surge, P_surge, I_surge, D_surge, control_yaw, output_force

    @staticmethod
    def _clip(value, limit):
        return max(-limit, min(limit, value))

    def _integrate(self, total, error, dt, gain, other, limit, output_limit):
        if gain == 0.0:
            return 0.0
        candidate = self._clip(total + error * dt, limit)
        output = other + gain * candidate
        # Freeze integration only when it would drive saturation further.
        # Opposite-sign errors can unwind the integral immediately.
        if ((output > output_limit and error > 0.0) or
                (output < -output_limit and error < 0.0)):
            return total
        return candidate

    def _reset(self):
        self.prev_time = None
        self.prev_error_surge = self.prev_error_yaw = 0.0
        self.I_surge_total = self.I_yaw_total = 0.0

    def _publish_zero(self, msg):
        output = WrenchStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = msg.child_frame_id or 'base_link'
        self.wrench_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
