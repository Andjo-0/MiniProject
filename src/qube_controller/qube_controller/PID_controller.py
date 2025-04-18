#!/home/drinkalotofwater/anaconda3/bin/python

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.parameter import Parameter
from rclpy.node import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class pidController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, reference=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.reference = reference
        self.commanded_velocity = 0  # renamed from voltage
        self.P = 0
        self.I = 0
        self.D = 0
        self.error = 0
        self.lastTime = time.time()
        self.lastError = 0

    def update(self, measuredValue):
        currentTime = time.time()
        deltaT = currentTime - self.lastTime
        maxCommand = 500.0  # maximum velocity command
        maxIntegral = 50.0

        # Calculate error between desired velocity and measured velocity
        self.error = self.reference - measuredValue
        self.P = self.Kp * self.error
        deltaError = self.error - self.lastError
        self.D = self.Kd * (deltaError / deltaT if deltaT > 0 else 0)
        self.I += self.error * deltaT
        self.I = max(min(self.I, maxIntegral), -maxIntegral)

        # Calculate the PID output (velocity command)
        self.commanded_velocity = self.P + self.Ki * self.I + self.D
        self.commanded_velocity = max(min(self.commanded_velocity, maxCommand), -maxCommand)

        self.lastError = self.error
        self.lastTime = currentTime
        return self.commanded_velocity


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.declare_parameter('p', 0.5)
        self.declare_parameter('i', 0.0)
        self.declare_parameter('d', 0.0)
        self.declare_parameter('reference', 2.2)

        self.p = self.get_parameter('p').get_parameter_value().double_value
        self.i = self.get_parameter('i').get_parameter_value().double_value
        self.d = self.get_parameter('d').get_parameter_value().double_value
        self.reference = self.get_parameter('reference').get_parameter_value().double_value

        self.pid = pidController(self.p, self.i, self.d, self.reference)

        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        # Subscribe to joint states and use velocity feedback instead of position
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info('PID Controller Node has been started.')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p' and param.value >= 0.0:
                self.p = param.value
                self.pid.Kp = self.p
                self.get_logger().info(f'Updated P: {self.p}')
            elif param.name == 'i' and param.value >= 0.0:
                self.i = param.value
                self.pid.Ki = self.i
                self.get_logger().info(f'Updated I: {self.i}')
            elif param.name == 'd' and param.value >= 0.0:
                self.d = param.value
                self.pid.Kd = self.d
                self.get_logger().info(f'Updated D: {self.d}')
            elif param.name == 'reference':
                self.reference = param.value
                self.pid.reference = self.reference
                self.get_logger().info(f'Updated Reference (Velocity): {self.reference}')
        return SetParametersResult(successful=True)

    def joint_state_callback(self, msg):
        # Now using velocity feedback instead of position feedback
        try:
            index = msg.name.index('motor_joint')
            measured_velocity = msg.velocity[index]
        except (ValueError, IndexError):
            self.get_logger().error("motor_joint not found in JointState message or velocity not available.")
            return

        commanded_velocity = self.pid.update(measured_velocity)
        vel_msg = Float64MultiArray()
        vel_msg.data = [commanded_velocity]
        self.publisher.publish(vel_msg)
        self.get_logger().info(
            f"[PID] Target Velocity: {self.pid.reference:.2f} | Measured Velocity: {measured_velocity:.2f} | Commanded Vel: {commanded_velocity:.2f}"
        )


def main():
    rclpy.init()
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
