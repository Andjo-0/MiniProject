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
        # lastTime is being initialized, at the moment when the class object is being constructed
        self.lastTime = time.time()
        self.lastError = 0
        
    #updating the velocity out of the controller 
    def update(self, measuredValue):
        currentTime = time.time()
        deltaT = currentTime - self.lastTime
        maxCommand = 500.0  # maximum velocity command
        maxIntegral = 50.0 # does not allow 

        # Calculate error between desired velocity and measured velocity
        self.error = self.reference - measuredValue
        
        self.P = self.Kp * self.error
        
        deltaError = self.error - self.lastError
        
        if deltaT > 0:
            self.D = self.Kd * (deltaError / deltaT)
        else:
            self.D = 0
            
        self.I += self.error * deltaT
        # limits the value of I to be smaller then the value of maxIntegral
        if self.I > maxIntegral:
            self.I = maxIntegral
        # The idea of including the protection against the negative value of I comes from CHATGPT.
        elif self.I < -maxIntegral:
            self.I = -maxIntegral

        # Calculate the PID output (velocity command)
        self.commanded_velocity = self.P + self.Ki * self.I + self.D

        # Make sure that the velocity out of the controller is in limits of the maxCommand velocity
        if self.commanded_velocity > maxCommand:
            self.commanded_velocity = maxCommand
        elif self.commanded_velocity < -maxCommand:
            self.commanded_velocity = -maxCommand

        self.lastError = self.error
        self.lastTime = currentTime
        return self.commanded_velocity


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        # default values for pid
        self.declare_parameter('p', 0.5)
        self.declare_parameter('i', 0.0)
        self.declare_parameter('d', 0.0)
        # target value for velocity
        self.declare_parameter('reference', 2.2) 
        # getting the parameters from the controller
        self.p = self.get_parameter('p').get_parameter_value().double_value
        self.i = self.get_parameter('i').get_parameter_value().double_value
        self.d = self.get_parameter('d').get_parameter_value().double_value
        self.reference = self.get_parameter('reference').get_parameter_value().double_value

        #Creates the pid object connected to this node obj
        self.pid = pidController(self.p, self.i, self.d, self.reference)
        #creates the publisher
        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        # Subscribe to joint states and use velocity feedback instead of position
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info('PID Controller Node has been started.')

    def parameter_callback(self, params):
        # activtes the callback when a parameter is changed at runtime
        for param in params:
            # update p
            if param.name == 'p' and param.value >= 0.0:
                self.p = param.value
                self.pid.Kp = self.p
                self.get_logger().info(f'Updated P: {self.p}')
            # update i
            elif param.name == 'i' and param.value >= 0.0:
                self.i = param.value
                self.pid.Ki = self.i
                self.get_logger().info(f'Updated I: {self.i}')
            # update d
            elif param.name == 'd' and param.value >= 0.0:
                self.d = param.value
                self.pid.Kd = self.d
                self.get_logger().info(f'Updated D: {self.d}')
            # update target value
            elif param.name == 'reference':
                self.reference = param.value
                self.pid.reference = self.reference
                self.get_logger().info(f'Updated Reference (Velocity): {self.reference}')
        return SetParametersResult(successful=True)

    def joint_state_callback(self, msg):
        
        try:
            # store the index nr of motor joint
            index = msg.name.index('motor_joint')
            # get the vel of the motor joint
            measured_velocity = msg.velocity[index]
            
        except (ValueError, IndexError):
            self.get_logger().error("motor_joint not found in JointState message or velocity not available.")
            return
        # find new velocity using the pid 
        commanded_velocity = self.pid.update(measured_velocity)
        vel_msg = Float64MultiArray()
        vel_msg.data = [commanded_velocity]
        self.publisher.publish(vel_msg)
        self.get_logger().info(
            f"[PID] Target Velocity: {self.pid.reference:.2f} | Measured Velocity: {measured_velocity:.2f} | Commanded Vel: {commanded_velocity:.2f}"
        )


def main():
    # init ros
    rclpy.init()
    node = PIDControllerNode()
    # keep the node running 
    rclpy.spin(node)
    # delete the node when it has stop "spinning"
    node.destroy_node()
    # turn of the ros
    rclpy.shutdown()


if __name__ == "__main__":
    main()
