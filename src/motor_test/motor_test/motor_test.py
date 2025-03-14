"""
Motor Input Publisher Node for ROS 2

This ROS 2 node (`InputPublisher`) allows users to manually input motor positions
via the terminal, which are then published to the `/maxon_bringup/set_all_states`
topic as `MotorStates` messages.

Functionality:
- Prompts the user for four motor position values.
- Constructs `MotorState` messages for each motor.
- Publishes the motor states to the designated topic.
- Continuously runs until interrupted (Ctrl+C).

Usage:
Run with $ ros2 run motor_test motor_test
Ensure that the necessary `maxon_epos_msgs` package is installed.
Ensure that the python environment is activated, ROS2 workspace is built, and the install/setup.bash is sourced

Author: Connor Guzikowski
Date: 2/01/2025
"""


import rclpy
from rclpy.node import Node
from maxon_epos_msgs.msg import MotorState, MotorStates
from std_msgs.msg import String

class InputPublisher(Node):
    def __init__(self):
        super().__init__('motor_test')
        self.publisher_ = self.create_publisher(MotorStates, '/maxon_bringup/set_all_states', 10)
        self.get_logger().info("Node started. Type input to publish (Ctrl+C to exit)")
        self.publish_input()

    def publish_input(self):
        """Continuously prompt for input and publish to the topic."""
        try:
            while rclpy.ok():
                user_input = input("Enter motor 1 position: ")  # Get input for motor 1 and set message data
                msg = MotorStates()
                msg.header.stamp = self.get_clock().now().to_msg()
                
                # NOTE: The velocity and current fields are arbitrary and not currently used. Values need to be set to have no errors 
                motor1 = MotorState(motor_name='1', velocity=5.5, position=float(user_input), current=2.1)
                motor1.header.stamp = self.get_clock().now().to_msg()
                
                user_input = input("Enter motor 2 position: ")  # Get input for motor 2 and set message data
                motor2 = MotorState(motor_name='2', velocity=5.5, position=float(user_input), current=2.1)
                motor2.header.stamp = self.get_clock().now().to_msg()
                
                user_input = input("Enter motor 3 position: ")  # Get input for motor 3 and set message data
                motor3 = MotorState(motor_name='3', velocity=5.5, position=float(user_input), current=2.1)
                motor3.header.stamp = self.get_clock().now().to_msg()
                
                user_input = input("Enter motor 4 position: ")  # Get input for motor 4 and set message data
                motor4 = MotorState(motor_name='4', velocity=5.5, position=float(user_input), current=2.1)
                motor4.header.stamp = self.get_clock().now().to_msg()                
                
                msg.states = [motor1, motor2, motor3, motor4]
                self.publisher_.publish(msg)  # Publish message
                self.get_logger().info(f"Published: '{[motor1, motor2, motor3, motor4]}'")
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down node...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InputPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
