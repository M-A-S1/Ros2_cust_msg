import rclpy
from rclpy.node import Node
from robpos_msg.msg import Robot
import math

class Global(Node):

    def __init__(self):
        super().__init__('info_global')

        # Subscription to the 'topic'
        self.subscription = self.create_subscription(
            Robot,
            'topic',
            self.listener_callback,
            10)
        
        # Publisher for the 'output' topic
        self.publisher_ = self.create_publisher(Robot, 'output', 10)

        # 50 degrees converted to radians for rotation
        self.rotation_angle = math.radians(50)

    def listener_callback(self, msg):
        # Logging the received message in local frame
        self.get_logger().info(f'Received (local frame): Name: {msg.name}, Temp: {msg.temp}, x: {msg.center.x}, y: {msg.center.y}, theta: {msg.center.theta}')

        # Apply 50 degree rotation to the coordinates
        x_global = msg.center.x * math.cos(self.rotation_angle) - msg.center.y * math.sin(self.rotation_angle)
        y_global = msg.center.x * math.sin(self.rotation_angle) + msg.center.y * math.cos(self.rotation_angle)

        # Update the message to reflect the global frame coordinates
        global_msg = Robot()
        global_msg.name = msg.name
        global_msg.temp = msg.temp
        global_msg.center.x = x_global
        global_msg.center.y = y_global
        global_msg.center.theta = msg.center.theta  # Assuming theta is already in the global frame

        # Log and publish the message in the global frame
        self.get_logger().info(f'Publishing (global frame): Name: {global_msg.name}, Temp: {global_msg.temp}, x: {global_msg.center.x}, y: {global_msg.center.y}, theta: {global_msg.center.theta}')
        self.publisher_.publish(global_msg)

def main(args=None):
    rclpy.init(args=args)
    info_global = Global()
    rclpy.spin(info_global)
    info_global.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

