import rclpy
from rclpy.node import Node
from robpos_msg.msg import Robot

class Local(Node):

    def __init__(self):
        super().__init__('info_local')
        self.publisher_ = self.create_publisher(Robot, 'topic', 10)
        timer_period = 4  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Robot()
        msg.name = "Robot_" + str(self.i)
        msg.temp = 36.5 + self.i  # some arbitrary temperature
        
        # Ensure the Pose2D fields are assigned as floats
        msg.center.x = float(self.i)  # x should be a float
        msg.center.y = float(self.i + 1)  # y should be a float
        msg.center.theta = 30.0 + self.i # theta should be a float

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Robot Name: {msg.name}, Temperature: {msg.temp}, x: {msg.center.x}, y: {msg.center.y}, theta: {msg.center.theta}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    info_local = Local()
    rclpy.spin(info_local)
    info_local.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

