from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import rclpy

class GoToGoal(Node):
    def __init__(self):
        super().__init__("controller")

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0 
        self.goal = [1.8, 0.0]  # Goal position (x, y)
        self.pos_x = 0.0

        self.kp = 0.5  # Proportional gain
        self.kd = 1.0
        self.error = [0.0, 0.0]
        self.prev_error = [0.0, 0.0]
        self.diff_error = [0.0, 0.0]

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.timer = self.create_timer(
            0.01,  # 10 ms
            self.update_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

    def update_callback(self):
        self.pid_control()

    def pid_control(self):
        self.error[0] = self.goal[0] - self.pos_x
        self.diff_error[0] = self.error[0] - self.prev_error[0]
        self.linear_velocity = self.kp * self.error[0] + self.kd * self.diff_error[0]
        self.prev_error[0] = self.error[0]

        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
