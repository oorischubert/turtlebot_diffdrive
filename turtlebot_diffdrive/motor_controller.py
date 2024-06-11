import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from turtlebot.motorComms import MotorController, MotorMessage
from turtlebot.configuration import *
import transforms3d
import sys
import time

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        # esp comms init 
        self.esp = MotorController()
        self.motorMessage = MotorMessage()

        connectBool = self.esp.initHandshake()
        if not connectBool: 
            self.get_logger().info("Error opening serial port!")
        else: 
            self.get_logger().info(f"Using serial port: {self.esp.port}")
        self.esp.send_velocity_command(0.0,0.0)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10  # message queue size
        )

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

    def listener_callback(self, msg):
        start_time = time.time()
        try:
            self.esp.send_velocity_command(float(msg.linear.x), float(msg.angular.z))
            self.esp.read_serial_data(self.motorMessage)
        except Exception as e:
            self.get_logger().info(f"Error {e}")

        end_time = time.time()

        self.get_logger().info(f"Linear Vel: {self.motorMessage.velocity_x}, Requested: {msg.linear.x}")
        self.get_logger().info(f"Angular Vel: {self.motorMessage.velocity_angular}, Requested: {msg.angular.z}")
        self.get_logger().info(f"Callback duration: {end_time - start_time:.4f} seconds")

        self.publish_odometry()
    
    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # Set position
        odom.pose.pose.position = Point(x=self.motorMessage.position_x, y=self.motorMessage.position_y, z=0.0)
        # Convert Euler angles to quaternion and set orientation
        quaternion = transforms3d.euler.euler2quat(0, 0, self.motorMessage.position_angular)
        odom.pose.pose.orientation = Quaternion(x=quaternion[1], y=quaternion[2], z=quaternion[3], w=quaternion[0])
        odom.twist.twist.linear.x = self.motorMessage.velocity_x
        odom.twist.twist.angular.z = self.motorMessage.velocity_angular
        self.odom_publisher.publish(odom)

    def destroy_node(self):
        super().destroy_node()  # Don't forget to call the base implementation

def main(args=None):
    rclpy.init(args=args)
    diff_drive_node = DiffDriveController()

    executor = MultiThreadedExecutor()
    executor.add_node(diff_drive_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        diff_drive_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()