import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import random
import math

class VehicleSimulatorNode(Node):
    def __init__(self):
        super().__init__('vehicle_simulator')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.publish_rate = self.get_parameter('publish_rate').value

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)

        # Publisher
        self.velocity_publisher = self.create_publisher(TwistStamped, 'velocity', 10)

        # Velocity and yaw rate ranges
        self.ego_velocity_range = (-81.92, 81.9175)  # m/s
        self.yaw_rate_range = (-math.radians(163.84), math.radians(163.83))  # rad/s

        self.get_logger().info("Vehicle simulator node initialized.")

    def publish_velocity(self):
        forward_velocity = 2.0 
        yaw_rate = 0.0 
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = forward_velocity
        twist_msg.twist.angular.z = yaw_rate

        self.velocity_publisher.publish(twist_msg)
        self.get_logger().info(f"Published velocity: {forward_velocity:.2f} m/s, yaw rate: {math.degrees(yaw_rate):.2f} deg/s")


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting vehicle sim node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

