import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import struct
import math

class BrakingNode(Node):
    def __init__(self):
        super().__init__('braking_node')

        # Publisher for braking command
        self.braking_pub = self.create_publisher(Bool, 'braking_actuation', 10)

        # Publisher for Marker (visualizing the braking command)
        self.marker_pub = self.create_publisher(Marker, 'bool_display_marker', 10)

        # Publisher for radar object visualization
        self.object_marker_pub = self.create_publisher(MarkerArray, 'radar_objects_markers', 10)

        # Subscriber for /objects_pcl_radar topic
        self.create_subscription(PointCloud2, '/objects_pcl', self.callback, 10)

        # Threshold for x and y position to trigger braking
        self.x_threshold = 3.0  # meters
        self.y_threshold = 3.0  # meters

        # Threshold for warning zone
        self.warning_x_threshold = 5.0  # meters
        self.warning_y_threshold = 5.0  # meters

        # Active markers tracking
        self.active_markers = {}

    def callback(self, msg: PointCloud2):
        """
        Callback function to process PointCloud2 data for braking actuation.
        """
        # Unpack radar PointCloud2 data
        pc_data = self.unpack_pointcloud(msg)

        braking_command = False
        warning_command = False
        markers = MarkerArray()

        # Create a set of current marker ids
        current_marker_ids = set()

        # Iterate through each radar point in the PointCloud2 message
        for i, obj in enumerate(pc_data):
            x, y, v_x, v_y, exist_prob = obj  # Extract position, velocity, and existence probability

            distance = math.sqrt(x**2 + y**2)  # Calculate the Euclidean distance
            velocity = math.sqrt(v_x**2 + v_y**2)  # Calculate velocity magnitude

            # Decision logic for braking thresholds
            if abs(x) <= self.x_threshold and abs(y) <= self.y_threshold:
                braking_command = True
                self.get_logger().info(f"[BRAKING] Object detected at (x: {x:.2f}, y: {y:.2f}), Distance: {distance:.2f}m, Velocity: {velocity:.2f} m/s")
            elif abs(x) <= self.warning_x_threshold and abs(y) <= self.warning_y_threshold:
                warning_command = True
                self.get_logger().info(f"[WARNING] Object near at (x: {x:.2f}, y: {y:.2f}), Distance: {distance:.2f}m, Velocity: {velocity:.2f} m/s")

            # Create a marker to visualize this radar point
            object_marker = self.create_object_marker(i, x, y, distance, velocity, exist_prob)
            markers.markers.append(object_marker)

            # Add current marker id to the set
            current_marker_ids.add(i)

        # Remove markers that are no longer relevant (delete old ones)
        markers_to_remove = set(self.active_markers.keys()) - current_marker_ids
        for marker_id in markers_to_remove:
            delete_marker = Marker()
            delete_marker.header.frame_id = "base_link"
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "radar_objects"
            delete_marker.id = marker_id
            delete_marker.action = Marker.DELETE
            markers.markers.append(delete_marker)

        # Update active markers with the current markers
        self.active_markers = {i: True for i in current_marker_ids}

        # Publish braking command based on detection
        braking_msg = Bool()
        braking_msg.data = braking_command
        self.braking_pub.publish(braking_msg)

        # Publish visualization Marker for braking status
        self.publish_marker(braking_command, warning_command)

        # Publish radar object markers for visualization
        self.object_marker_pub.publish(markers)

    def unpack_pointcloud(self, msg: PointCloud2):
        """
        Unpack the PointCloud2 message to extract radar data.
        Assumes the data structure has x, y, v_x, v_y, and exist_prob fields.
        """
        pc_data = []
        for i in range(0, len(msg.data), msg.point_step):
            x = struct.unpack_from('f', msg.data, i)[0]
            y = struct.unpack_from('f', msg.data, i + 4)[0]
            v_x = struct.unpack_from('f', msg.data, i + 16)[0]
            v_y = struct.unpack_from('f', msg.data, i + 20)[0]
            exist_prob = struct.unpack_from('f', msg.data, i + 32)[0]
            pc_data.append((x, y, v_x, v_y, exist_prob))
        return pc_data

    def create_object_marker(self, idx, x, y, distance, velocity, exist_prob):
        """
        Create a marker for an individual radar object.
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "radar_objects"
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=1.0)  # Adjust z for better visibility
        marker.scale.z = 0.2  # Text size
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Opaque

        # Set the text to display distance, velocity, and existence probability
        marker.text = f"Dist: {distance:.2f}m\nVel: {velocity:.2f}m/s\nProb: {exist_prob:.2f}"

        return marker

    def publish_marker(self, braking_command: bool, warning_command: bool):
        """
        Publish a marker to visualize braking or warning status in RViz.
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bool_display"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=0.0, y=1.0, z=-0.5)  # Adjust position to make it visible in RViz
        marker.scale.z = 0.3  # Text size

        # Set text and color based on braking and warning commands
        if braking_command:
            marker.text = "Braking"
            marker.color.r = 1.0  # Red for braking
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif warning_command:
            marker.text = "Warning, object close!"
            marker.color.r = 1.0  # Orange for warning
            marker.color.g = 0.5
            marker.color.b = 0.0
        else:
            marker.text = "Clear"
            marker.color.r = 0.0  # Green for no braking
            marker.color.g = 1.0
            marker.color.b = 0.0

        marker.color.a = 1.0  # Set transparency to 1 (opaque)

        # Publish the marker
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BrakingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

