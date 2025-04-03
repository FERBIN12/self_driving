import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from rclpy.qos import QoSProfile

class TrajectoryReaderPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_reader_publisher')

        # Create a publisher for PoseStamped messages to show in RViz
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/trajectory', 10)

        # Create a publisher for MarkerArray to show points in RViz
        self.marker_publisher_ = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        
        # Load trajectory from the CSV file
        self.file_path = '/path/to/your/trajectory.csv'  # Update with the correct CSV file path
        self.trajectory_data = self.load_trajectory_data(self.file_path)
        self.index = 0

        # Initialize tf2 listener for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to periodically publish trajectory data
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10Hz
        
    def load_trajectory_data(self, file_path):
        trajectory = []
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # Skip header row
            for row in reader:
                timestamp = float(row[0])
                x = float(row[1])
                y = float(row[2])
                z = float(row[3])
                trajectory.append((timestamp, x, y, z))
        return trajectory

    def transform_to_odom_frame(self, x, y, z):
        try:
            # Look for the transformation from 'base_link' (or current frame) to 'odom'
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            
            # Create a PoseStamped message
            pose = PoseStamped()
            pose.header.frame_id = "base_link"  # Initial frame for data, you can adjust it
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Transform the pose to 'odom' frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            return transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z
        except Exception as e:
            self.get_logger().warn(f"Transformation failed: {e}")
            return x, y, z  # Return the original coordinates if transformation fails

    def timer_callback(self):
        if self.index < len(self.trajectory_data):
            timestamp, x, y, z = self.trajectory_data[self.index]
            self.index += 1

            # Transform the trajectory data to the odom frame
            x_transformed, y_transformed, z_transformed = self.transform_to_odom_frame(x, y, z)

            # Create Marker for RViz visualization (point visualization)
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "odom"  # Set the target frame to 'odom'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x_transformed
            marker.pose.position.y = y_transformed
            marker.pose.position.z = z_transformed
            marker.scale.x = 0.1  # Size of the point
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 1.0  # Red color
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            # Publish the MarkerArray with a single Marker
            marker_array = MarkerArray()
            marker_array.markers.append(marker)
            self.marker_publisher_.publish(marker_array)
            
            # Optionally, you can also publish the PoseStamped message
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "odom"
            pose.pose.position.x = x_transformed
            pose.pose.position.y = y_transformed
            pose.pose.position.z = z_transformed
            self.pose_publisher_.publish(pose)

        else:
            self.get_logger().info("Trajectory reading complete.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReaderPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
