#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose
import rclpy.time
from tf2_ros import TransformListener, Buffer, LookupException

class Dijkstra_planner(Node):
    def __init__(self):
        super().__init__("dijkstra_planner")

        ''' define the required subs and pubs'''
        # map sub for receiving map published by map publisher, also set the durability qos to Transient local
        self.map_qos = QoSProfile(depth = 10)
        self.map_qos = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, self.map_qos)

        # pose sub to get the goal pose to be sent into the path planner
        self.pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        # Then we need 2 publishers to publish the path and visited map to keep track of the path followed by the bot
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 10)

        ''' define the variables required'''
        self.map = None
        self.visited_map = OccupancyGrid()
        # to get tf info about the map and the bot we need to get info about their transforms
        self.tf_buffer = Buffer()
        self.tf_listerner = TransformListener(self.tf_buffer, self)
    
    # map_cb is used to update the visited_map to keep track of the nodes travelled by the bot
    def map_cb(self, map_msg:OccupancyGrid):
        self.map = map_msg
        self.visited_map.header = map_msg.header
        self.visited_map.info = map_msg.info
        self.visited_map.data = [-1] * (map_msg.info.height * map_msg.info.width)

    def goal_cb(self, goal_pose:PoseStamped):
        
        while(map):
            try:
                map_to_base_tf = self.tf_buffer.lookup_transform(self.map.header.frame_id, "base_footprint", rclpy.time.Time())
            except LookupException:
                self.get_logger().error("Could not find transform b/w map and base")
                return
        # now we have the tf from map and base, but we need them in pose to be processed for the pathplanner
        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        path = self.plan(map_to_base_pose, goal_pose)
        self.path_pub.publish(path)

    def plan(self, start, goal):
        pass

def main():
    rclpy.init()
    node = Dijkstra_planner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()