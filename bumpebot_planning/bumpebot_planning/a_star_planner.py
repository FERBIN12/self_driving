#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose
import rclpy.time
from tf2_ros import TransformListener, Buffer, LookupException

from queue import PriorityQueue


'''
    The only difference between an a* and dijkstra are the heuristic element( i.e the displacement bet the node and goal)  
'''

# To perform the path planner we need graphNode class that wouldhelp us in proiortizing and creating and maintainning nodes
class GraphNode():
    def __init__(self, x, y, cost=0, heuristic = 0, prev = None):
        self.x = x
        self.y = y
        self.cost= cost
        self.heuristic = heuristic
        self.prev = prev

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + self.heuristic)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __hash__(self):
        return hash((self.x, self.y))
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])

class AStar_planner(Node):
    def __init__(self):
        super().__init__("AStar_planner")

        ''' define the required subs and pubs'''
        # map sub for receiving map published by map publisher, also set the durability qos to Transient local
        self.map_qos = QoSProfile(depth = 10)
        self.map_qos = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, self.map_qos)

        # pose sub to get the goal pose to be sent into the path planner
        self.pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        # Then we need 2 publishers to publish the path and visited map to keep track of the path followed by the bot
        self.path_pub = self.create_publisher(Path, "/a_star/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/a_star/visited_map", 10)

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
            self.visited_map.data = [-1] * (self.visited_map.info.height * self.visited_map.info.width)

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
        '''First we start by exploring in all directions so we set a list for that and a PQ for pending nodes and set of visited nodes'''
        
        explore_dirns = [(-1,0), (1,0), (0,1), (0,-1)]
        pending_nodes = PriorityQueue()
        visited_nodes = set()
        '''To process the pose we got from the map, we need to convert the pose from world to grid (i.e World co-ords are received as continous values)
            But for planner we need to descretize them so we convert them to descrete value using:: (robot.pose - map.pose)/ map.resolution]
            This step is crucical because the planner is performed using map(Ocuupancy-grid) so it is crucial to convert them to grid '''
        
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)
        start_node.heuristic = self.manhattan_distance(start_node, goal_node)
        # now we initialize the pending node
        pending_nodes.put(start_node)
        
        # While function is excecutred untill the pending node is empty(i.e every grid explored)
        while (pending_nodes.not_empty and rclpy.Ok()):
            active_node = pending_nodes.get()
            # we have a checker to check if the active node is the present node
            if (active_node == self.world_to_grid(goal)):
                break
            # If the goal is not reached then we explore in every direction i.e) in the first loop 4 diff nodes are created
            for dir_x, dir_y in explore_dirns:
                newNode: GraphNode = active_node + (dir_x, dir_y)

                # now We add another checker to check whether the newNode is present in the visited_node set and the newNode is on the map and the map data is empty(i.e) 0)
                if newNode not in visited_nodes and  self.pose_on_map(newNode) and self.map.data[self.pose_to_cell(newNode) == 0]:
                    # We set the cost between each node to be 1
                    newNode.cost = active_node.cost + 1
                    # here we update the prev pointer of newNode to keep in track of the path followed by our planner
                    newNode.heuristic = self.manhattan_distance(newNode, goal_node)
                    newNode.prev = active_node
                    pending_nodes.put(newNode)
                    visited_nodes.add(newNode)
            # now the node that we explored are added to the visited_map and the value 10 just means that we have travelled in it.
            self.visited_map.data[self.pose_to_cell(active_node)] = -106
            self.map_pub.publish(self.visited_map)
        # while loop comes out when the pending node becomes empty
        path = Path()
        path.header.frame_id = self.map.header.frame_id

        'this loop is exccuted when the goal is reached and is back tracked untill the active_node.prev reaches nullptr'
        while active_node and active_node.prev and rclpy.ok():
            last_pose: Pose = self.grid_to_world(active_node)
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)
            active_node = active_node.prev
        path.poses.reverse(active_node)
        return path 

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int ((pose.position.x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int ((pose.position.y - self.map.info.origin.position.y) / self.map.info.resolution)
        return GraphNode(grid_x, grid_y)
    
    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map.info.resolution + self.map.info.origin.x
        pose.position.y = node.y * self.map.info.resolution + self.map.info.origin.y
        return pose

    # tho I'm not sure abut it ig we are considering the map to be square so that x and y will stay inside the width
    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map.info.width and 0 <= node.y < self.map.info.width
    
    ''' This fn trivial coz Ocuuucpancy grid in nav_msgs process the map as 1d matrix but our node has data in 2D (x,y)
    So we use index = y* width + x'''

    def pose_to_cell(self, node:GraphNode):
        return node.y * self.map.info.width + node.x 
    
    # To estimate the distance between two nodes in a 2D space we use manhattan distance to calculate the ideal one
    def manhattan_distance(self, node: GraphNode, goal_node : GraphNode):
        return abs(goal_node.x - node.x) + abs(goal_node.y - node.y)
        

def main():
    rclpy.init()
    node = AStar_planner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()