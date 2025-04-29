#include "bumpebot_planning/dijkstra_planner.hpp"
#include <rmw/qos_profiles.h>

#include <vector>

namespace bumperbot_planning
{
    Dijkistra_planner::Dijkistra_planner() : Node("dijkstra_planner")
    {
        rclcpp::QoS map_qos(10);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>
                    ("/map", map_qos, std::bind(&Dijkistra_planner::map_cb, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>
                    ("/goal_pose",10, std::bind(&Dijkistra_planner::goal_cb,this,std::placeholders::_1));

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/dijkstra/path",10);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dijkstra/visited_map",10);

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

      
        }
        void Dijkistra_planner::map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
            map_ = map;
            visited_map.header.frame_id = map_->header.frame_id;
            visited_map.info = map->info;
            visited_map.data = std::vector<int8_t>(map_->info.height * map->info.width,-1 ); 

        }
        void Dijkistra_planner::goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose){
            if(!map_){
                RCLCPP_ERROR(get_logger(),("No map received"));
                return;
            }
            visited_map.data = std::vector<int8_t>(map_->info.height * map_->info.width, -1);
            geometry_msgs::msg::TransformStamped map_to_base_tf ;
            try{
                map_to_base_tf = tf_buffer->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero);
            }catch(const tf2::TransformException & ex){
                RCLCPP_WARN(get_logger(),"No transform fowund bet the map and the bot");
            }
            geometry_msgs::msg::Pose map_to_base_pose;
            map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
            map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
            map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

            auto path_ = plan(map_to_base_pose, goal_pose->pose);
            path_pub_->publish(path_);
        }
        nav_msgs::msg::Path Dijkistra_planner::plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal){
            
        }

        
    } 

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_planning::Dijkistra_planner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
