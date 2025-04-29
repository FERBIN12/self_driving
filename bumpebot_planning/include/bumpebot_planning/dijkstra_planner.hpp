#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace bumperbot_planning{
class Dijkistra_planner : public rclcpp::Node{

public:
    Dijkistra_planner();

private:

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // req variables for the planner
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer ;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    void goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose);

    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);


};
}