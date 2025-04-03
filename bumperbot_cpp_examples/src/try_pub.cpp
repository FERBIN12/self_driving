#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>
#include <sstream>
#include <vector>

class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher() : Node("odom_publisher") {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_data", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/odom_markers", 10);
        
        loadOdomData("/home/ferbin/selfdriving_bot/trajectory.csv");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdomPublisher::publishData, this));
    }

private:
    struct OdomEntry {
        double timestamp, x, y, z;
    };
    std::vector<OdomEntry> odom_data_;
    size_t index_ = 0;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void loadOdomData(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file");
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            OdomEntry entry;
            if (iss >> entry.timestamp >> entry.x >> entry.y >> entry.z) {
                odom_data_.push_back(entry);
            }
        }
        file.close();
    }

    void publishData() {
        if (index_ >= odom_data_.size()) return;
        
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        msg.pose.pose.position.x = odom_data_[index_].x;
        msg.pose.pose.position.y = odom_data_[index_].y;
        msg.pose.pose.position.z = odom_data_[index_].z;
        
        odom_pub_->publish(msg);
        publishMarkers(odom_data_[index_]);
        index_++;
    }

    void publishMarkers(const OdomEntry& entry) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "odom_path";
        marker.id = index_;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = entry.x;
        marker.pose.position.y = entry.y;
        marker.pose.position.z = entry.z;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
