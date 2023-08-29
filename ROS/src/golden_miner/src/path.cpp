#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "mine_interfaces/msg/ore.hpp"

class Visualization: public rclcpp::Node{
private:
    rclcpp::Subscription<mine_interfaces::msg::Ore>::SharedPtr ore; //定义一个接受当前目标矿石信息的订阅者
    nav_msgs::msg::Path path;   //定义路径
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;   //定义路径发布者
    void callback(const mine_interfaces::msg::Ore &ore){
        geometry_msgs::msg::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = ore.location.position.x;
        this_pose_stamped.pose.position.y = ore.location.position.y;
        this_pose_stamped.pose.position.z = ore.location.position.z;

        this_pose_stamped.pose.orientation = ore.location.orientation;

        this_pose_stamped.header.stamp = this->now();
        this_pose_stamped.header.frame_id = "odom";
        
        path.poses.push_back(this_pose_stamped);

        path.header.stamp = this->now();
        path.header.frame_id = "odom";
        path_publisher->publish(path);
    }
public:
    Visualization(std::string name):Node(name){
        ore = this->create_subscription<mine_interfaces::msg::Ore>("cur_ore", 10, std::bind(&Visualization::callback, this, std::placeholders::_1));
        path_publisher = this->create_publisher<nav_msgs::msg::Path>("trajectory_odom", 10);
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Visualization>("Visualization");
    rclcpp::spin(node);
    rclcpp::shutdown();
}