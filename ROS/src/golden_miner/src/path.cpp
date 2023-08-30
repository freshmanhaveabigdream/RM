#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "mine_interfaces/msg/ore.hpp"
#include "mine_interfaces/msg/ore_array.hpp"

class Visualization: public rclcpp::Node{
private:
    rclcpp::Subscription<mine_interfaces::msg::Ore>::SharedPtr ore; // 定义一个接受当前目标矿石信息的订阅者
    rclcpp::Subscription<mine_interfaces::msg::OreArray>::SharedPtr ore_array; // 定义一个接受当前所有矿石信息的订阅者
    nav_msgs::msg::Path path;   // 定义路径
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;   // 定义路径发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ball_publisher;   // 定义球发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_publisher;   // 定义箭头发布者
    geometry_msgs::msg::Point last_location;    // 定义采矿人上一次的位置
    void path_callback(const mine_interfaces::msg::Ore &ore){   //接收到单个矿石的回调函数
        // 画箭头部分
        visualization_msgs::msg::Marker arrow_msg;
        arrow_msg.type = visualization_msgs::msg::Marker::ARROW;
        arrow_msg.scale.x = 0.5;
        arrow_msg.scale.y = 1;
        arrow_msg.scale.z = 0;
        arrow_msg.color.r = 0.0;
        arrow_msg.color.g = 0.0;
        arrow_msg.color.b = 1.0;
        arrow_msg.color.a = 0.5;
        arrow_msg.action = visualization_msgs::msg::Marker::ADD;
        arrow_msg.lifetime = rclcpp::Duration::from_seconds(0.5);
        arrow_msg.header.stamp = this->now();
        arrow_msg.header.frame_id = "odom";
        arrow_msg.points.clear();
        arrow_msg.points.push_back(last_location);
        arrow_msg.points.push_back(ore.location.position);
        arrow_msg.pose.orientation.w = 1.0;
        visualization_msgs::msg::MarkerArray arrow;
        arrow.markers.push_back(arrow_msg);
        last_location = ore.location.position;
        arrow_publisher->publish(arrow);
        
        // 画路径部分
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
    void ball_callback(const mine_interfaces::msg::OreArray &ore_array){    //接 受到矿石数组的回调函数
        int num = ore_array.header.stamp.sec;    //获得目前矿石数量
        // 画矿石部分
        visualization_msgs::msg::MarkerArray makerArrayMsg;
        for(int i=0; i < num; i++){
            visualization_msgs::msg::Marker maker;
            maker.id = ore_array.ores[i].id.data;
            maker.type = visualization_msgs::msg::Marker::SPHERE;
            maker.scale.x = 1;
            maker.scale.y = 1;
            maker.scale.z = 1;
            if(ore_array.ores[i].type.data == "金矿"){
                maker.color.r = 1.0;
                maker.color.g = 0.843;
                maker.color.b = 0.0;
            }else{
                maker.color.r = 0.753;
                maker.color.g = 0.753;
                maker.color.b = 0.753;
            }
            maker.pose.position.x = ore_array.ores[i].location.position.x;
            maker.pose.position.y = ore_array.ores[i].location.position.y;
            maker.pose.position.z = ore_array.ores[i].location.position.z;
            maker.color.a = 1.0;
            maker.header.stamp = this->now();
            maker.header.frame_id = "odom";
            maker.pose.orientation = ore_array.ores[i].location.orientation;
            maker.lifetime = rclcpp::Duration::from_seconds(0.5);
            makerArrayMsg.markers.push_back(maker);
        }
        ball_publisher->publish(makerArrayMsg);
    }
public:
    Visualization(std::string name):Node(name){
        ore = this->create_subscription<mine_interfaces::msg::Ore>("cur_ore", 10, std::bind(&Visualization::path_callback, this, std::placeholders::_1));
        path_publisher = this->create_publisher<nav_msgs::msg::Path>("trajectory_odom", 10);
        ore_array = this->create_subscription<mine_interfaces::msg::OreArray>("location", 10, std::bind(&Visualization::ball_callback, this, std::placeholders::_1));
        ball_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("ball", 1);
        arrow_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("arrow_array", 10);
        last_location.x = 0;
        last_location.y = 0;
        last_location.z = 0;
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Visualization>("Visualization");
    rclcpp::spin(node);
    rclcpp::shutdown();
}