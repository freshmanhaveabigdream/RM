#include<tf2_ros/transform_listener.h>
#include<tf2_ros/buffer.h>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include<string>
#include<chrono>
using namespace std::chrono_literals;

class FrameListener:public rclcpp::Node{
public:
    FrameListener(std::string name):Node(name){
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);    //创建TF订阅对象
        timer_ = this->create_wall_timer(1s, std::bind(&FrameListener::on_timer, this));    //创建定时器函数，定期调用回调函数
    }
private:
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    void on_timer(){
        std::string fromFrameRel = "camera_optical_link";   //创建变量存储原坐标系名称
        std::string toFrameRel = "gimbal_odom"; //创建变量存储转换后坐标系名称
        geometry_msgs::msg::TransformStamped transformStamped;  //创建一个带时间戳的存储坐标转换信息（旋转和平移）的变量
        transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);   //求两个坐标系的转换关系的函数
        RCLCPP_INFO(this->get_logger(), "\ntranslation:('%f', '%f', '%f')\nrotation:('%f', '%f', '%f', '%f')\nfrom '%s' to '%s'", transformStamped.transform.translation.x,
        transformStamped.transform.translation.y, transformStamped.transform.translation.z, transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z, transformStamped.transform.rotation.w, fromFrameRel.c_str(), toFrameRel.c_str());    //打印坐标转换信息
        geometry_msgs::msg::PointStamped ps;    //创建存储坐标点信息的变量
        ps.header.frame_id = "gimbal_odom"; //将原坐标名称赋值给变量
        ps.header.stamp = rclcpp::Time(0);
        ps.point.x = 3;
        ps.point.y = 4;
        ps.point.z = 5; //初始化坐标点
        geometry_msgs::msg::PointStamped ps_out;    //创建存储转换后坐标点信息的变量
        ps_out = tf_buffer_->transform(ps, "camera_optical_link");  //求坐标点从自身坐标系转换到“camera_optical_link”坐标系之后的位置信息
        ps = tf_buffer_->transform(ps_out, "gimbal_odom");  //上一步的逆操作
        RCLCPP_INFO(this->get_logger(), "\n转换后的坐标点为:(x=%.2f,y=%.2f,z=%.2f)，坐标系为:%s\n转换回去的坐标点为:(x=%.2f,y=%.2f,z=%.2f)，坐标系为:%s", ps_out.point.x,ps_out.point.y,
        ps_out.point.z,ps_out.header.frame_id.c_str(), ps.point.x,ps.point.y,ps.point.z,ps.header.frame_id.c_str());    //打印转换过程中的坐标信息
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>("FrameListener"));
    rclcpp::shutdown();
    return 0;
}