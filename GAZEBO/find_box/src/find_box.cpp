// 录播观看时间2023年8月27日
// 学长看到这儿，辛苦了

#include "opencv4/opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/int32.hpp"
#include<chrono>
#include<string>
#include<memory>

class find_box:public rclcpp::Node
{
public:
    find_box(): Node("find_box")
    {
        //读取camera画面
        subscription_camera = this->create_subscription<sensor_msgs::msg::Image>("camera", 10, std::bind(&find_box::callback_camera, this, std::placeholders::_1));
        subscription_color = this->create_subscription<std_msgs::msg::Int32>("color", 10, std::bind(&find_box::callback_color, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        hsv = {
            std::make_pair(cv::Scalar(29, 8, 90), cv::Scalar(33, 255, 200)),   //box_0
            std::make_pair(cv::Scalar(150, 238, 95), cv::Scalar(162, 255, 185)), //box_1
            std::make_pair(cv::Scalar(89, 140, 95), cv::Scalar(100, 255, 128)),   //box_2
            std::make_pair(cv::Scalar(101, 135, 70), cv::Scalar(102, 147, 133)),  //box
        };
        color_choice.data = 0;  // 默认选择box_0
    }
private:
    std_msgs::msg::Int32 color_choice;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> hsv;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_camera; //定义相机订阅者
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_color;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    // 根据目标颜色物体，处理图像
    cv::Mat tackle_img(cv::Mat img, int color){
        cv::Mat img_hsv;
        // 高斯降噪
        cv::GaussianBlur(img, img, cv::Size(7, 7), 0, 0);
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(img_hsv, this->hsv[color].first, this->hsv[color].second, img_hsv);
        return img_hsv;
    }
    void find_center(cv::Mat& img, double& center_x, double& center_y, double& width, bool& isFound){
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // 判断是否找到目标颜色正方体
        if(contours.size()){
            isFound = true;
            cv::RotatedRect rect = cv::minAreaRect(contours.back());
            cv::Rect r = rect.boundingRect();
            // 得到中心点以及矩形宽度
            center_x = rect.center.x;
            center_y = rect.center.y;
            width = r.width;
        }else{
            isFound = false;
        }
    }
    void callback_color(const std_msgs::msg::Int32 & color){
        this->color_choice = color;
    }
    // 订阅相机信息的回调函数
    void callback_camera(const sensor_msgs::msg::Image & msg){
        cv_bridge::CvImagePtr CVPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = CVPtr->image;
        cv::Mat pro_img = tackle_img(img, color_choice.data);
        double center_x, center_y, width;
        bool isFound;
        find_center(pro_img, center_x, center_y, width, isFound);
        cv::imshow("camera_img", img);
        geometry_msgs::msg::Twist pub;
        if(isFound){
            pub.angular.z = (pro_img.cols / 2 - center_x) * 0.002;   //角速度
            pub.linear.x = (300 - width) * 0.005; //线速度
        }else{
            pub.angular.z = 0.5;
            pub.linear.x = 0.5;
        }
        publisher_->publish(pub);
        cv::waitKey(1);
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<find_box>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}