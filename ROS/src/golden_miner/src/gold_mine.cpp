#include "rclcpp/rclcpp.hpp"
#include "mine_interfaces/msg/ore.hpp"
#include "mine_interfaces/msg/ore_array.hpp"
#include "mine_interfaces/srv/mining.hpp"
#include<vector>
#include<stdlib.h>
#include<cmath>

std::vector<mine_interfaces::msg::Ore> random_generation(int n){ //用于生成随机的矿石坐标信息
    const float pi = 3.14;
    float phi, theta, r;
    std::vector<mine_interfaces::msg::Ore> result;
    mine_interfaces::msg::Ore temp;
    for(int i = 0; i < n; i++){
        phi = drand48() * 2 * pi;
        theta = drand48() * 2 * pi;
        r = drand48() * 10; //使用极坐标随机生成三维点，半径为10
        int type = rand() % 2;  //矿石类型的随机数
        if(type == 0){
            temp.type.data = "金矿";
            temp.value.data = 80.8;
        }else{
            temp.type.data = "银矿";
            temp.value.data = 40.4;
        }
        temp.number.data = i;
        temp.location.position.x = r * sin(phi) * sin(theta);
        temp.location.position.y = r * sin(phi) * cos(theta);
        temp.location.position.z = r * cos(phi);
        result.push_back(temp);
    }
    return result;
}

class MineNode:public rclcpp::Node  //自定义矿界类
{
private:
    int ore_number = -1;    //声明矿石默认值为-1
    rclcpp::Publisher<mine_interfaces::msg::OreArray>::SharedPtr mine;   //定义一个发布者，发布信息类型为自定义矿石信息数组
    rclcpp::TimerBase::SharedPtr timer_;    //定义时间循环
    std::vector<mine_interfaces::msg::Ore> information;  //设置存储矿石信息的数组
    rclcpp::Service<mine_interfaces::srv::Mining>::SharedPtr mining_server;  //声明服务端
    rclcpp::Publisher<mine_interfaces::msg::Ore>::SharedPtr cur_ore;   //定义发布当前目标矿石的信息的发布者，用来可视化路径
    float totalvalue;
    void timer_callback()   //定义一个时间回调函数
    {
        mine_interfaces::msg::OreArray message;  //定义一个矿石信息数组变量，用于发布
        message.ores = information;
        message.header.stamp.sec = information.size();
        RCLCPP_INFO(this->get_logger(), "剩余矿石数量：%ld", information.size());
        mine->publish(message); //发布矿石信息
    }
    void mining_callback(const mine_interfaces::srv::Mining::Request::SharedPtr request,
                        mine_interfaces::srv::Mining::Response::SharedPtr response){   //创建采矿请求的回调函数
        RCLCPP_INFO(this->get_logger(), "收到采矿请求");
        response->location.x = information[request->number.data].location.position.x;
        response->location.y = information[request->number.data].location.position.y;
        response->location.z = information[request->number.data].location.position.z;
        totalvalue += information[request->number.data].value.data;
        response->total_value.data = totalvalue;
        information.erase(information.begin() + request->number.data);
        for(unsigned int i = request->number.data; i < information.size(); i++){
            information[i].number.data--;
        }
        mine_interfaces::msg::Ore ore;
        ore.location.position = response->location;
        cur_ore->publish(ore);
    }
public:
    MineNode(std::string name):Node(name){
        RCLCPP_INFO(this->get_logger(), "发布者创建成功！");
        mine = this->create_publisher<mine_interfaces::msg::OreArray>("location", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MineNode::timer_callback, this));   //创建500ms周期的一个循环，每一个周期调一次回调函数
        mining_server = this->create_service<mine_interfaces::srv::Mining>("mining", std::bind(&MineNode::mining_callback, this, std::placeholders::_1, std::placeholders::_2));    //定义服务端
        cur_ore = this->create_publisher<mine_interfaces::msg::Ore>("cur_ore", 10);
        this->declare_parameter<std::int64_t>("ore_number", ore_number);
        while(ore_number == -1){    // 一直循环直到launch文件中对ore_number的设置生效
            this->get_parameter("ore_number", ore_number);
        }
        information = random_generation(ore_number);    //随机生成矿石信息数组变量
        totalvalue = 0;
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MineNode>("gold_mine");
    rclcpp::spin(node);
    rclcpp::shutdown();
}