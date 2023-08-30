#include "rclcpp/rclcpp.hpp"
#include "mine_interfaces/msg/ore.hpp"
#include "mine_interfaces/msg/ore_array.hpp"
#include "mine_interfaces/srv/mining.hpp"
#include<math.h>

class MinerNode: public rclcpp::Node
{
private:
    rclcpp::Subscription<mine_interfaces::msg::OreArray>::SharedPtr miner;   //声明订阅者
    geometry_msgs::msg::Point location; //采矿人当前的位置
    float totalvalue;   // 记录当前已经采到的矿石总价值
    rclcpp::Client<mine_interfaces::srv::Mining>::SharedPtr mining_client;  //声明采矿话题的客户端
    void info_callback(const mine_interfaces::msg::OreArray ore_location) //话题的回调函数，接受矿石信息，并向服务端发出采矿请求
    {
        while(!this->mining_client->wait_for_service(std::chrono::seconds(1))) //等待服务段上线
        {
            RCLCPP_INFO(this->get_logger(), "等待服务端上线");
        }
        int num = ore_location.header.stamp.sec;    //获得目前矿石数量
        if(num > 0){    //判断矿石是否采完
            int id = 0; //设置一个记录距离目前位置最近的矿石的编号变量
            for(int i = 1; i < num; i++){   //遍历目前矿石信息，找出与自身距离最近的矿石的编号
                if(pow((ore_location.ores[i].location.position.x-this->location.x), 2) + pow((ore_location.ores[i].location.position.y-this->location.y), 2) + pow((ore_location.ores[i].location.position.z-this->location.z), 2) <
                    pow((ore_location.ores[id].location.position.x-this->location.x), 2) + pow((ore_location.ores[id].location.position.y-this->location.y), 2) + pow((ore_location.ores[id].location.position.z-this->location.z), 2))
                {
                    id = i;
                }
            }
            auto request = std::make_shared<mine_interfaces::srv::Mining_Request>();    //定义向服务端发送的请求
            request->number.data = id;  //把将要采的矿的编号赋值给请求
            RCLCPP_INFO(this->get_logger(), "第一个矿石的信息如下:\n类型：%s\n位置：（%f,%f,%f）\n价值：%f", ore_location.ores[0].type.data.c_str(), ore_location.ores[0].location.position.x, ore_location.ores[0].location.position.y, ore_location.ores[0].location.position.z, ore_location.ores[0].value.data);
            RCLCPP_INFO(this->get_logger(), "发出采矿请求");
            this->mining_client->async_send_request(request, std::bind(&MinerNode::mining_callback, this, std::placeholders::_1));
        }else{
            RCLCPP_INFO(this->get_logger(), "采矿结束，采到的矿石总价值为：%f", this->totalvalue);
        }
    }
    void mining_callback(rclcpp::Client<mine_interfaces::srv::Mining>::SharedFuture response)   //接收服务端响应后的回调函数
    {
        auto result = response.get();
        this->location = result->location;
        this->totalvalue = result->total_value.data;
        RCLCPP_INFO(this->get_logger(), "成功采矿，当前矿石总价值为：%f，当前位置在（%f,%f,%f）", this->totalvalue, this->location.x, this->location.y, this->location.z);
    }
public:
    MinerNode(std::string name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "生成采矿人成功！");
        this->location.x = 0;
        this->location.y = 0;
        this->location.z = 0;
        this->miner = this->create_subscription<mine_interfaces::msg::OreArray>("location", 10, std::bind(&MinerNode::info_callback, this, std::placeholders::_1));
        this->mining_client = this->create_client<mine_interfaces::srv::Mining>("mining");
        totalvalue = 0;
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinerNode>("miner");
    rclcpp::spin(node);
    rclcpp::shutdown();
}