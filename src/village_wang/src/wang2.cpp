#include "rclcpp/rclcpp.hpp"
// 1.0 导入订阅者消息类型
#include "std_msgs/msg/string.hpp"
// 2.1 导入发布者消息类型
#include "std_msgs/msg/u_int32.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

/*
   1.1 创建一个类节点，名字叫做SingleDogNode,继承自Node.
*/
class SingleDogNode : public rclcpp::Node
{

public:
    // 1.2 构造函数,有一个参数为节点名称
    SingleDogNode(std::string name) : Node(name)
    {
        // 打印一句自我介绍
        RCLCPP_INFO(this->get_logger(), "大家好，我是单身狗%s.", name.c_str());
         // 1.4 创建一个订阅者来订阅李四写的小说，通过名字sexy_girl
        sub_novel = this->create_subscription<std_msgs::msg::String>("sexy_girl", 10, 
                                            std::bind(&SingleDogNode::topic_callback, this, _1));
        // 2.4 创建发布者
        pub_money = this->create_publisher<std_msgs::msg::UInt32>("sexy_girl_money", 10);
    }

private:
    // 1.3 声明一个订阅者（成员变量）,用于订阅小说
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_novel;

    // 2.2 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_money;

    // 1.5 收到话题数据的回调函数
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        /* 2.3 编写发布逻辑来发布数据*/
        // 2.5 新建一张人民币
        std_msgs::msg::UInt32 money;
        money.data = 10;
        // 2.6 发送人民币给李四
        pub_money->publish(money);
        
        RCLCPP_INFO(this->get_logger(), "朕已阅：'%s'", msg->data.c_str());
    };

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个Wang2的节点*/
    auto node = std::make_shared<SingleDogNode>("wang2");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
