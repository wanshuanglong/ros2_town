# OOP方法编写一个节点

import rclpy # 1.1 导入库文件
from rclpy.node import Node
from std_msgs.msg import String, UInt32# 2.1 导入消息类型
#std_msgs是ROS2自带的接口类型，其中规定了我们常用的大多数消息类型，

#从村庄接口服务类中导入借钱服务
from village_interfaces.srv import BorrowMoney

class WritherNode(Node): # 继承Node类的功能
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是作家%s" %name) 
        # 2.2 声明并创建发布者
        self.pub_novel = self.create_publisher(String, "sexy_girl", 10)
        # create_publisher方法来创建的发布者，
        # 该方法一共有三个参数，第一个是方法类型，第二个是话题名称，第三个是消息队列长度，

        #2.3 编写发布逻辑发布数据
        self.count = 0
        self.write_timer_period = 5
        self.timer = self.create_timer(self.write_timer_period, self.timer_callback)
        #这个定时器的作用就是根据传入的timer_period时间周期，每隔一个timer_period秒，调用一次self.timer_callback函数。
        
        self.account = 80
        self.sub_money = self.create_subscription(UInt32, "sexy_girl_money", self.recv_money_callback, 10)

        # 4.3 声明并创建服务端
        # 新建借钱服务
        self.borrow_server = self.create_service(BorrowMoney, "borrow_money", self.borrow_money_callback)

        # 5.1 声明参数,参数名字，默认值
        self.declare_parameter("write_timer_period",5)

    def timer_callback(self):
        msg = String()
        msg.data = "第%d回: 潋滟湖 %d 次偶遇胡艳娘" % (self.count, self.count)
        self.pub_novel.publish(msg) # 让发布者发布消息
        self.get_logger().info("发布了一个章节的小说，内容是：%s" % msg.data)
        self.count += 1
        # 回调之后更新回调周期
        timer_period = self.get_parameter('write_timer_period').get_parameter_value().integer_value
        # 更新回调周期
        self.timer.timer_period_ns = timer_period * (1000*1000*1000) #纳秒转秒

    def recv_money_callback(self,money):
        """
        3.4. 编写订阅回调处理逻辑
        """
        self.account += money.data
        self.get_logger().info('李四：我已经收到了%d元的稿费, 当前帐户共%d元' % (money.data, self.account))
    
    """
    这块到底哪里出问题了啊？？？？？？？？？？？？
    我根本没找到问题出在哪里！！！！！！！！！！
    def recv_money_callback(self,money):
        self.account += money.data
        self.get_logger().info("收到了%d的稿费") % money.data #这地方出问题了，
    """ 
    #4.4 编写回调函数逻辑处理请求
    def borrow_money_callback(self,request, response):
        """
        借钱回调函数
        参数：request 客户端请求
            response 服务端响应
        返回值：response
        """
        self.get_logger().info("收到来自: %s 的借钱请求，目前账户内还有%d元" % (request.name, self.account))
        #根据李四借钱规则，借出去的钱不能多于自己所有钱的十分之一，不然就不借
        if request.money <= int(self.account*0.1):
            response.success = True
            response.money = request.money
            self.account = self.account - request.money
            self.get_logger().info("借钱成功，借出%d 元 ,目前账户余额%d 元" % (response.money,self.account))
        else:
            response.success = False
            response.money = 0
            self.get_logger().info("对不起兄弟，手头紧,不能借给你")
        return response


def main (args=None): #入口函数
    """
    1.编写ROS2节点的一般步骤
    1.1 导入库文件
    1.2 初始化客户端库
    1.3 新建节点
    1.4 spin循环节点
    1.5 关闭客户端库

    """
    """
    2.编写一个话题发布者一流程：
    2.1 导入消息类型
    2.2 声明并创建发布者
    2.3 编写发布逻辑发布数据
    
    """

    """
    
    3. 创建话题订阅者的一般流程：
    3.1 导入订阅的话题接口类型
    3.2 创建订阅回调函数
    3.3 声明并创建订阅者
    3.4 编写订阅回调处理逻辑
    
    """
    """
    4. 创建ROS2服务端基本步骤
    4.1 导入服务接口
    4.2 创建服务端回调函数
    4.3 声明并创建服务端
    4.4 编写回调函数逻辑处理请求
    
    """
    #1.2 初始化客户端库
    rclpy.init(args=args) 
    #1.3 新建节点
    li4_node = WritherNode("li4") 
    #li4_node.get_logger().info("大家好,我是作家li4.")
    #1.4 spin循环节点
    rclpy.spin(li4_node) 
    #1.5 关闭客户端
    rclpy.shutdown() 