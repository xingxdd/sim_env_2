// #include "ros/ros.h"
// #include "std_msgs/String.h"

// void doMsg(const std_msgs::String::ConstPtr& msg_p){
//     ROS_INFO("我听见:%s",msg_p->data.c_str());
//     // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
// }
// int main(int argc, char  *argv[])
// {
//     setlocale(LC_ALL,"");
//     //2.初始化 ROS 节点:命名(唯一)
//     ros::init(argc,argv,"listener");
//     //3.实例化 ROS 句柄
//     ros::NodeHandle nh;

//     //4.实例化 订阅者 对象
//     ros::Subscriber sub = nh.subscribe<std_msgs::String>("fang",10,doMsg);
//     //5.处理订阅的消息(回调函数)

//     //     6.设置循环调用回调函数
//     ros::spin();//循环读取接收的数据，并调用回调函数处理

//     return 0;
// }
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>  // 使用 Int32MultiArray 消息类型

// 回调函数，处理接收到的坐标数据
void coordinatesCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        int x = msg->data[0];
        int y = msg->data[1];
        ROS_INFO("Received coordinates: (%d, %d)", x, y);
    } else {
        ROS_ERROR("Invalid message format");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    // 创建一个订阅者，订阅 "coordinates" 话题
    ros::Subscriber sub = nh.subscribe<std_msgs::Int32MultiArray>("coordinates", 10, coordinatesCallback);

    ros::spin();  // 进入 ROS 消息循环

    return 0;
}