#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>  // 使用 Int32MultiArray 消息类型
#include <vector>
#include<talker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "coordinate_talker");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到 "coordinates" 话题，消息类型为 std_msgs::Int32MultiArray
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("coordinates", 10);

    ros::Rate loop_rate(1);  // 设置循环频率为 1Hz

    while (ros::ok()) {
        std_msgs::Int32MultiArray msg;
        msg.data.push_back(all_x);  // x 坐标
        msg.data.push_back(all_y);  // y 坐标

        pub.publish(msg);  // 发布消息

        ROS_INFO("Publishing coordinates: (%d, %d)", msg.data[0], msg.data[1]);

        loop_rate.sleep();  // 等待循环的剩余部分
    }

    return 0;
}