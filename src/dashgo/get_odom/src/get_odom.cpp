#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
 
void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
 
    double x, y, z;
    double roll, pitch, yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    tf::Quaternion quat;                                     //定义一个四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat); //取出方向存储于四元数
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
 
    ROS_INFO("Odom: %f, %f, %f, %f, %f, %f", x, y, z, roll, pitch, yaw);
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle node;
    ros::Subscriber subOdom = node.subscribe("odom", 1000, OdomCallback);
    ros::spin();
    return 0;
}