/*
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)

         PS: 二者需要设置相同的话题


    消息发布方:
        循环发布信息:HelloWorld 后缀数字编号

    实现流程:
        1.包含头文件 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 发布者 对象
        5.组织被发布的数据，并编写逻辑发布数据

*/
// 1.包含头文件 
#include <serial/serial.h>
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>
#include <iostream>
class single_laser
{
    serial::Serial ser;
    serial::Timeout to ;
    double dis;
public:
    single_laser();
    double get_dis();
    void set_dis(double distance);
    double laser_read();
};
void single_laser::set_dis(double distance)
{
    this->dis = distance;
}
double single_laser::get_dis()
{
    return this->dis;
}
single_laser::single_laser(void)
{
    dis = 0;
    to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    ser.setPort("/dev/single_laser");
    //设置串口通信的波特率
    ser.setBaudrate(9600);
    //串口设置timeout
    ser.setTimeout(to);
    try
    {
        //打开串口
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }
}


double hex2dig(unsigned char str_list[])
{
    double digital = 0.0;
    int flag = 1;
    for(int i =0 ;i< 7;i++)
    {
        char str = str_list[i];
        // ROS_INFO("str: %d",int(str));
        if(str == 'e')
            flag = 0;
        if(str>=0 && str <=9)
        {
            if(flag)
                // digital += pow(10,2-i) * std::atoi(str.c_str());
                digital += double(str) * pow(10,2-i) ;
            else
                // digital += pow(10,3-i) * std::atoi(str.c_str());
                digital += double(str) * pow(10,3-i) ;
        }
    }
    digital *= 10;
    if (digital > 1.0)
        return -1;
    // ROS_INFO("Dis: %lf",digital);
    return digital;
}
double single_laser::laser_read()
{
    if(!ser.isOpen())
    {
        try
        {
            //打开串口
            ser.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
        }
    }
    // ros::Rate loop_rate(500);

    unsigned char str_list[7];
    int match_score = 0;
    if(ros::ok())
    {
        size_t n = ser.available();
        if(n)
        {
            uint8_t buffer[12];
            //获取缓冲区内的字节数
            memset(str_list,'\0',sizeof(str_list));
            //读出数据
            n = ser.read(buffer,11);
            int index = 0;
            for(int i=3;i<n;i++)
            {
                uint8_t result = buffer[i]& 0xff;
                // ROS_INFO("%x",result);
                result = buffer[i]& 0xff;
                str_list[index++] = result & 0x0f;
                // ROS_INFO("str_list: %x",str_list[index++]);
                    
            }
            set_dis(hex2dig(str_list));
            // break;
        }
        // loop_rate.sleep();
    }
    // ser.close();
    return 0;

}


int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"talker");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);

    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    std_msgs::String msg;
    // msg.data = "你好啊！！！";
    std::string msg_front = ""; //消息前缀
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(10);
    single_laser laser;
    //节点不死
    while (ros::ok())
    {
        laser.laser_read();
        //使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << laser.get_dis();
        msg.data = ss.str();
        //发布消息
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s",msg.data.c_str());

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        //暂无应用
        ros::spinOnce();
    }


    return 0;
}
