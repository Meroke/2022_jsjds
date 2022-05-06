#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <loop_move/LoopMoveMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <sstream>
#include <boost/thread.hpp>
#include <geometry_msgs/Quaternion.h>
//#include <QDebug>
#include <unistd.h>
#include <std_msgs/Bool.h>
using namespace std;

bool nav_state=true;
//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

void loopPoseCallback(const geometry_msgs::PoseArray& msg)
{
 // qDebug()<<"in loopPoseCallback ";
  ROS_WARN("in loopPoseCallback");

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ac.waitForServer();
  move_base_msgs::MoveBaseGoal goal1;
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();
  geometry_msgs::Quaternion quaternion;
  bool finished_before_timeout;
//  actionlib::SimpleClientGoalState state;
  int i=0;
  while(ros::ok())
  {
    for(;i<msg.poses.size()&&nav_state;)
    {

      quaternion.x = msg.poses[i].orientation.x ;
      quaternion.y = msg.poses[i].orientation.y;
      quaternion.z = msg.poses[i].orientation.z ;
      quaternion.w = msg.poses[i].orientation.w ;
      goal1.target_pose.pose.position.x = msg.poses[i].position.x ;
      goal1.target_pose.pose.position.y = msg.poses[i].position.y ;
      goal1.target_pose.pose.position.z = msg.poses[i].position.z ;
      goal1.target_pose.pose.orientation = quaternion ;

      ac.sendGoal(goal1);
      finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
      if(finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_WARN("**pose[%d] Action finished: %s",i,state.toString().c_str());
      }
      else
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_WARN("pose[%d] navigation error, state =%s",i,state.toString().c_str());
      }
      //ROS_WARN("i=%d",i);
      if(nav_state)
      {
        //ROS_WARN("after i=%d",i);
        i=i+1;
      }

      sleep(3);
    }
    //如果是执行完队列中的目标点，则从头开始，如果是暂停引起跳出导航，则继续保留该点
    //等继续导航时，从该点开始
    if(nav_state)
    {
      i=0;
    }

    sleep(1);
  }

}

void pauseNavigationCallback(const std_msgs::Bool & msg)
{
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  nav_state=msg.data;
  ac.cancelAllGoals();
}

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"excute_navigation");
  ros::NodeHandle nh;
  ros::Rate loop(1);

  ros::Subscriber loop_pose_sub=nh.subscribe("/loop_navigation/goal",1,loopPoseCallback);
  ros::Subscriber pause_navigation_sub=nh.subscribe("/pause_navigation",1,pauseNavigationCallback);

  //为了让两个订阅器同时执行
  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();
  ros::waitForShutdown();
//    ros::spin();
  return 0;
}
