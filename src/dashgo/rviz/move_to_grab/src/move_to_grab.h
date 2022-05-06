
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h> //plugin基类的头文件

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <move_to_grab/MoveToGrabMsg.h>
#include <move_to_grab/LoopMoveToGrabMsg.h>

#endif

class QLineEdit;
class QComboBox;
class QLabel;

namespace move_to_grab
{

class DriveWidget;

/*
这里我们声明了rviz::Panel的新子类。每个可以通过panel /Add_New_Panel菜单添加的面板都是rviz:: panel的子类。

TeleopPanel将显示一个文本输入字段来设置输出主题

和一个二维控制区。2D控制区域由DriveWidget类实现，并在那里进行了描述。
*/
class MoveToGrab: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
// 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
  MoveToGrab( QWidget* parent = 0 );

  // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
  // 中，数据就是topic的名称
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // 公共槽.
public Q_SLOTS:

  void recordRobotPose1();
  void publishGoal1();

  void recordRobotPose2();
  void publishGoal2();

  void GetRobotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void GetNavigationStatusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void getPose1Data(move_to_grab::MoveToGrabMsg& pose1_msg);
  void getPose2Data(move_to_grab::MoveToGrabMsg& pose2_msg);
  
  void loopMoveToGrab();
  void pauseMoveToGrab();

  // 内部槽.
protected Q_SLOTS:

  // 内部变量.
protected:
  // One-line text editor for entering the outgoing ROS topic name.
  QLabel* pose_1_editor_;
  QLineEdit* back_dist_1_editor_;
  QComboBox* dobot_1_action_;
  
  QLabel* pose_2_editor_;
  QLineEdit* back_dist_2_editor_;
  QComboBox* dobot_2_action_;

  // The ROS publisher for the command velocity.
  ros::Publisher velocity_publisher_;
  ros::Publisher goal_publisher_, loop_goal_publisher_,pause_loop_goal_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  ros::Subscriber get_robot_pose_sub;
  ros::Subscriber navigation_status_sub;

  geometry_msgs::PoseStamped current_pose_, pose1, pose2;
  move_to_grab::MoveToGrabMsg pose1_msg,pose2_msg;
  move_to_grab::LoopMoveToGrabMsg loop_msg;
  // END_TUTORIAL
};

} // end namespace

#endif 
