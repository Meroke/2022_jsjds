#ifndef LOOP_MOVE_H
#define LOOP_MOVE_H

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

#include <loop_move/LoopMoveMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <QPushButton>
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#endif

class QLineEdit;
class QComboBox;
class QLabel;

namespace loop_move
{
class LoopMove: public rviz::Panel
{
Q_OBJECT
public:
    LoopMove( QWidget* parent = 0 );
    
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    
public Q_SLOTS:
    void recordRobotPose1();
    void publishGoal1();

    void recordRobotPose2();
    void publishGoal2();    
    
    void GetRobotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

    void loopNavigation();
    void pauseNavigation();

protected:
    QLabel* pose_1_editor_;
    QLabel* pose_2_editor_;
    QPushButton* pause_button_;

    ros::Publisher goal_publisher_,loop_go_publisher_,pause_go_publisher_, \
                   loop_navigation_pub_,pause_navigation_pub_;
    ros::Publisher markerArrayPub;
    
    ros::NodeHandle nh_;
    ros::Subscriber get_robot_pose_sub;
    geometry_msgs::PoseStamped current_pose_, pose1, pose2;

    visualization_msgs::MarkerArray markerArray;

    geometry_msgs::PoseArray poses;
    int pause_count;
};

}// end namespace


#endif 


