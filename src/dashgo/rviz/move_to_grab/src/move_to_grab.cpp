
#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QComboBox>
#include <QString>

#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <boost/thread.hpp>

#include "drive_widget.h"
#include "move_to_grab.h"

namespace move_to_grab
{
// 构造函数，初始化变量
MoveToGrab::MoveToGrab( QWidget* parent )
  : rviz::Panel( parent )
  //, linear_velocity_( 0 )
  //, angular_velocity_( 0 )
{
  qDebug()<<"in MoveToGrab ";
  //设置界面布局
  //记录位置，发布位置 按钮1
  QHBoxLayout* button_1_layout = new QHBoxLayout;
  QPushButton* record_1_button =new QPushButton("record",this);
  button_1_layout->addWidget(record_1_button);
  
  QPushButton* publish_1_button =new QPushButton("publish",this);
  button_1_layout->addWidget(publish_1_button);
  
  //显示记录的位置坐标1
  QHBoxLayout* pose_1_layout = new QHBoxLayout;
  pose_1_layout->addWidget( new QLabel( "pose_1:" ));
  pose_1_editor_ = new QLabel;
  pose_1_layout->addWidget(pose_1_editor_);

  //机器人导航到相应位置后，选择机械臂的动作
  QHBoxLayout* dobot_1_action_layout= new QHBoxLayout;
  dobot_1_action_layout->addWidget(new QLabel( "dobot_1_action:" ));
  dobot_1_action_ =  new QComboBox(this);

  dobot_1_action_->addItem("suck up");
  dobot_1_action_->addItem("release");
  dobot_1_action_->addItem(" ");
  dobot_1_action_layout->addWidget( dobot_1_action_ );
  
   //记录位置，发布位置 按钮2
  QHBoxLayout* button_2_layout = new QHBoxLayout;
  
  QPushButton* record_2_button =new QPushButton("record",this);
  button_2_layout->addWidget(record_2_button);  
  QPushButton* publish_2_button =new QPushButton("publish",this);
  button_2_layout->addWidget(publish_2_button);
  
  //显示记录的位置坐标2
  QHBoxLayout* pose_2_layout = new QHBoxLayout;
  pose_2_layout->addWidget( new QLabel( "pose_2:" ));
  pose_2_editor_ = new QLabel;
  pose_2_layout->addWidget(pose_2_editor_);

  //机器人导航到相应位置后，选择机械臂的动作
  QHBoxLayout* dobot_2_action_layout= new QHBoxLayout;
  dobot_2_action_layout->addWidget(new QLabel( "dobot_2_action:" ));

  dobot_2_action_ =  new QComboBox(this);
  dobot_2_action_->addItem("suck up");
  dobot_2_action_->addItem("release");
  dobot_2_action_->addItem(" ");
  dobot_2_action_layout->addWidget( dobot_2_action_ );
  
    //在1,2两点循环导航，抓取与停止按钮
  QHBoxLayout* button_loop_layout = new QHBoxLayout;
  QPushButton* loop_button =new QPushButton("loop",this);
  button_loop_layout->addWidget(loop_button); 
  QPushButton* pause_button =new QPushButton("pause",this);
  button_loop_layout->addWidget(pause_button);
  
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(button_1_layout);
  layout->addLayout(pose_1_layout);
//  layout->addLayout(back_dist_1_layout);
  layout->addLayout(dobot_1_action_layout);
  
  layout->addLayout(button_2_layout);
  layout->addLayout(pose_2_layout);
//  layout->addLayout(back_dist_2_layout);
  layout->addLayout(dobot_2_action_layout);
  
  layout->addLayout(button_loop_layout);
  
  setLayout(layout);
  


  //goal_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1 );
  goal_publisher_ = nh_.advertise<move_to_grab::MoveToGrabMsg>("/move_to_grab/goal", 1 );
  loop_goal_publisher_ = nh_.advertise<move_to_grab::LoopMoveToGrabMsg>("/loop_move_to_grab/goal", 1 );
  
  pause_loop_goal_publisher_ = nh_.advertise<move_to_grab::LoopMoveToGrabMsg>("/pause_loop_move_to_grab/goal", 1 );
  get_robot_pose_sub=nh_.subscribe<geometry_msgs::Pose>("/robot_pose", 1, \
  boost::bind(&MoveToGrab::GetRobotPoseCallback,this,_1));
  
  navigation_status_sub=nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, \
  boost::bind(&MoveToGrab::GetNavigationStatusCallback,this,_1));

  // 设置信号与槽的连接
  connect(record_1_button,SIGNAL(clicked()),this,SLOT(recordRobotPose1()));
  connect(publish_1_button,SIGNAL(clicked()),this,SLOT(publishGoal1()));

  connect(record_2_button,SIGNAL(clicked()),this,SLOT(recordRobotPose2()));
  connect(publish_2_button,SIGNAL(clicked()),this,SLOT(publishGoal2()));

  connect(loop_button,SIGNAL(clicked()),this,SLOT(loopMoveToGrab()));
  connect(pause_button,SIGNAL(clicked()),this,SLOT(pauseMoveToGrab()));

  ros::spinOnce();
}

void MoveToGrab::loopMoveToGrab()
{
    qDebug()<<"start loop move to grab";
    loop_msg.isLoop.data=true;   
    
    getPose1Data(pose1_msg);
    loop_msg.action1=pose1_msg;
    getPose2Data(pose2_msg);
    loop_msg.action2=pose2_msg;
    
    loop_goal_publisher_.publish(loop_msg);
}

void MoveToGrab::pauseMoveToGrab()
{
    qDebug()<<"start pause move to grab";
    loop_msg.isLoop.data=false;
    pause_loop_goal_publisher_.publish(loop_msg);
}

void MoveToGrab::GetNavigationStatusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    
}

void MoveToGrab::GetRobotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
 // qDebug()<<"****in GetRobotPoseCallback ";
  current_pose_.pose.position.x= msg->position.x;
  current_pose_.pose.position.y= msg->position.y;
  current_pose_.pose.position.z= msg->position.z;

  current_pose_.pose.orientation.x=msg->orientation.x;
  current_pose_.pose.orientation.y=msg->orientation.y;
  current_pose_.pose.orientation.z=msg->orientation.z;
  current_pose_.pose.orientation.w=msg->orientation.w;

}

void MoveToGrab::getPose1Data(move_to_grab::MoveToGrabMsg& pose1_msg)
{
    pose1.header.frame_id = "map";
    pose1.header.stamp=ros::Time::now();
    
    pose1_msg.pose=pose1;
    if(dobot_1_action_->currentText() == "suck up")
    {
        //qDebug()<<"isSuckup";
        pose1_msg.isSuckup.data=1;
    }
    else
    {
        //qDebug()<<"is release";
        pose1_msg.isSuckup.data=0;
    }
    qDebug()<<dobot_1_action_->currentText();

}

void MoveToGrab::publishGoal1()
{
    getPose1Data(pose1_msg);
    goal_publisher_.publish(pose1_msg);
  
}

void MoveToGrab::recordRobotPose1()
{
  pose1.pose.position.x=current_pose_.pose.position.x;
  pose1.pose.position.y=current_pose_.pose.position.y;
  pose1.pose.position.z=current_pose_.pose.position.z;

  pose1.pose.orientation.x=current_pose_.pose.orientation.x;
  pose1.pose.orientation.y=current_pose_.pose.orientation.y;
  pose1.pose.orientation.z=current_pose_.pose.orientation.z;
  pose1.pose.orientation.w=current_pose_.pose.orientation.w;
  qDebug()<<"in button_1 callback recordRobotPose1";
  QString str;
  str.sprintf("(%f,%f,%f)",pose1.pose.position.x,\
                           pose1.pose.position.y, \
                           pose1.pose.position.z);
  pose_1_editor_->setText(str);
  pose_1_editor_->show();
}

void MoveToGrab::getPose2Data(move_to_grab::MoveToGrabMsg& pose2_msg)
{
    pose2.header.frame_id = "map";
    pose2.header.stamp=ros::Time::now();
    
    pose2_msg.pose=pose2;
    if(dobot_2_action_->currentText() == "suck up")
    {
        //qDebug()<<"isSuckup";
        pose2_msg.isSuckup.data=1;
    }
    else
    {
        //qDebug()<<"is release";
        pose2_msg.isSuckup.data=0;
    }

}

void MoveToGrab::publishGoal2()
{
    getPose2Data(pose2_msg);
    goal_publisher_.publish(pose2_msg);


}

void MoveToGrab::recordRobotPose2()
{
  pose2.pose.position.x=current_pose_.pose.position.x;
  pose2.pose.position.y=current_pose_.pose.position.y;
  pose2.pose.position.z=current_pose_.pose.position.z;

  pose2.pose.orientation.x=current_pose_.pose.orientation.x;
  pose2.pose.orientation.y=current_pose_.pose.orientation.y;
  pose2.pose.orientation.z=current_pose_.pose.orientation.z;
  pose2.pose.orientation.w=current_pose_.pose.orientation.w;
  qDebug()<<"in button_1 callback recordRobotPose1";
  QString str;
  str.sprintf("(%f,%f,%f)",pose2.pose.position.x,\
                           pose2.pose.position.y, \
                           pose2.pose.position.z);
  pose_2_editor_->setText(str);
  pose_2_editor_->show();
}

// 重载父类的功能, 这两个函数应该是ctrl+ s 保存和打开rviz 加载用到
void MoveToGrab::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  QString str_pose1,str_pose2;
  str_pose1.sprintf("%f,%f,%f,%f,%f,%f,%f",pose1.pose.position.x,\
                           pose1.pose.position.y, \
                           pose1.pose.position.z, \
                           pose1.pose.orientation.x,\
                           pose1.pose.orientation.y, \
                           pose1.pose.orientation.z, \
                           pose1.pose.orientation.w
              );

  str_pose2.sprintf("%f,%f,%f,%f,%f,%f,%f",pose2.pose.position.x,\
                           pose2.pose.position.y, \
                           pose2.pose.position.z, \
                           pose2.pose.orientation.x,\
                           pose2.pose.orientation.y, \
                           pose2.pose.orientation.z, \
                           pose2.pose.orientation.w
              );
  config.mapSetValue( "pose1", str_pose1 );
  config.mapSetValue( "pose2", str_pose2 );
  
  config.mapSetValue("dobot_1_action",dobot_1_action_->currentText());
  config.mapSetValue("dobot_2_action",dobot_2_action_->currentText());
  
}

// 重载父类的功能，加载配置数据
void MoveToGrab::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString str_pose1,str_pose2;
  //geometry_msgs::PoseStamped pose1,pose2;
  float tx,ty,tz,tox,toy,toz,tow;
  if( config.mapGetString( "pose1", &str_pose1 ))
  {

    sscanf(str_pose1.toLatin1().data(),"%f,%f,%f,%f,%f,%f,%f",&tx,&ty,&tz,&tox,&toy,&toz,&tow);
    qDebug()<<"pose= ("<<tx<<","<<ty<<","<<tz<<")";
    qDebug()<<"orientation= ("<<tox<<","<<toy<<","<<toz<<")";

    str_pose1.sprintf("(%f,%f,%f)",tx,\
                             ty, \
                             tz
                      );
    pose_1_editor_->setText(str_pose1);
    pose1.pose.position.x=tx;
    pose1.pose.position.y=ty;
    pose1.pose.position.z=tz;

    pose1.pose.orientation.x=tox;
    pose1.pose.orientation.y=toy;
    pose1.pose.orientation.z=toz;
    pose1.pose.orientation.w=tow;
  }

  if( config.mapGetString( "pose2", &str_pose2 ))
  {

    sscanf(str_pose2.toLatin1().data(),"%f,%f,%f,%f,%f,%f,%f",&tx,&ty,&tz,&tox,&toy,&toz,&tow);
    qDebug()<<"pose= ("<<tx<<","<<ty<<","<<tz<<")";
    qDebug()<<"orientation= ("<<tox<<","<<toy<<","<<toz<<")";

    str_pose2.sprintf("(%f,%f,%f)",tx,\
                             ty, \
                             tz
                      );
    pose_2_editor_->setText(str_pose2);
    pose2.pose.position.x=tx;
    pose2.pose.position.y=ty;
    pose2.pose.position.z=tz;

    pose2.pose.orientation.x=tox;
    pose2.pose.orientation.y=toy;
    pose2.pose.orientation.z=toz;
    pose2.pose.orientation.w=tow;
  }

  QString dobot_1_action_str,dobot_2_action_str;
  if( config.mapGetString( "dobot_1_action", &dobot_1_action_str ))
  {
    dobot_1_action_->setCurrentText(dobot_1_action_str);
  }
  
  if( config.mapGetString( "dobot_2_action", &dobot_2_action_str ))
  {
    dobot_2_action_->setCurrentText(dobot_2_action_str);
  }
  
}

} // end namespace 


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(move_to_grab::MoveToGrab,rviz::Panel )
// END_TUTORIAL
