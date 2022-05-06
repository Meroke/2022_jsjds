#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
//#include <QPushButton>
#include <QComboBox>
#include <QString>

#include <geometry_msgs/Twist.h>
//#include <QDebug>
#include <boost/thread.hpp>

#include "loop_move.h"

namespace loop_move
{
// 构造函数，初始化变量
LoopMove::LoopMove( QWidget* parent )
  : rviz::Panel( parent )
{
 // qDebug()<<"in LoopMove ";
  pause_count=0;
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
  
  //loop 循环按钮
  QHBoxLayout* button_loop_layout = new QHBoxLayout;
  QPushButton* loop_button =new QPushButton("loop",this);
  button_loop_layout->addWidget(loop_button);

  pause_button_ =new QPushButton("pause",this);
  button_loop_layout->addWidget(pause_button_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(button_1_layout);
  layout->addLayout(pose_1_layout);
  
  layout->addLayout(button_2_layout);
  layout->addLayout(pose_2_layout);
  
  layout->addLayout(button_loop_layout);

  setLayout(layout);
  
  goal_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1 );
  
  loop_navigation_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/loop_navigation/goal", 1 );
  pause_navigation_pub_ = nh_.advertise<std_msgs::Bool>("/pause_navigation", 1 );

  get_robot_pose_sub=nh_.subscribe<geometry_msgs::Pose>("/robot_pose", 1, \
  boost::bind(&LoopMove::GetRobotPoseCallback,this,_1));
  
  markerArrayPub = nh_.advertise<visualization_msgs::MarkerArray>("MarkerArray", 10);

  // 设置信号与槽的连接
  connect(record_1_button,SIGNAL(clicked()),this,SLOT(recordRobotPose1()));
  connect(publish_1_button,SIGNAL(clicked()),this,SLOT(publishGoal1()));

  connect(record_2_button,SIGNAL(clicked()),this,SLOT(recordRobotPose2()));
  connect(publish_2_button,SIGNAL(clicked()),this,SLOT(publishGoal2()));

  connect(loop_button,SIGNAL(clicked()),this,SLOT(loopNavigation()));
  connect(pause_button_,SIGNAL(clicked()),this,SLOT(pauseNavigation()));

  ros::spinOnce();
}

void LoopMove::loopNavigation()
{
//  qDebug()<<"in LoopNavigation ";
  //geometry_msgs::PoseArray poses;
  poses.poses.clear();
  poses.header.frame_id="map";
  poses.header.stamp=ros::Time::now();

  geometry_msgs::Pose tmp_pose1,tmp_pose2;
  tmp_pose1.position=pose1.pose.position;
  tmp_pose1.orientation=pose1.pose.orientation;
  tmp_pose2.position=pose2.pose.position;
  tmp_pose2.orientation=pose2.pose.orientation;

  poses.poses.push_back(tmp_pose1);
  poses.poses.push_back(tmp_pose2);
  loop_navigation_pub_.publish(poses);
}

void LoopMove::pauseNavigation()
{  
  std_msgs::Bool msg;
  if(pause_count%2 ==0)
  {
    msg.data=0;
    pause_button_->setText("continue");
  }
  else
  {
    msg.data=1;
    pause_button_->setText("pause");
  }
  pause_navigation_pub_.publish(msg);
  pause_count=pause_count+1;
}

void LoopMove::GetRobotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
 // qDebug()<<"****in GetRobotPoseCallback ";
  current_pose_.pose.position.x= msg->position.x;
  current_pose_.pose.position.y= msg->position.y;
  current_pose_.pose.position.z= msg->position.z;

  current_pose_.pose.orientation.x=msg->orientation.x;
  current_pose_.pose.orientation.y=msg->orientation.y;
  current_pose_.pose.orientation.z=msg->orientation.z;
  current_pose_.pose.orientation.w=msg->orientation.w;

 // ROS_WARN("robot pose= (%f,%f,%f)",current_pose_.position.x, \
                                    current_pose_.position.y, \
                                    current_pose_.position.z);
  //qDebug()<<"robot pose= ("<<current_pose_.pose.position.x<<","<<current_pose_.pose.position.y \
         <<","<<current_pose_.pose.position.z<<")";
}


void LoopMove::publishGoal1()
{
    pose1.header.frame_id = "map";
    pose1.header.stamp=ros::Time::now();
    
    goal_publisher_.publish(pose1);
  
}

void LoopMove::recordRobotPose1()
{
  pose1.pose.position.x=current_pose_.pose.position.x;
  pose1.pose.position.y=current_pose_.pose.position.y;
  pose1.pose.position.z=current_pose_.pose.position.z;

  pose1.pose.orientation.x=current_pose_.pose.orientation.x;
  pose1.pose.orientation.y=current_pose_.pose.orientation.y;
  pose1.pose.orientation.z=current_pose_.pose.orientation.z;
  pose1.pose.orientation.w=current_pose_.pose.orientation.w;
 // qDebug()<<"in button_1 callback recordRobotPose1";
  QString str;
  str.sprintf("(%f,%f,%f)",pose1.pose.position.x,\
                           pose1.pose.position.y, \
                           pose1.pose.position.z);
  pose_1_editor_->setText(str);
  pose_1_editor_->show();

  visualization_msgs::Marker marker;
  marker.header.frame_id="/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id =0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.b = 0;
  marker.color.g = 0;
  marker.color.r = 1.0;
  marker.color.a = 1;
  marker.text="0";

  marker.pose=pose1.pose;
  //markerPub.publish(marker);
  markerArray.markers.push_back(marker);
  markerArrayPub.publish(markerArray);
}

void LoopMove::publishGoal2()
{

    pose2.header.frame_id = "map";
    pose2.header.stamp=ros::Time::now();
    goal_publisher_.publish(pose2);

}

void LoopMove::recordRobotPose2()
{
  pose2.pose.position.x=current_pose_.pose.position.x;
  pose2.pose.position.y=current_pose_.pose.position.y;
  pose2.pose.position.z=current_pose_.pose.position.z;

  pose2.pose.orientation.x=current_pose_.pose.orientation.x;
  pose2.pose.orientation.y=current_pose_.pose.orientation.y;
  pose2.pose.orientation.z=current_pose_.pose.orientation.z;
  pose2.pose.orientation.w=current_pose_.pose.orientation.w;
//  qDebug()<<"in button_1 callback recordRobotPose1";
  QString str;
  str.sprintf("(%f,%f,%f)",pose2.pose.position.x,\
                           pose2.pose.position.y, \
                           pose2.pose.position.z);
  pose_2_editor_->setText(str);
  pose_2_editor_->show();

  visualization_msgs::Marker marker;
  marker.header.frame_id="/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id =1;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.b = 0;
  marker.color.g = 0;
  marker.color.r = 1.0;
  marker.color.a = 1;
  marker.text="1";

  marker.pose=pose2.pose;
  //markerPub.publish(marker);
  markerArray.markers.push_back(marker);
  markerArrayPub.publish(markerArray);
}

// 重载父类的功能, 这两个函数应该是ctrl+ s 保存和打开rviz 加载用到
void LoopMove::save( rviz::Config config ) const
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
  
  
}

// 重载父类的功能，加载配置数据
void LoopMove::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString str_pose1,str_pose2;
  //geometry_msgs::PoseStamped pose1,pose2;
  float tx,ty,tz,tox,toy,toz,tow;
  if( config.mapGetString( "pose1", &str_pose1 ))
  {
    //output_topic_editor_->setText( topic );
    //updateTopic();

    sscanf(str_pose1.toLatin1().data(),"%f,%f,%f,%f,%f,%f,%f",&tx,&ty,&tz,&tox,&toy,&toz,&tow);
 //   qDebug()<<"pose= ("<<tx<<","<<ty<<","<<tz<<")";
 //   qDebug()<<"orientation= ("<<tox<<","<<toy<<","<<toz<<")";

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
    //output_topic_editor_->setText( topic );
    //updateTopic();

    sscanf(str_pose2.toLatin1().data(),"%f,%f,%f,%f,%f,%f,%f",&tx,&ty,&tz,&tox,&toy,&toz,&tow);
 //   qDebug()<<"pose= ("<<tx<<","<<ty<<","<<tz<<")";
  //  qDebug()<<"orientation= ("<<tox<<","<<toy<<","<<toz<<")";

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

}

}// end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(loop_move::LoopMove,rviz::Panel )



