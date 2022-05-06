#include "virtual_path_tool.h"

#include "rviz/ogre_helpers/line.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/load_resource.h"
#include <QDebug>

#include <OgreSceneNode.h>

#include <sstream>
#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Point32.h>
#include <math.h>
#include <iostream>

namespace rviz {
   VirtualPathTool::VirtualPathTool():state_(NONE),point_num(0)
   {
       qDebug()<<"init VirtualPathTool sucess";
       pub_ = nh_.advertise<sensor_msgs::PointCloud>("/virtualPath_cloudPoint", 1 );
       virtualPath_pointCloud.header.stamp= ros::Time::now();
       
       deleteVirtualPath_sub=delete_nh.subscribe("/deleteVirtualPath_cloudPoint", 1, &VirtualPathTool::deletePathCallback, this);

       yaw_sub_=yaw_nh.subscribe("/cameral_yaw", 1, &VirtualPathTool::yawSubCallback, this);
   }

   VirtualPathTool::~VirtualPathTool()
   {
       /*
       sensor_msgs::PointCloud* Null_PointCloud= new PointCloud();
       pub_.publish(Null_PointCloud);
       delete line_;
       delete Null_PointCloud;
       */
       qDebug()<<"enter in ~VirtualPathTool";
   }

   void VirtualPathTool::onInitialize()
   {
       line_ = new Line(context_->getSceneManager());
     
       std_cursor_ = getDefaultCursor();
       hit_cursor_ = makeIconCursor( "package://rviz/icons/crosshair.svg" );
   }

   void VirtualPathTool::activate()
   {
       state_ = NONE;
   }

   void VirtualPathTool::deactivate()
   {
   }
   int VirtualPathTool::processMouseEvent( ViewportMouseEvent& event )
   {
       Ogre::Vector3 pos;
       bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
       setCursor( success ? hit_cursor_ : std_cursor_ );
       //qDebug()<<"event.viewport="<<event.viewport;
       //qDebug()<<" event-x,y=("<<event.x<<","<<event.y<<")";
       //qDebug()<<" event.y" <<event.y;
       //qDebug()<<" pos=("<<pos.x<<","<<pos.y<<")";
       if (event.leftDown())
       {
           virtualPath_pointCloud.header.frame_id = context_->getFixedFrame().toStdString();
           state_=LEFT_DOWN;
           temp_pos=start_=pos;
           points.push_back(start_);

           pointPath.x=temp_pos.x;
           pointPath.y=temp_pos.y;
           pointPath.z=0.35;        
           virtualPath_pointCloud.points.push_back(pointPath);

           qDebug()<<"in virtual path left down,start_=("<<start_.x<<","<<start_.y<<")";
       }
       else if(event.leftUp())
       {
           state_=LEFT_UP;
       }
       else if(event.middleDown())
       {
           qDebug()<<"in middle down";
           virtualPath_pointCloud.points.clear();
           pub_.publish(virtualPath_pointCloud);
       }

       switch(state_)
       {
            case LEFT_DOWN:
                qDebug()<<"**** in left down pos=("<<pos.x<<","<<pos.y<<")";               
                if(sqrt(pow(temp_pos.x-pos.x,2)+pow(temp_pos.y-pos.y,2))>=0.05)
                {
                    temp_pos=pos;
                    points.push_back(pos);

                    pointPath.x=pos.x;
                    pointPath.y=pos.y;
                    pointPath.z=0.35;                    
                    virtualPath_pointCloud.points.push_back(pointPath);
                    pub_.publish(virtualPath_pointCloud);               
                    qDebug()<<"===points_num="<<points.size();
                }
                break;

            case LEFT_UP:
                qDebug()<<"**** in left up";
                break;
            
       }
       /*
       switch ( state_ )
       {
         case START:
           break;
         case END:
           if ( success )
           {
               points.push_back(pos);
               for(QVector<Ogre::Vector3>::iterator ite=points.begin();ite<points.end()-1;ite++)
               {
                 qDebug()<<"****set pos=("<<(*ite).x<<","<<(*ite).y<<")";
                 line_->setPoints(*ite,*ite);
                 //line_->setPosition(*ite);
               }
             //line_->setPoints(start_,pos);
             //length_ = (start_-pos).length();
             //qDebug()<<"in VirtualWallTool start is "<<"("<<start_.x<<","<<start_.y<<")";
             //qDebug()<<"VirtualWallTool END is "<<"("<<pos.x<<","<<pos.y<<")";
           }
           break;
       }

       if( event.leftUp() && success )
       {
         switch ( state_ )
         {
           case START:
             start_ = pos;
             points.push_back(start_);
             state_ = END;
             break;
           case END:
             end_ = pos;
             state_ = START;
             //line_->setPoints(start_,end_);
             points.push_back(end_);
             
             //BuildPathPointCloud(start_,end_);
             break;
         }
        }*/
       return 0;
   }

   void VirtualPathTool::BuildPathPointCloud(Ogre::Vector3 &start_pos,Ogre::Vector3&end_pos)
   {
        geometry_msgs::Point32 p;
        //直线y=ax+b
        float a=(end_pos.y-start_pos.y)/(end_pos.x-start_pos.x);
        float b=start_pos.y-a*start_pos.x;
        //画的直线长度，并计算出需要多少个点 (x2-x1)平方+(y2-y1)的平方再开根号得到直线长度，

        int point_num=int (sqrt(pow((end_pos.x-start_pos.x),2)+pow((end_pos.y-start_pos.y),2))/0.05);

        for(int i=0;i<point_num;i++)
        {//xi=x1+(x2-x1)/point_num *i  yi=a*xi+b
            p.x=start_pos.x+((end_pos.x-start_pos.x)/point_num)*i;
            p.y=a*p.x+b;
            p.z=0.35;

            virtualPath_pointCloud.points.push_back(p);
        }
        virtualPath_pointCloud.header.frame_id = context_->getFixedFrame().toStdString();
         //qDebug()<<"*****"<<context_->getFixedFrame().toStdString();;
        //qDebug()<<"virtualWall_pointCloud"<<virtualWall_pointCloud.points.size();
        pub_.publish(virtualPath_pointCloud);
        //virtualWall_pointCloud.points.clear();
        //pub_.publish(virtualWall_pointCloud);
   }

void VirtualPathTool::deletePathCallback(const sensor_msgs::PointCloud &deletePcloud)
{
    
    qDebug()<<"****before delete num= "<<virtualPath_pointCloud.points.size();   
    deleteVirtualPath_pcloud = deletePcloud;
    delete_flag=false;
    int count=0;
    /*
    for( std::vector<geometry_msgs::Point32>::iterator it = deleteVirtualWall_pcloud.points.begin(); \
         it != deleteVirtualWall_pcloud.points.end(); it++) 
    {
	for( std::vector<geometry_msgs::Point32>::iterator its = virtualWall_pointCloud.points.begin(); \
             its != virtualWall_pointCloud.points.end(); its++)
        {
		if((fabs((*it).x -(*its).x) < 0.3 && fabs((*it).y -(*its).y) < 0.3) )
                {
			virtualWall_pointCloud.points.erase(its);
			break;
		}
    	}
    }
    for(int i=0;i<5;i++)
    {
        pub_.publish(virtualWall_pointCloud);
    }
    qDebug()<<"****after delete num= "<<virtualWall_pointCloud.points.size(); 
    */
    //获取出画的删除矩形起点和终点
    //Ogre::Vector3 rotation_pos1,rotation_pos2,rotation_pos3,min_rotation_pos,max_rotation_pos;
    //旋转cameral_yaw，得到在新坐标系的的坐标
    rotation_yaw(deleteVirtualPath_pcloud.points[0],rotation_pos1);
    rotation_yaw(deleteVirtualPath_pcloud.points[1],rotation_pos2);
    //qDebug()<<"before rotation start_x="<<deleteVirtualWall_pcloud.points[0].x<<"start_y"<<deleteVirtualWall_pcloud.points[0].y;
    //qDebug()<<"before rotation end_x= "<<deleteVirtualWall_pcloud.points[1].x<<"end_y= "<<deleteVirtualWall_pcloud.points[1].y;
    //qDebug()<<"after rotation start_x="<<rotation_pos1.x<<"start_y"<<rotation_pos1.y;
    //qDebug()<<"after rotation end_x= "<<rotation_pos2.x<<"end_y= "<<rotation_pos2.y;

    //求出离旋转后的坐标轴，矩形的另外两个点，并求出离坐标原点最近的和最远的两个点
    min_rotation_pos.x=rotation_pos1.x<rotation_pos2.x?rotation_pos1.x:rotation_pos2.x;
    min_rotation_pos.y=rotation_pos1.y<rotation_pos2.y?rotation_pos1.y:rotation_pos2.y;

    max_rotation_pos.x=rotation_pos1.x>rotation_pos2.x?rotation_pos1.x:rotation_pos2.x;
    max_rotation_pos.y=rotation_pos1.y>rotation_pos2.y?rotation_pos1.y:rotation_pos2.y;
    qDebug()<<"min_x="<<min_rotation_pos.x<<"min_y"<<min_rotation_pos.y;
    qDebug()<<"max_x= "<<max_rotation_pos.x<<"max_y= "<<max_rotation_pos.y;
    for( std::vector<geometry_msgs::Point32>::iterator its = virtualPath_pointCloud.points.begin(); \
             its != virtualPath_pointCloud.points.end(); )
    {
        //qDebug()<<"before rotation del_x="<<(*its).x<<"del_y"<<(*its).y;
        rotation_yaw(*its,rotation_pos3);

        //qDebug()<<"after rotation del_x="<<rotation_pos3.x<<"del_y"<<rotation_pos3.y;

        //if((rotation_pos3.x>=min_rotation_pos.x &&rotation_pos3.x<=max_rotation_pos.x)\
        //   &&(rotation_pos3.y>=min_rotation_pos.y &&rotation_pos3.y<=max_rotation_pos.y))
        if(isDelete(rotation_pos3,min_rotation_pos,max_rotation_pos))
        {
            its = virtualPath_pointCloud.points.erase(its);
            delete_flag=true;
            //break;
        }

        if(delete_flag)
        {
            //its=virtualWall_pointCloud.points.begin();
            //its=its;
            qDebug()<<"is delete ";
            delete_flag=false;
        }
        else
        {
            qDebug()<<"not delete num="<<count++;
            qDebug()<<" the pos is:("<<rotation_pos3.x<<","<<rotation_pos3.y<<")";
            its++;
            delete_flag=false;
        }
    }
    for(int i=0;i<5;i++)
    {
        pub_.publish(virtualPath_pointCloud);
    }
    qDebug()<<"****after delete num= "<<virtualPath_pointCloud.points.size();
    //rotation_pos1.clear();
    //rotation_pos2.clear();
    //rotation_pos3.clear();
   // min_rotation_pos.clear();
   // max_rotation_pos.clear();
}

bool VirtualPathTool::isDelete(Ogre::Vector3&rotation_pos3,Ogre::Vector3&min_rotation_pos,Ogre::Vector3&max_rotation_pos)
{
    if((rotation_pos3.x>=min_rotation_pos.x &&rotation_pos3.x<=max_rotation_pos.x)\
       &&(rotation_pos3.y>=min_rotation_pos.y &&rotation_pos3.y<=max_rotation_pos.y))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void VirtualPathTool::rotation_yaw(geometry_msgs::Point32 &origin_pos,Ogre::Vector3&rotation_pos)
{
    rotation_pos.x=origin_pos.x * cos(rotation_anglue)+origin_pos.y*sin(rotation_anglue);
    rotation_pos.y=origin_pos.y * cos(rotation_anglue)-origin_pos.x *sin(rotation_anglue);
}

void VirtualPathTool::yawSubCallback(std_msgs::Float32 msg)
{
   //cameral_yaw = msg.data;
   cameral_yaw=msg.data;
   //cameral_yaw=fmodf(cameral_yaw,float(M_PI_2));
    //qDebug()<<" in VirtualWallTool cameral_yaw= "<<cameral_yaw;
    //当坐标轴旋转0,M_PI_2，M_PI 和2*M_PI 时，画的删除矩形与坐标轴重合，可直接计算，不需要再转。
    if(fabs(cameral_yaw-0.00)<=0.01 ||fabs(cameral_yaw-float(M_PI_2))<=0.01 \
       ||fabs(cameral_yaw-float(M_PI))<=0.01 ||fabs(cameral_yaw-float(M_PI+M_PI_2))<=0.01 \
       || fabs(cameral_yaw-float(M_PI+M_PI))<=0.01)
    {
        rotation_anglue=0;
        //qDebug()<<"in 0,rotation_anglue="<<rotation_anglue;
    } //0~90
    else if (cameral_yaw-0.00>0.01 && float(M_PI_2)-cameral_yaw >0.01)
    {
        rotation_anglue=cameral_yaw;
        //qDebug()<<"in 0~90,rotation_anglue=cameral_yaw="<<rotation_anglue;
    } //90~180
    else if (cameral_yaw-float(M_PI_2)> 0.01 && float(M_PI)-cameral_yaw >0.01)
    {
        rotation_anglue=cameral_yaw-float(M_PI_2);
        //qDebug()<<"in 90~180,cameral_yaw-float(M_PI_2)="<<rotation_anglue;
    }//180~270
    else if (cameral_yaw-float(M_PI) >0.01 && float(M_PI_2+M_PI)-cameral_yaw>0.01)
    {
        rotation_anglue=cameral_yaw-float(M_PI);
        //qDebug()<<"in 180~270,cameral_yaw-float(M_PI)="<<rotation_anglue;
    }//270~360
    else if(cameral_yaw-float(M_PI_2+M_PI) >0.01 && float(M_PI+M_PI)-cameral_yaw>0.01)
    {
        rotation_anglue=cameral_yaw-float(M_PI_2+M_PI);
        //qDebug()<<"in 180~270,cameral_yaw-float(M_PI_2+M_PI)="<<rotation_anglue;
    }
    //qDebug()<<" in VirtualPahtTool rotation_anglue= "<<rotation_anglue;
}

}
// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::VirtualPathTool,rviz::Tool )
// END_TUTORIAL
