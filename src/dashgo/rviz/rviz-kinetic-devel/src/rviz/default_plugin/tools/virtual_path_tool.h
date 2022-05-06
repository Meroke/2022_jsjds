#ifndef VIRTUAL_Path_TOOL_H_
#define VIRTUAL_path_TOOL_H_
#include "rviz/tool.h"
#include <OgreVector3.h>
#include "rviz/ogre_helpers/line.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/load_resource.h"
#include <QDebug>

#include <OgreSceneNode.h>
#include <std_msgs/Float32.h>

#include <sstream>

# include <ros/node_handle.h>
# include <ros/publisher.h>
#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Point32.h>
#include <QVector>


namespace rviz
{
    class Line;
    class VirtualPathTool: public Tool
    {
      public:
         VirtualPathTool();
         virtual ~VirtualPathTool();
         virtual void onInitialize();
         virtual void activate();
         virtual void deactivate();

         virtual int processMouseEvent(ViewportMouseEvent& event);

         void BuildPathPointCloud(Ogre::Vector3 &start_pos,Ogre::Vector3&end_pos);
         void deletePathCallback(const sensor_msgs::PointCloud &deletePcloud);
         void yawSubCallback(std_msgs::Float32 msg);
         void rotation_yaw(geometry_msgs::Point32 &origin_pos,Ogre::Vector3&rotation_pos);
         bool isDelete(Ogre::Vector3&rotation_pos3,Ogre::Vector3&min_rotation_pos,Ogre::Vector3&max_rotation_pos);
    private:
         QCursor std_cursor_;
         QCursor hit_cursor_;
         enum {
           LEFT_DOWN,
           LEFT_UP,
           NONE
         } state_;
         Line* line_;
         Ogre::Vector3 start_,temp_pos;
         Ogre::Vector3 end_;

         ros::NodeHandle nh_,delete_nh,yaw_nh;
         ros::Publisher pub_;
         ros::Subscriber deleteVirtualPath_sub;
         sensor_msgs::PointCloud virtualPath_pointCloud, deleteVirtualPath_pcloud;
         geometry_msgs::Point32 pointPath;
         ros::Subscriber yaw_sub_;
         float cameral_yaw,rotation_anglue;
         Ogre::Vector3 rotation_pos1,rotation_pos2,rotation_pos3,min_rotation_pos,max_rotation_pos;
         bool delete_flag;
         int point_num;
         float line_len;
         
         QVector<Ogre::Vector3>points;
        

    };

}


#endif