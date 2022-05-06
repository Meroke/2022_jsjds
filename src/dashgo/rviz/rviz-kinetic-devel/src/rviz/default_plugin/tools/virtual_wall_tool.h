#ifndef VIRTUAL_WALL_TOOL_H_
#define VIRTUAL_WALL_TOOL_H_
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

namespace rviz
{
    class Line;
    class VirtualWallTool: public Tool
    {
      public:
         VirtualWallTool();
         virtual ~VirtualWallTool();
         virtual void onInitialize();
         virtual void activate();
         virtual void deactivate();

         virtual int processMouseEvent(ViewportMouseEvent& event);

         void BuildWallPointCloud(Ogre::Vector3 &start_pos,Ogre::Vector3&end_pos);
         void deleteWallCallback(const sensor_msgs::PointCloud &deletePcloud);
         void yawSubCallback(std_msgs::Float32 msg);
         void rotation_yaw(geometry_msgs::Point32 &origin_pos,Ogre::Vector3&rotation_pos);
         bool isDelete(Ogre::Vector3&rotation_pos3,Ogre::Vector3&min_rotation_pos,Ogre::Vector3&max_rotation_pos);
    private:
         QCursor std_cursor_;
         QCursor hit_cursor_;
         enum {
           START,
           END
         } state_;
         Line* line_;
         Ogre::Vector3 start_;
         Ogre::Vector3 end_;

         ros::NodeHandle nh_,delete_nh,yaw_nh;
         ros::Publisher pub_;
         ros::Subscriber deleteVirtualWall_sub;
         sensor_msgs::PointCloud virtualWall_pointCloud, deleteVirtualWall_pcloud;
         ros::Subscriber yaw_sub_;
         float cameral_yaw,rotation_anglue;
         Ogre::Vector3 rotation_pos1,rotation_pos2,rotation_pos3,min_rotation_pos,max_rotation_pos;
         bool delete_flag;
         float line_len;

    };

}


#endif
