/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_SELECTION_TOOL_H
#define RVIZ_SELECTION_TOOL_H

#include "rviz/tool.h"
#include "rviz/selection/forwards.h"
#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Point32.h>
# include <ros/node_handle.h>
# include <ros/publisher.h>
#include <OgreVector3.h>
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include <OgreViewport.h>

#include <vector>
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"
#include <std_msgs/Float32.h>
namespace Ogre
{
class Viewport;
}

namespace rviz
{

class MoveTool;

class SelectionTool : public Tool//, public OrbitViewController
{
public:
  SelectionTool();
  virtual ~SelectionTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );
  virtual int processKeyEvent( QKeyEvent* event, RenderPanel* panel );

  virtual void update(float wall_dt, float ros_dt);
  void BuildWallPointCloud(Ogre::Vector3 &start_pos,Ogre::Vector3&end_pos);
  void yawSubCallback(std_msgs::Float32 msg);

private:

  MoveTool* move_tool_;

  bool selecting_;
  int sel_start_x_;
  int sel_start_y_;
  Ogre::Vector3 start_;
  Ogre::Vector3 end_;

  M_Picked highlight_;

  bool moving_;

  ros::NodeHandle nh_,yaw_nh;
  ros::Publisher deletePub_;
  sensor_msgs::PointCloud deleteVirtualWall_pointCloud;
  Property* property_container_;
  ros::Subscriber yaw_sub_;
  float cameral_yaw;
};

}

#endif

