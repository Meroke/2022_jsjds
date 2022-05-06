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

#include <QKeyEvent>

#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreMovableObject.h>
#include <OgreRectangle2D.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <ros/time.h>

#include "move_tool.h"

#include "rviz/ogre_helpers/camera_base.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"

#include "selection_tool.h"

#include <QDebug>
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"
#include "rviz/properties/float_property.h"

namespace rviz
{

SelectionTool::SelectionTool()
  : Tool()
  , move_tool_( new MoveTool() )
  , selecting_( false )
  , sel_start_x_( 0 )
  , sel_start_y_( 0 )
  , moving_( false )
  , cameral_yaw(0)
{
  shortcut_key_ = 's';
  access_all_keys_ = true;
  deletePub_ = nh_.advertise<sensor_msgs::PointCloud>("/deleteVirtualWall_cloudPoint", 1 );
  deleteVirtualWall_pointCloud.header.stamp= ros::Time::now();
  //property_container_=new Property();
  yaw_sub_=yaw_nh.subscribe("/cameral_yaw", 1, &SelectionTool::yawSubCallback, this);
}

SelectionTool::~SelectionTool()
{
  delete move_tool_;
}

void SelectionTool::onInitialize()
{
  move_tool_->initialize( context_ );
  //OrbitViewController::onInitialize();
}

void SelectionTool::activate()
{
  setStatus( "Click and drag to select objects on the screen." );
  context_->getSelectionManager()->setTextureSize(512);
  selecting_ = false;
  moving_ = false;
//  context_->getSelectionManager()->enableInteraction(true);
}

void SelectionTool::deactivate()
{
  context_->getSelectionManager()->removeHighlight();
}

void SelectionTool::update(float wall_dt, float ros_dt)
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  if (!selecting_)
  {
    sel_manager->removeHighlight();
  }
}

int SelectionTool::processMouseEvent( ViewportMouseEvent& event )
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  int flags = 0;

  if( event.alt() )
  {
    moving_ = true;
    selecting_ = false;
  }
  else
  {
    moving_ = false;

    if( event.leftDown() )
    {
      selecting_ = true;

      sel_start_x_ = event.x;
      sel_start_y_ = event.y;

      bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, start_ );
      qDebug()<<"***in selection-leftDown, start_x= "<<start_.x<<" start_y="<<start_.y;
      qDebug()<<"***in select start_event-x,y=("<< event.x<<","<<event.y<<")";
    }
  }

  if( selecting_ )
  {
    sel_manager->highlight( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y );

    if( event.leftUp() )
    {
      SelectionManager::SelectType type = SelectionManager::Replace;

      bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, end_ );
      qDebug()<<"***in selection-leftup, end_x= "<<end_.x<<" end_y="<<end_.y;
      qDebug()<<"***in select end_event-x,y=("<< event.x<<","<<event.y<<")";
      BuildWallPointCloud(start_,end_);

      M_Picked selection;

      if( event.shift() )
      {
        type = SelectionManager::Add;
      }
      else if( event.control() )
      {
        type = SelectionManager::Remove;
      }

      sel_manager->select( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y, type );

      selecting_ = false;
    }

    flags |= Render;
  }
  else if( moving_ )
  {
    sel_manager->removeHighlight();

    flags = move_tool_->processMouseEvent( event );

    if( event.type == QEvent::MouseButtonRelease )
    {
      moving_ = false;
    }
  }
  else
  {
    sel_manager->highlight( event.viewport, event.x, event.y, event.x, event.y );
  }

  return flags;
}

int SelectionTool::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  if( event->key() == Qt::Key_F )
  {
    sel_manager->focusOnSelection();
  }

  return Render;
}

void SelectionTool::BuildWallPointCloud(Ogre::Vector3 &start_pos,Ogre::Vector3&end_pos)
{
     //qDebug()<<"11111111111111111111";
     /*
     geometry_msgs::Point32 p;
     //int flag_x=start_pos.x-end_pos.x>0? 1 : -1;
     //int flag_y=start_pos.y-end_pos.y>0? 1 : -1;
     int count=0;
     float i=0,j=0;
     float start_x=start_pos.x<end_pos.x?start_pos.x:end_pos.x;
     float end_x=end_pos.x >start_pos.x?end_pos.x:start_pos.x;

     float start_y=start_pos.y<end_pos.y?start_pos.y:end_pos.y;
     float end_y=end_pos.y >start_pos.y?end_pos.y:start_pos.y;
     for(i=start_x;i<=end_x;i=i+0.05)
     {
         for(j=start_y;j<=end_y;j=j+0.05)
         {
            p.x=i;
            p.y=j;
            p.z=0.35;
             //p.x=0;
             //p.y=0;
            // p.z=0.35;
            deleteVirtualWall_pointCloud.points.push_back(p);
            //qDebug()<<"count= "<<count <<"p=("<<p.x<<","<<p.y<<")";
            count++;
         }
     }
     deleteVirtualWall_pointCloud.header.frame_id = context_->getFixedFrame().toStdString();
     qDebug()<<"deleteVirtualWall_pointCloud"<<deleteVirtualWall_pointCloud.points.size();
     deletePub_.publish(deleteVirtualWall_pointCloud);
     deleteVirtualWall_pointCloud.points.clear();
     */
    //发起点和终点到虚拟墙类中
    geometry_msgs::Point32 p_start,p_end;
    p_start.x=start_pos.x;
    p_start.y=start_pos.y;
    p_start.z=0.35;
    p_end.x=end_pos.x;
    p_end.y=end_pos.y;
    p_end.z=0.35;
    deleteVirtualWall_pointCloud.header.frame_id = context_->getFixedFrame().toStdString();
    deleteVirtualWall_pointCloud.points.push_back(p_start);
    deleteVirtualWall_pointCloud.points.push_back(p_end);
    deletePub_.publish(deleteVirtualWall_pointCloud);
    deleteVirtualWall_pointCloud.points.clear();

}

void SelectionTool::yawSubCallback(std_msgs::Float32 msg)
{
    cameral_yaw =msg.data;
    //qDebug()<<"cameral_yaw= "<<cameral_yaw;
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::SelectionTool, rviz::Tool )
