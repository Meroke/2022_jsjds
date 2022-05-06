#ifndef VIRTUALWALL_LAYER_H_
#define VIRTUALWALL_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <mutex>

namespace costmap_2d
{

class VirtualWallLayer : public CostmapLayer
{
public:
  VirtualWallLayer();
  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                               double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  void subCallback(const sensor_msgs::PointCloud &pcloud);
  void deleteWallCallback(const sensor_msgs::PointCloud &pcloud);
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

  unsigned char interpretValue(unsigned char value);
private:
  ros::Subscriber virtualWall_sub_,map_sub_,deleteVirtualWall_sub;
  sensor_msgs::PointCloud virtualWall_pcloud,deleteVirtualWall_pcloud,wallStatus_pcloud;
  bool pc_recived;
  std::string global_frame_;
  std::string map_frame_;

  bool trinary_costmap_;
  bool track_unknown_space_;
  unsigned char lethal_threshold_, unknown_cost_value_;
  double local_width_,local_height_;
};



}

#endif
