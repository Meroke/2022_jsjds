/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

StaticLayer::~StaticLayer()
{
  if (dsrv_)
    delete dsrv_;
}

void StaticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);
  //默认没有配置，使用true
  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);
   //致命cost 阈值，使用默认的为100，未知的区域，cost 值为255
  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    //ROS_INFO("****************** Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
    //这个没有使用
    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

unsigned char StaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  // 实际上就是高于一个阈值lethal_threshold那么就认为是障碍物
  // 没有信息的就认为是未探测的区域
  // 否则为自由空间，true, -1
  if (track_unknown_space_ && value == unknown_cost_value_)
  {
    //ROS_WARN("value-unknown_cost_value_：%c-%c",value,unknown_cost_value_);
    return NO_INFORMATION;
  }
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  // >=100 就是致命的障碍物，
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;
  // 对于其他的数值则进行插值运算，其实没必要，map 中只有0,-1,100三种情况
  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  //判断master costmap地图是否发生变化
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    //ROS_INFO("******************Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }
  //一开始size_x_为0,所以会进入并仅仅更新static costmap 层
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    //ROS_INFO("================Resizing static layer to %d X %d at %f m/pix,new_x=%d,new_y=%d", size_x, size_y, 
    //        new_map->info.resolution,new_map->info.origin.position.x,new_map->info.origin.position.y);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }
  /*
  * 此时size_x_，size_y_，resolution_，origin_x_，origin_y_
  * 都已经变成map 的地图的范围等信息了
  */

  unsigned int index = 0;

  // initialize the costmap with static data
  //根据新地图中的数据来更新costmap 的值,
  //将图片的像素值转换成代价值了，具体转换可以看interpretValue函数
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      //地图传来的值为-1(即255),0 和100
      unsigned char value = new_map->data[index];
      //这个变量是从Costmap2D 类中继承来的,是一个指针，具体指向哪里呢？
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

//这个没有用
void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayer::activate()
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

/*如果是非rolling地图，则看是否有地图数据更新（初始化后会在接收到map话题后有一次更新），
 * 如果没有则不更新边界，否则，根据静态层更新的区域的边界更新传入的边界。
*/
void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  //传过来的参数 min_x=min_y=1e30，max_x=max_y=-1e30
  if( !layered_costmap_->isRolling() ){
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }
  //初始化边界
  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy; //像素坐标变量
  /*计算静态costmap地图左下角像素的坐标
  * x_,y_ 是0,0
  * 返回wx=origin_x_, wy=origin_y_
  */
  mapToWorld(x_, y_, wx, wy);
  /*
  *min_x=min_y是1e30，必定臂wx，wy大
  * 所以此时min_x，min_y为地图左下角的像素坐标wx,wy
  */
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  /*
  * 此时min_x，min_y为地图左下角的像素坐标
  */

  /*计算静态costmap地图右上角像素的坐标
  * x_,y_ 是0,0，加多0.5 是把边界算上
  * 返回的是wx = origin_x_ + (width_ + 0.5) * resolution_;
  * wx = origin_y_ + (height_ + 0.5) * resolution_;
  */
  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  /*
  *max_x=max_y=-1e30, 因此会取右上角像素坐标wx，wy
  */
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

/*对于静态层，其内部的自身更新主要是通过订阅话题的信息进行的。由于一般是通过map_server提供静态地图信息，
 * 因此一般只会发布一次地图信息，也就是说静态一般只会更新一次，即初始化的时候。
另外，也可选择定于map_topic_update形式的话题，从而动态更新静态层（目前暂未使用）。
在updateCosts函数中，默认情况下，静态层会用自己的信息覆盖传入的总地图信息。另外，
可以选择最大值更新机制（通过use_maximum参数）*/
/*作用：用static costmap 的值更新master costmap 的值
*参数:master_grid： 是master costmap 地图
* min_i,min_j,max_i,max_j 是该master costmap 地图的范围，以像素坐标表示
*/
void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_)
      //静态costmap 会进入这里面，会获取static costmap 的cost 值，然后更新master costmap
      //static costmap 的cost 值是在incomingMap函数中处理初始化的
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    tf::StampedTransform transform;
    try
    {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf::Point p(wx, wy, 0);
        p = transform(p);
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
