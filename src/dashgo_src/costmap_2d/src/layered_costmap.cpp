/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(), global_frame_(global_frame), rolling_window_(rolling_window), initialized_(false), size_locked_(false)
{
  /*track_unkown_space参数先给其内部的总地图Costmap2D设置地图
   * 数据缺省值NO_INFORMATION或者FREE_SPACE
  track_unknown_space is false */
  if (track_unknown)
    costmap_.setDefaultValue(255);
  else
    costmap_.setDefaultValue(0);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();
  }
}
/*如果地图时rolling的，则先更新原点信息。然后，根据各层的更新情况确定地图更新范围的边界。
 * 然后，将更新范围内的地图信息重置为缺省值。接着，调用各层的updateCosts函数用各层的信息
 * 去更新更新范围内的地图信息
 * 参数：robot_x，robot_y，robot_yaw是当前机器人的坐标位置
*/
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  //如果我们使用的是滚动缓冲costmap …我们需要使用机器人的位置来更新原点
  //计算出局部costmap 地图左下角坐标
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;
  /*在这里会调用各层的updateBounds 更新各层的costmap地图边界
  *最小值和最大值，以像素坐标表示，最后会取所有costmap层中(静态层，动态层)最小//和最大的范围，
  *作为master 综合层
  */
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }
  /*这里出来的minx_，miny_,maxx_, maxy_是
  * 综合了静态，动态，膨胀层的范围后master costmap范围，它是
  * 以世界坐标系(map)为参考的坐标，double类型
  */
   //更新master costmap 综合层的边界
  int x0, xn, y0, yn;
  /*
  *将世界坐标系(map)下转成图像坐标系(地图左下角)，以像素点表示，
  * 是int类型
  */
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);
  //ROS_WARN("222222222222222");
  //这里什么意思
  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);

  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0)
    return;

   //更新master costmap 的范围
  costmap_.resetMap(x0, y0, xn, yn);
  //ROS_WARN("3333333333333333");
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
   //调用各层的updateCosts函数用各层的信息去更新更新范围内的地图信息
   //x0, y0, xn, yn是该层costmap地图左下角和右上角坐标，以像素表示
  //把master costmap 及相应范围传给各层costmap(如static costmap)
    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
    //static cosmap 更新后x0, y0, xn, yn 会变成什么样值
    //obstacle_layer cosmap 更新后x0, y0, xn, yn 又会变成什么样值
  }
  //ROS_WARN("4444444444444444444");
  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;
  //这里才表明master costmap 初始化更新完成
  initialized_ = true;
}


bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  /*计算内切圆和外切圆半径*/
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
