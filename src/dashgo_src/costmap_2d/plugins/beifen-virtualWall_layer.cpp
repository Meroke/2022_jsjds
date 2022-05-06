#include<costmap_2d/virtualWall_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <unistd.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::VirtualWallLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

VirtualWallLayer::VirtualWallLayer() {}

void VirtualWallLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_),g_nh,map_nh,delete_nh;
  current_ = true;
  pc_recived = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &VirtualWallLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  //ROS_WARN("0000000000000000000000");
  std::string virtualWall_topic;
  nh.param("topic", virtualWall_topic, std::string("/virtualWall_cloudPoint"));

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  map_sub_ = map_nh.subscribe(map_topic, 1, &VirtualWallLayer::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  virtualWall_sub_=g_nh.subscribe(virtualWall_topic, 1, &VirtualWallLayer::subCallback, this);

  deleteVirtualWall_sub=delete_nh.subscribe("/deleteVirtualWall_cloudPoint", 1, &VirtualWallLayer::deleteWallCallback, this);
  begin = ros::Time::now();
}


void VirtualWallLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  //ROS_WARN("111111111111111111111+ enabled_= %d",enabled_);
  has_updated_data_ = true;
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
}

void VirtualWallLayer::matchSize()
{

}

void VirtualWallLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  double min_px,min_py,max_px,max_py;
  int pcloud_len=virtualWall_pcloud.points.size();
  //ROS_WARN("22222222"); 
  //ROS_WARN("pcloud_len=%d",pcloud_len);
  useExtraBounds(min_x, min_y, max_x, max_y);
      min_px=-500.0;
      min_py=-500.0;
      max_px=500.0;
      max_py=500.0;
      *min_x = std::min(*min_x, min_px);
      *min_y = std::min(*min_y, min_py);
      *max_x = std::max(*max_x, max_px);
      *max_y = std::max(*max_y, max_py);
     
      //ROS_WARN("**** min_x=%f, min_y=%f ,max_x= %f, max_y=%f ",*min_x,*min_y,*max_x,*max_y);
}

void VirtualWallLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
    //ROS_WARN("====the point num of pcloud is %d ",virtualWall_pcloud.points.size());

    if(virtualWall_pcloud.points.size() < 1)
    {
        return;
    }
    int i,j;
    unsigned int mx, mx_;  
    unsigned int my, my_;
    double wx, wy;
 // Might even be in a different frame
    tf::StampedTransform transform;
    try
    {
      //ROS_WARN("++++ %s",virtualWall_pcloud.header.frame_id.c_str());
      //ROS_WARN("global ++++ %s",global_frame_.c_str());
      tf_->lookupTransform( global_frame_,virtualWall_pcloud.header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }

    std::lock_guard<std::mutex> lck(virtual_mutex);
        
    for (i=0;i < virtualWall_pcloud.points.size();i++)
    {
        // Transform from global_frame_ to map_frame_
        tf::Point p(virtualWall_pcloud.points[i].x, virtualWall_pcloud.points[i].y, 0);
        //ROS_WARN("ori (%f, %f)",virtualWall_pcloud.points[i].x, virtualWall_pcloud.points[i].y);
        p = transform(p);
        //ROS_WARN("transform (%f, %f)",p.x(), p.y());

       if(master_grid.worldToMap(p.x(), p.y(), mx, my))
       { 
           master_grid.setCost(mx, my, LETHAL_OBSTACLE);
       }
    }
}

void VirtualWallLayer::subCallback(const sensor_msgs::PointCloud &pcloud)
{
    //ROS_WARN("555555555555555555");
    virtualWall_pcloud= pcloud;
    pc_recived = true;
}

void VirtualWallLayer::deleteWallCallback(const sensor_msgs::PointCloud &deletePcloud)
{

}

void VirtualWallLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  //判断综合层地图是否发生变化
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

  unsigned int index = 0;

  // initialize the costmap with static data
  //根据新地图中的数据来更新costmap 的值,
  //将图片的像素值转换成代价值了，具体转换可以看interpretValue函数
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      //地图传来的值为-1,0 和100
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;  //更改后地图的大小
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


} // end namespace
