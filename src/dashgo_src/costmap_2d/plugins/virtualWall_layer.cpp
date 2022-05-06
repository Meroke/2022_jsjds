#include<costmap_2d/virtualWall_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <unistd.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::VirtualWallLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{
  VirtualWallLayer::VirtualWallLayer() {}

  void VirtualWallLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_),g_nh,map_nh,delete_nh;
    global_frame_ = layered_costmap_->getGlobalFrameID();

    current_ = true;
    nh.param("track_unknown_space", track_unknown_space_, true);

    int temp_lethal_threshold, temp_unknown_cost_value;
    nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
    nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
    nh.param("trinary_costmap", trinary_costmap_, true);

    lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
    unknown_cost_value_ = temp_unknown_cost_value;

    std::string map_topic;
    nh.param("map_topic", map_topic, std::string("map"));
    map_sub_ = map_nh.subscribe(map_topic, 1, &VirtualWallLayer::incomingMap, this);
   // map_received_ = false;
  //  has_updated_data_ = false;

    std::string virtualWall_topic;
    nh.param("topic", virtualWall_topic, std::string("/virtualWall_cloudPoint"));
    virtualWall_sub_=g_nh.subscribe(virtualWall_topic, 1, &VirtualWallLayer::subCallback, this);

    deleteVirtualWall_sub=delete_nh.subscribe("/deleteVirtualWall_cloudPoint", 1, &VirtualWallLayer::deleteWallCallback, this);
  }

  unsigned char VirtualWallLayer::interpretValue(unsigned char value)
  {
    // check if the static value is above the unknown or lethal thresholds
    // 实际上就是高于一个阈值lethal_threshold那么就认为是障碍物
    // 没有信息的就认为是未探测的区域
    // 否则为自由空间
    if (track_unknown_space_ && value == unknown_cost_value_)
    {
      //ROS_WARN("value-unknown_cost_value_：%c-%c",value,unknown_cost_value_);
      return NO_INFORMATION;
    }
    else if (!track_unknown_space_ && value == unknown_cost_value_)
      return FREE_SPACE;
    else if (value >= lethal_threshold_)
      return LETHAL_OBSTACLE;
    else if (trinary_costmap_)
      return FREE_SPACE;
    // 对于其他的数值则进行插值运算
    double scale = (double) value / lethal_threshold_;
    return scale * LETHAL_OBSTACLE;
  }

  void VirtualWallLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
  {
    unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

    //ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

    unsigned int index = 0;
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
    ROS_WARN("================Resizing virtualWall layer to %d X %d at %f m/pix,new_x=%lf,new_y=%lf", size_x, size_y,
            new_map->info.resolution,new_map->info.origin.position.x,new_map->info.origin.position.y);
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
        //初始化成空闲的
        //costmap_[index] = FREE_SPACE;
        ++index;
      }
    }
    map_frame_ = new_map->header.frame_id;

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

  void VirtualWallLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                             double* min_y, double* max_x, double* max_y)
  {
    //ROS_WARN("*****************");
    //传过来的参数 min_x=min_y=1e30，max_x=max_y=-1e30
    //初始化边界
    useExtraBounds(min_x, min_y, max_x, max_y);
    double wx, wy;
    if (!layered_costmap_->isRolling())
    {
      //ROS_WARN("+++++++++++++");
      /*计算静态costmap地图左下角像素的坐标*/
      mapToWorld(0, 0, wx, wy);
      *min_x = std::min(wx, *min_x);
      *min_y = std::min(wy, *min_y);

      /*计算静态costmap地图右上角像素的坐标*/
      mapToWorld(size_x_, size_y_, wx, wy);
      *max_x = std::max(wx, *max_x);
      *max_y = std::max(wy, *max_y);
    }

    else  //设置局部costmap 的范围
    {
      //ROS_WARN("==================");
      /*计算costmap地图左下角像素的坐标*/
      mapToWorld(0, 0, wx, wy);
      *min_x = std::min(wx, *min_x);
      *min_y = std::min(wy, *min_y);
      
      //局部costmap 时，
      if(layered_costmap_->isRolling())
      {
          local_width_=layered_costmap_->getCostmap()->getSizeInMetersX();
          local_height_=layered_costmap_->getCostmap()->getSizeInMetersY();
          //nh.param("width", local_width_, 0.0);
          //nh.param("height", local_height_, 0.0);
      }
      //ROS_WARN("***fei in virtualWall,local_costmap, width=%lf, height=%lf",local_width_,local_height_);
      /*计算静态costmap地图右上角像素的坐标*/
      mapToWorld(local_width_, local_height_, wx, wy);
      //mapToWorld(3.0, 3.0, wx, wy);
      //mapToWorld(0.0, 0.0, wx, wy);
      *max_x = std::max(wx, *max_x);
      *max_y = std::max(wy, *max_y);
    }

    //ROS_WARN("min_point=(%ld,%ld),max_point=(%ld,%ld)",min_x,min_y,max_x,max_y);
  }

  void VirtualWallLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {

    if(virtualWall_pcloud.points.size() < 1)
    {
        return;
    }
    unsigned int mx;
    unsigned int my;

    tf::StampedTransform transform;
    if (!layered_costmap_->isRolling())
    {
        for (int i=0;i < virtualWall_pcloud.points.size();i++)
        {
            if(master_grid.worldToMap(virtualWall_pcloud.points[i].x, virtualWall_pcloud.points[i].y, mx, my))
            {
                master_grid.setCost(mx, my, LETHAL_OBSTACLE);
            }
        }
    }
    else
    {
        try
        {
          //ROS_WARN("++++ %s",virtualWall_pcloud.header.frame_id.c_str());
          //ROS_WARN("local_costmap global frame is: %s",global_frame_.c_str());
          tf_->lookupTransform( global_frame_,virtualWall_pcloud.header.frame_id, ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("11111111%s", ex.what());
          return;
        }
        
        for (int i=0;i < virtualWall_pcloud.points.size();i++)
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
  }

}// end namespace
