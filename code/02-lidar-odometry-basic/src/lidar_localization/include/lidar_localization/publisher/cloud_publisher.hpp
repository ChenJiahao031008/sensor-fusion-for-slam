/*
 * @Description: point cloud publisher
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class CloudPublisher {
  public:
    // frame_id：用来告诉你，发布的数据是来自哪一个坐标系的。
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
    CloudPublisher() = default;
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
}
#endif
