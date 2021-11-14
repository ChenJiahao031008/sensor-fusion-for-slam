/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/map", "/lidar", 100);

    front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        // init lidar odometry:
        // TODO: 雷达位置有固定的偏移，需要检查这个偏移值
        // Eigen::Quaternionf q(0.724454, 0.010385, 0.010385, 0.689161);
        Eigen::Matrix4f init = Eigen::Matrix4f::Identity();
        // init.block<3,3>(0,0) = q.matrix();
        // std::cout << "test:\n" << init << std::endl;
        front_end_ptr_->SetInitPose(init);
    }

    // update lidar odometry using current undistorted measurement:
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    // Eigen::Matrix4f tmp;
    // tmp << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // laser_odometry_ = tmp * laser_odometry_;
    // Eigen::Matrix3f tmp;
    // tmp << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    // laser_odometry_.block<3, 3>(0, 0) = tmp * laser_odometry_.block<3, 3>(0, 0);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}
}
