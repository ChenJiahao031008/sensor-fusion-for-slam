/*
 * @Description: 订阅odometry数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 */
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization{
OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
}

void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
    buff_mutex_.lock();
    PoseData pose_data;
    VelocityData velocity_data;
    pose_data.time = odom_msg_ptr->header.stamp.toSec();
    velocity_data.time = pose_data.time;

    //set the position
    pose_data.pose(0,3) = odom_msg_ptr->pose.pose.position.x;
    pose_data.pose(1,3) = odom_msg_ptr->pose.pose.position.y;
    pose_data.pose(2,3) = odom_msg_ptr->pose.pose.position.z;

    Eigen::Quaternionf q;
    q.x() = odom_msg_ptr->pose.pose.orientation.x;
    q.y() = odom_msg_ptr->pose.pose.orientation.y;
    q.z() = odom_msg_ptr->pose.pose.orientation.z;
    q.w() = odom_msg_ptr->pose.pose.orientation.w;
    pose_data.pose.block<3,3>(0,0) = q.matrix();

    velocity_data.linear_velocity.x = odom_msg_ptr->twist.twist.linear.x;
    velocity_data.linear_velocity.y = odom_msg_ptr->twist.twist.linear.y;
    velocity_data.linear_velocity.z = odom_msg_ptr->twist.twist.linear.z;
    velocity_data.angular_velocity.x = odom_msg_ptr->twist.twist.angular.x;
    velocity_data.angular_velocity.y = odom_msg_ptr->twist.twist.angular.y;
    velocity_data.angular_velocity.z = odom_msg_ptr->twist.twist.angular.z;

    new_pose_data_.push_back(pose_data);
    new_velocity_data_.push_back(velocity_data);
    buff_mutex_.unlock();
}

void OdometrySubscriber::ParseData(std::deque<PoseData>& pose_data_buff) {
    buff_mutex_.lock();
    if (new_pose_data_.size() > 0) {
        pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
        new_pose_data_.clear();
    }
    buff_mutex_.unlock();
}

void OdometrySubscriber::ParseData2(
    std::deque<PoseData> &pose_data_buff,
    std::deque<VelocityData> &velocity_data_buff)
{
    buff_mutex_.lock();
    if (new_pose_data_.size() > 0)
    {
        pose_data_buff.insert(
            pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
        velocity_data_buff.insert(
            velocity_data_buff.end(), new_velocity_data_.begin(), new_velocity_data_.end());

        new_pose_data_.clear();
        new_velocity_data_.clear();
    }
    buff_mutex_.unlock();
}
}
