/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */


#include <Eigen/Dense>

#include "glog/logging.h"

#include "lidar_localization/models/registration/sicp/ICP.h"

#include "lidar_localization/models/registration/sicp/scip_registration.hpp"

namespace lidar_localization {

SICPRegistration::SICPRegistration(
    const YAML::Node& node
):input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
{
    // parse params:
    // params_.p = node['p'].as<float>();
    // params_.mu = node['mu'].as<float>();
    // params_.alpha = node['alpha'].as<float>();
    // params_.max_mu = node['max_mu'].as<float>();
    // params_.max_icp = node['max_icp'].as<int>();
    // params_.max_outer = node['max_outer'].as<int>();
    // params_.max_inner = node['max_inner'].as<int>();
    // params_.stop = node['stop'].as<float>();
}

bool SICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    input_target_kdtree_->setInputCloud(input_target_);
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    // normalEstimator.setInputCloud(input_target_);
    // normalEstimator.setSearchMethod(input_target_kdtree_);
    // normalEstimator.setKSearch(15);
    // normalEstimator.compute(*normals);
    return true;
}

bool SICPRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source,
    const Eigen::Matrix4f& predict_pose,
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    //
    // TODO: second option -- adapt existing implementation
    //
    // TODO: format inputs for SICP:
    // auto cloud_target = input_target_kdtree_->getInputCloud();
    // Eigen::Matrix3Xd X(3, input_source_->size());  // source, transformed
    // Eigen::Matrix3Xd Y(3, cloud_target->size());   // target
    // Eigen::Matrix3Xd N(3, target_normals->size()); // target normal

    // for (int i = 0; i < input_source_->size(); i++)
    // {
    //     X(0, i) = input_source_->points[i].x;
    //     X(1, i) = input_source_->points[i].y;
    //     X(2, i) = input_source_->points[i].z;
    // }
    // for (int i = 0; i < cloud_target->size(); i++)
    // {
    //     Y(0, i) = cloud_target->points[i].x;
    //     Y(1, i) = cloud_target->points[i].y;
    //     Y(2, i) = cloud_target->points[i].z;
    // }
    // for (int i = 0; i < target_normals->size(); i++)
    // {
    //     N(0, i) = target_normals->points[i].normal_x;
    //     N(1, i) = target_normals->points[i].normal_y;
    //     N(2, i) = target_normals->points[i].normal_z;
    // }

    // int curr_iter = 0;
    // while (curr_iter < params_.max_icp)
    // {
    //     ICP::point_to_plane(X, Y, N, params_); // sparse ICP with normals

    //     for (int i = 0; i < cloud_source_trans->size(); i++)
    //     {
    //         cloud_source_trans->points[i].x = X(0, i);
    //         cloud_source_trans->points[i].y = X(1, i);
    //         cloud_source_trans->points[i].z = X(2, i);
    //     }
    //     curr_iter++;
    // }

    // TODO: SICP registration:

    // set output:
    result_pose = transformation_ * predict_pose;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

    return true;
}

} // namespace lidar_localization
