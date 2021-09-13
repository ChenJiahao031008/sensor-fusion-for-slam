/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "glog/logging.h"

#include "lidar_localization/models/registration/icp_svd_registration.hpp"

namespace lidar_localization {

ICPSVDRegistration::ICPSVDRegistration(
    const YAML::Node& node
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    // parse params:
    float max_corr_dist = node["max_corr_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPSVDRegistration::ICPSVDRegistration(
    float max_corr_dist,
    float trans_eps,
    float euc_fitness_eps,
    int max_iter
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPSVDRegistration::SetRegistrationParam(
    float max_corr_dist,
    float trans_eps,
    float euc_fitness_eps,
    int max_iter
) {
    // set params:
    max_corr_dist_ = max_corr_dist;
    trans_eps_ = trans_eps;
    euc_fitness_eps_ = euc_fitness_eps;
    max_iter_ = max_iter;

    LOG(INFO) << "ICP SVD params:" << std::endl
              << "max_corr_dist: " << max_corr_dist_ << ", "
              << "trans_eps: " << trans_eps_ << ", "
              << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
              << "max_iter: " << max_iter_
              << std::endl << std::endl;

    return true;
}

bool ICPSVDRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    // 设置需要建立kdtree的点云指针,这里的注册点云是关键帧点云
    input_target_kdtree_->setInputCloud(input_target_);

    return true;
}

bool ICPSVDRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source,
    const Eigen::Matrix4f& predict_pose,
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    // 首先进行粗定位，减少搜索范围
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    // init estimation:
    transformation_.setIdentity();

    //
    // TODO: first option -- implement all computing logic on your own
    //
    // do estimation:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_source_trans = input_source_;
    int curr_iter = 0;
    while (curr_iter < max_iter_) {
        // TODO: apply current estimation:
        // 初始化
        std::vector<Eigen::Vector3f> xs, ys;
        // 本次迭代所用的变换矩阵
        Eigen::Matrix4f tmp_transformation = Eigen::Matrix4f::Identity();
        // 本次迭代所用的点云
        CloudData::CLOUD_PTR tmp_input_source(new CloudData::CLOUD());
        pcl::transformPointCloud(*transformed_input_source, *tmp_input_source, transformation_);

        // TODO: get correspondence:
        int nCorrespondences = GetCorrespondence(tmp_input_source, xs, ys);

        // TODO: do not have enough correspondence -- break:
        if (nCorrespondences < 5)
            break;

        // TODO: update current transform:
        GetTransform(xs, ys, tmp_transformation);

        // TODO: whether the transformation update is significant:
        // 设置前后两次迭代的转换矩阵的最大容差，一旦迭代小于这个最大容查，
        // 则认为已经收敛到最优解，迭代停止。是迭代终止的第二个条件
        if (IsSignificant(tmp_transformation, trans_eps_) == false){
            std::cout << "[DEBUG] transformation update is not significant" << std::endl;
            std::cout << "        iter is : " << curr_iter << std::endl;
            break;
        }

        // TODO: update transformation:
        transformation_ = tmp_transformation * transformation_;

        ++curr_iter;
    }

    // set output:
    // 对粗定位得到的旋转矩阵进行矫正
    result_pose = transformation_ * predict_pose;

    //归一化（消除warning，保持旋转矩阵特性，很重要！）
    Eigen::Quaternionf qr(result_pose.block<3, 3>(0, 0));
    qr.normalize();
    Eigen::Vector3f t = result_pose.block<3, 1>(0, 3);
    result_pose.setIdentity();
    result_pose.block<3, 3>(0, 0) = qr.toRotationMatrix();
    result_pose.block<3, 1>(0, 3) = t;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

    return true;
}

size_t ICPSVDRegistration::GetCorrespondence(
    const CloudData::CLOUD_PTR &input_source,
    std::vector<Eigen::Vector3f> &xs,
    std::vector<Eigen::Vector3f> &ys
) {
    const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

    size_t num_corr = 0;

    // //  TODO: set up point correspondence

    // 只寻找最近的一个点
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);         //存储查询点近邻索引
    std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
    // CloudData::CLOUD_PTR kdtreeCloud(new CloudData::CLOUD());
    auto kdtreeCloud = input_target_kdtree_->getInputCloud();

    for (size_t i = 0; i < input_source->points.size(); ++i)
    {
        pointIdxNKNSearch.clear();
        pointNKNSquaredDistance.clear();
        // 最近邻存在
        if (input_target_kdtree_->nearestKSearch(input_source->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){

            float squaredDistance = pointNKNSquaredDistance.front();
            int idxNKNSearch = pointIdxNKNSearch.front();

            // 距离小于阈值
            if (squaredDistance <= MAX_CORR_DIST_SQR)
            {
                Eigen::Vector3f ns, raw;
                ns <<
                kdtreeCloud->points[idxNKNSearch].x,
                kdtreeCloud->points[idxNKNSearch].y,
                kdtreeCloud->points[idxNKNSearch].z;

                raw << input_source->points[i].x,
                       input_source->points[i].y,
                       input_source->points[i].z;

                xs.push_back(ns);
                ys.push_back(raw);
                num_corr++;
            }

        }
    }

    return num_corr;
}

void ICPSVDRegistration::GetTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation_
) {
    const size_t N = xs.size();
    Eigen::Vector3f mu_x = Eigen::Vector3f::Zero();
    Eigen::Vector3f mu_y = Eigen::Vector3f::Zero();

    // TODO: find centroids of mu_x and mu_y:
    assert(xs.size() == ys.size());
    for (size_t i=0 ; i<N; ++i){
        mu_x = mu_x + xs[i];
        mu_y = mu_y + ys[i];
    }
    mu_x = mu_x * 1.0 / N;
    mu_y = mu_y * 1.0 / N;

    // TODO: build H:
    Eigen::Matrix3Xf X(3, xs.size());
    Eigen::Matrix3Xf Y(3, ys.size());
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i=0; i<N; ++i){
        X.block(0, i, 3, 1) = xs[i] - mu_x;
        Y.block(0, i, 3, 1) = ys[i] - mu_y;
    }
    H = Y * X.transpose();

    // TODO: solve R:
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf U = svd.matrixU();
    Eigen::MatrixXf V = svd.matrixV();
    // Eigen::MatrixXf A = svd.singularValues();
    Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
    A(2, 2) = (V * U.transpose()).determinant();
    Eigen::Matrix3f R = V*A*(U.transpose());

    // TODO: solve t:
    Eigen::Vector3f t = mu_x - R*mu_y;

    // TODO: set output:
    transformation_ = Eigen::Matrix4f::Identity();
    transformation_.block(0,0,3,3) = R;
    transformation_.block(0,3,3,1) = t;

}

bool ICPSVDRegistration::IsSignificant(
    const Eigen::Matrix4f &transformation,
    const float trans_eps
) {
    // a. translation magnitude -- norm:
    float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
    // b. rotation magnitude -- angle:
    float rotation_magnitude = fabs(
        acos(
            (transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f
        )
    );

    return (
        (translation_magnitude > trans_eps) ||
        (rotation_magnitude > trans_eps)
    );
}

} // namespace lidar_localization
