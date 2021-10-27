/*
 * @Description: ceres residual block for lidar frontend relative pose measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGRelativePose : public ceres::SizedCostFunction<6, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;

  FactorPRVAGRelativePose(void) {};

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Map<const Eigen::Vector3d>     pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori_i = Sophus::SO3d::exp(log_ori_i);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d>     pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d                    ori_j = Sophus::SO3d::exp(log_ori_j);

    //
    // parse measurement:
    //
		const Eigen::Vector3d     &pos_ij = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &log_ori_ij = m_.block<3, 1>(INDEX_R, 0);
    const Sophus::SO3d         ori_ij = Sophus::SO3d::exp(log_ori_ij);

    // TODO: get square root of information matrix:
    // Cholesky 分解 : http://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    Eigen::LLT< Eigen::Matrix<double, 6, 6> > LowerI(I_);
    // sqrt_info 为上三角阵
    Eigen::Matrix<double, 6, 6> sqrt_info = LowerI.matrixL().transpose();

    //
    // TODO: compute residual:
    // Eigen::Matrix<double, 6, 1> resid;
    // resid.setZero();
    Eigen::Map<Eigen::Matrix<double, 6, 1> > resid(residuals);
    const Eigen::Matrix3d oriRT_i = ori_i.inverse().matrix();

    resid.block<3, 1>(INDEX_P, 0) = oriRT_i * (pos_j - pos_i) - pos_ij;
    resid.block<3, 1>(INDEX_R, 0) = ( ori_i.inverse() * ori_j * ori_ij.inverse() ).log();
    // residuals[0] = resid;

    //
    // TODO: compute jacobians:
    if ( jacobians ) {
      // compute shared intermediate results:
      Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> jacobian_i(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> jacobian_j(jacobians[1]);
      jacobian_i.setZero();
      jacobian_j.setZero();

      const Eigen::Vector3d deltaR = resid.block<3, 1>(INDEX_R, 0);
      const Eigen::Matrix3d J_r_inv = JacobianRInv(deltaR);


      if ( jacobians[0] ) {
        jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -oriRT_i;
        // jacobian_i.block<3, 3>(INDEX_P, INDEX_R) =  oriRT_i * Sophus::SO3d::hat(pos_j - pos_i);
        jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv * (ori_ij * ori_j.inverse() * ori_i ).matrix();
      }

      if ( jacobians[1] ) {
        jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = oriRT_i;
        jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv * ori_ij.matrix();
      }

      //
      // TODO: correct residuals by square root of information matrix:
      jacobian_i = sqrt_info * jacobian_i;
      jacobian_j = sqrt_info * jacobian_j;

    }

    resid = sqrt_info * resid;

    return true;
  }

private:
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
      Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

      double theta = w.norm();

      if ( theta > 1e-5 ) {
          Eigen::Vector3d k = w.normalized();
          Eigen::Matrix3d K = Sophus::SO3d::hat(k);

          J_r_inv = J_r_inv
                    + 0.5 * K
                    + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;
      }

      return J_r_inv;
  }

  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
