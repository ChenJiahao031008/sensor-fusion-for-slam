// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

//
// TODO: implement analytic Jacobians for LOAM residuals in this file
//

#include <eigen3/Eigen/Dense>

//
// TODO: Sophus is ready to use if you have a good undestanding of Lie algebra.
//
#include <sophus/so3.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

static Eigen::Matrix<double, 3, 3> skew(Eigen::Matrix<double, 3, 1> &mat_in)
{
	Eigen::Matrix<double, 3, 3> skew_mat;
	skew_mat.setZero();
	skew_mat(0, 1) = -mat_in(2);
	skew_mat(0, 2) = mat_in(1);
	skew_mat(1, 2) = -mat_in(0);
	skew_mat(1, 0) = mat_in(2);
	skew_mat(2, 0) = -mat_in(1);
	skew_mat(2, 1) = mat_in(0);
	return skew_mat;
};

struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		// 考虑运动补偿，ktti点云已经补偿过所以可以忽略下面的对四元数slerp插值以及对平移的线性插值
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		// Odometry线程时：下面是将当前帧Lidar坐标系下的cp点变换到上一帧的Lidar坐标系下
		// 然后在上一帧的Lidar坐标系计算点到线的残差距离
		// Mapping线程时：下面是将当前帧Lidar坐标系下的cp点变换到world坐标系下
		// 然后在world坐标系下计算点到线的残差距离
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		// residual[0] = nu.x() / de.norm();
		// residual[1] = nu.y() / de.norm();
		// residual[2] = nu.z() / de.norm();
		residual[0] = nu.norm( )* 1.0 / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 1, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};


struct LidarDistanceFactor
{

	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_)
						: curr_point(curr_point_), closed_point(closed_point_){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;


		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d closed_point;
};

// 第1个是残差的维度, 第2个是待优化参数的维度。这里是4，表示旋转的四元数, 对应parameters[0]
// 第3个也是待优化参数的维度。这里是3，表示平移量的xyz, 对应parameters[1]。
class LidarEdgeCostFunction : public ceres::SizedCostFunction<1, 4, 3>
{
public:
	LidarEdgeCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
						  Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {};
	virtual ~LidarEdgeCostFunction() {};
	virtual bool Evaluate(double const *const *parameters,
						  double *residuals,
						  double **jacobians) const
	{
		// const double *p = parameters[0];
		Eigen::Map<const Eigen::Quaterniond> q(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t(parameters[1]);
		Eigen::Quaternion<double> q_last_curr = q;
		Eigen::Quaternion<double> q_identity{1.0, 0.0, 0.0, 0.0};
		q_last_curr = q_identity.slerp(s, q_last_curr).normalized();
		Eigen::Vector3d t_last_curr{s * t[0], s * t[1], s * t[2]};

		Eigen::Vector3d cp  = curr_point;
		Eigen::Vector3d lpa = last_point_a;
		Eigen::Vector3d lpb = last_point_b;
		Eigen::Vector3d lp = q_last_curr * cp + t_last_curr;
		Eigen::Vector3d rp = q_last_curr * cp;

		Eigen::Vector3d nu = (lp - lpb).cross(lp - lpa);
		Eigen::Vector3d de = lpa - lpb;
		double res = (nu.norm() * 1.0 / de.norm());
		residuals[0] = res;

		// Compute the Jacobian if asked for.
		// 旋转在前，平移在后
		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Matrix3d skew_de = skew(de);
				Eigen::Matrix3d dp_by_so3 = -skew(rp);
				Eigen::Matrix<double, 1, 3> nu_normalized_t = nu.normalized().transpose();
				Eigen::Map<Eigen::Matrix<double, 1, 4> > J_se3(jacobians[0]);
				J_se3.setZero();
				J_se3.block<1, 3>(0, 0) = nu_normalized_t * skew_de * dp_by_so3 / de.norm();
			}

			if (jacobians[1] != NULL)
			{
				// Eigen::Matrix3d skew_rp = skew(lp);
				Eigen::Matrix3d dp_by_t = Eigen::Matrix3d::Identity();
				Eigen::Map<Eigen::Matrix<double, 1, 3> > J_se3(jacobians[1]);
				J_se3.setZero();
				Eigen::Matrix3d skew_de = skew(de);
				J_se3.block<1, 3>(0, 0) = (nu.normalized().transpose()) * skew_de * dp_by_t /  de.norm();
			}
		}
		return true;
	};



public:
	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

class LidarEdgeCostFunctionVector : public ceres::SizedCostFunction<3, 4, 3>
{
public:
	LidarEdgeCostFunctionVector(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
						  Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_){};
	virtual ~LidarEdgeCostFunctionVector(){};
	virtual bool Evaluate(double const *const *parameters,
						  double *residuals,
						  double **jacobians) const
	{
		Eigen::Map<const Eigen::Quaterniond> q(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t(parameters[1]);
		Eigen::Quaternion<double> q_last_curr = q;
		Eigen::Quaternion<double> q_identity{1.0, 0.0, 0.0, 0.0};
		q_last_curr = q_identity.slerp(s, q_last_curr).normalized();
		Eigen::Vector3d t_last_curr{s * t[0], s * t[1], s * t[2]};

		Eigen::Vector3d cp = curr_point;
		Eigen::Vector3d lpa = last_point_a;
		Eigen::Vector3d lpb = last_point_b;
		Eigen::Vector3d lp = q_last_curr * cp + t_last_curr;
		Eigen::Vector3d rp = q_last_curr * cp;

		Eigen::Vector3d nu = (lp - lpb).cross(lp - lpa);
		Eigen::Vector3d de = lpa - lpb;
		double de_norm = de.norm();
		// residuals[0] = nu_morm / de_norm;
		residuals[0] = nu.x() / de.norm();
		residuals[1] = nu.y() / de.norm();
		residuals[2] = nu.z() / de.norm();

		// Compute the Jacobian if asked for.
		// 旋转在前，平移在后
		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Matrix3d skew_rp = skew(rp);
				Eigen::Matrix<double, 3, 3> dp_by_so3;
				dp_by_so3.block<3, 3>(0, 0) = -skew_rp;
				Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > J_se3(jacobians[0]);
				J_se3.setZero();
				Eigen::Matrix3d skew_de = skew(de);
				J_se3.block<3, 3>(0, 0) = skew_de * dp_by_so3 / de_norm;
			}

			if (jacobians[1] != NULL)
			{
				// Eigen::Matrix3d skew_rp = skew(lp);
				Eigen::Matrix<double, 3, 3> dp_by_t;
				dp_by_t.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
				Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > J_se3(jacobians[1]);
				J_se3.setZero();
				Eigen::Matrix3d skew_de = skew(de);
				J_se3.block<3, 3>(0, 0) = skew_de * dp_by_t / de_norm;
			}
		}
		return true;
	};



public:
	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

class LidarPlaneCostFunction : public ceres::SizedCostFunction<1, 4, 3>
{
public:
	LidarPlaneCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
						   Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	};

	virtual ~LidarPlaneCostFunction(){};
	virtual bool Evaluate(double const *const *parameters,
						  double *residuals,
						  double **jacobians) const
	{
		const double *q = parameters[0];
		const double *t = parameters[1];

		Eigen::Matrix<double, 3, 1> cp{curr_point.x(), curr_point.y(), curr_point.z()};
		Eigen::Matrix<double, 3, 1> lpj{last_point_j.x(), last_point_j.y(), last_point_j.z()};
		Eigen::Matrix<double, 3, 1> ljm{ljm_norm.x(), ljm_norm.y(), ljm_norm.z()};

		Eigen::Quaternion<double> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<double> q_identity{1.0, 0.0, 0.0, 0.0};
		q_last_curr = q_identity.slerp(s, q_last_curr);
		Eigen::Matrix<double, 3, 1> t_last_curr{s * t[0], s * t[1], s * t[2]};
		Eigen::Matrix<double, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;
		Eigen::Matrix<double, 3, 1> rp = q_last_curr * cp;

		residuals[0] = (lp - lpj).dot(ljm);
		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Matrix3d skew_point_w = skew(rp);
				Eigen::Matrix<double, 3, 3> dp_by_so3 = Eigen::Matrix3d::Identity();
				Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor> > J_se3(jacobians[0]);
				J_se3.setZero();
				J_se3.block<1, 3>(0, 0) = ljm_norm.transpose() * dp_by_so3;
			}

			if (jacobians[1] != NULL)
			{
				// Eigen::Matrix3d skew_point_w = skew(rp);
				Eigen::Matrix<double, 3, 3> dp_by_t = Eigen::Matrix3d::Identity();
				Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> > J_se3(jacobians[0]);
				J_se3.setZero();
				J_se3.block<1, 3>(0, 0) = ljm_norm.transpose() * dp_by_t;
			}
		}
		return true;
	};


public:
	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

// 参考博客：https://blog.csdn.net/hzwwpgmwy/article/details/86490556
//         https://blog.csdn.net/weixin_43991178/article/details/100532618
class PoseSE3Parameterization : public ceres::LocalParameterization
{
public:
	PoseSE3Parameterization() {}
	virtual ~PoseSE3Parameterization() {}
	virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
	{
		Eigen::Map<const Eigen::Vector3d> trans(x + 4);

		Eigen::Quaterniond delta_q;
		Eigen::Vector3d delta_t;
		getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double, 6, 1> >(delta), delta_q, delta_t);
		Eigen::Map<const Eigen::Quaterniond> quater(x);
		Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
		Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

		quater_plus = delta_q * quater;
		trans_plus = delta_q * trans + delta_t;
		return true;
	};

	virtual bool ComputeJacobian(const double *x, double *jacobian) const
	{
		// 这里的雅各比矩阵维度是 GlobalSize x LocalSize
		// RowMajor表示行主导，猜测表示最后1维度没有用，实际上就是从6x6变成7x6
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > j(jacobian);
		(j.topRows(6)).setIdentity();
		(j.bottomRows(1)).setZero();
		return true;
	};
	// 自由度（可能有冗余） = 4 + 3
	virtual int GlobalSize() const { return 7; }
	// 正切空间的自由度 = 3 + 3
	virtual int LocalSize() const { return 6; }

	// TODO：这块暂时没有看懂，不过猜测是从se3上分配q和t的微小变量，即变换矩阵转四元数和平移
	// https://blog.csdn.net/liuerin/article/details/116991228
	void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t) const
	{
		Eigen::Vector3d omega(se3.data());		 // 前3维
		Eigen::Vector3d upsilon(se3.data() + 3); // 后三维
		Eigen::Matrix3d Omega = skew(omega);	 // 反对称矩阵

		double theta = omega.norm();
		double half_theta = 0.5 * theta;

		double imag_factor;
		double real_factor = cos(half_theta);
		if (theta < 1e-10)
		{
			double theta_sq = theta * theta;
			double theta_po4 = theta_sq * theta_sq;
			imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
		}
		else
		{
			double sin_half_theta = sin(half_theta);
			imag_factor = sin_half_theta / theta;
		}

		q = Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());

		Eigen::Matrix3d J;
		if (theta < 1e-10)
		{
			J = q.matrix();
		}
		else
		{
			Eigen::Matrix3d Omega2 = Omega * Omega;
			J = (Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * Omega + (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
		}

		t = J * upsilon;
	};


};

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
	EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_)
	{
	}

	virtual ~EdgeAnalyticCostFunction() {}

	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
		Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
		Eigen::Vector3d lp;
		lp = q_last_curr * curr_point + t_last_curr;

		Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
		Eigen::Vector3d de = last_point_a - last_point_b;
		double de_norm = de.norm();
		residuals[0] = nu.norm() / de_norm;

		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Matrix3d skew_lp = skew(lp);
				Eigen::Matrix<double, 3, 6> dp_by_se3;
				dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
				(dp_by_se3.block<3, 3>(0, 3)).setIdentity();
				// 为什么这里取前7列但是只用了前6列，最后一列为0，猜测：待优化变量为7
				Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
				J_se3.setZero();
				Eigen::Matrix3d skew_de = skew(de);
				J_se3.block<1, 6>(0, 0) = -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
			}
		}

		return true;
	};

public:
	Eigen::Vector3d curr_point;
	Eigen::Vector3d last_point_a;
	Eigen::Vector3d last_point_b;
	double s;
};

class SurfAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
	SurfAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
							 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	};

	virtual ~SurfAnalyticCostFunction() {};

	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
		Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
		Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;
		residuals[0] = ljm_norm.dot(point_w);

		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Matrix3d skew_point_w = skew(point_w);
				Eigen::Matrix<double, 3, 6> dp_by_se3;
				dp_by_se3.block<3, 3>(0, 0) = -skew_point_w;
				(dp_by_se3.block<3, 3>(0, 3)).setIdentity();
				Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
				J_se3.setZero();
				J_se3.block<1, 6>(0, 0) = ljm_norm.transpose() * dp_by_se3;
			}
		}
		return true;
	};

public:
	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};


