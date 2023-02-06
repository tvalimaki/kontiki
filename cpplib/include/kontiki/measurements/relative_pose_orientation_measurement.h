//
// Created by tuomas on 2023-01-17.
//

#ifndef KONTIKI_RELATIVE_POSE_ORIENTATION_MEASUREMENT_H
#define KONTIKI_RELATIVE_POSE_ORIENTATION_MEASUREMENT_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <iostream>
#include <kontiki/trajectories/trajectory.h>
#include <kontiki/trajectory_estimator.h>
#include "../sensors/pose.h"

namespace kontiki {
namespace measurements {

template<typename PoseModel>
class RelativePoseOrientationMeasurement {
  using Quat = Eigen::Quaternion<double>;

 public:
  RelativePoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Quat &q, double loss, double weight)
    : pose_(pose), t0(t0), t1(t1), q_(q), loss_function_(loss), weight(weight) {}
  RelativePoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Eigen::Vector4d &qvec, double loss, double weight)
    : pose_(pose), t0(t0), t1(t1), q_(Eigen::Quaternion<double>(qvec(0), qvec(1), qvec(2), qvec(3))), loss_function_(loss), weight(weight) {}

  RelativePoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Quat &q, double loss)
    : pose_(pose), t0(t0), t1(t1), q_(q), loss_function_(loss), weight(1.0) {}
  RelativePoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Eigen::Vector4d &qvec, double loss)
    : pose_(pose), t0(t0), t1(t1), q_(Eigen::Quaternion<double>(qvec(0), qvec(1), qvec(2), qvec(3))), loss_function_(loss), weight(1.0) {}

  RelativePoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Quat &q)
    : pose_(pose), t0(t0), t1(t1), q_(q), loss_function_(0.5), weight(1.0) {}
  RelativePoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Eigen::Vector4d &qvec)
    : pose_(pose), t0(t0), t1(t1), q_(Eigen::Quaternion<double>(qvec(0), qvec(1), qvec(2), qvec(3))), loss_function_(0.5), weight(1.0) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Quaternion<T> Measure(const type::Pose<PoseModel, T> &pose,
                               const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    Eigen::Quaternion<T> q_M_I0 = trajectory.Orientation(T(t0));
    Eigen::Quaternion<T> q_M_I1 = trajectory.Orientation(T(t1));
    Eigen::Quaternion<T> q_I0_I1 = q_M_I0.conjugate() * q_M_I1;
    const Eigen::Quaternion<T> q_L_I = pose.relative_orientation();
    Eigen::Quaternion<T> q_L0_L1 = q_L_I * q_I0_I1 * q_L_I.conjugate();
    return q_L0_L1;
  }

  template<typename TrajectoryModel>
  Quat Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Measure<TrajectoryModel, double>(*pose_, trajectory);
  };

  template<typename TrajectoryModel, typename T>
  T Error(const type::Pose<PoseModel, T> &pose, const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    Eigen::Quaternion<T> qhat = Measure<TrajectoryModel, T>(pose, trajectory);
    return T(weight) * q_.cast<T>().angularDistance(qhat);
  }

  template<typename TrajectoryModel>
  double Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Error<TrajectoryModel, double>(*pose_, trajectory);
  }

  // Measurement data
  std::shared_ptr<PoseModel> pose_;
  double t0;
  double t1;
  double weight;
  Quat q_;

 protected:
  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const RelativePoseOrientationMeasurement<PoseModel> &m) : measurement(m) {}

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto pose = entity::Map<PoseModel, T>(&params[offset], pose_meta);

      residual[0] = measurement.Error<TrajectoryModel, T>(pose, trajectory);
      return true;
    }

    const RelativePoseOrientationMeasurement& measurement;
    typename TrajectoryModel::Meta trajectory_meta;
    typename PoseModel::Meta pose_meta;
  };  // Residual;

  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<entity::ParameterInfo<double>> parameter_info;

    // Add trajectory to problem
    estimator.AddTrajectoryForTimes({{t0, t0},{t1, t1}}, residual->trajectory_meta, parameter_info);
    pose_->AddToProblem(estimator.problem(), {{t0, t0},{t1, t1}}, residual->pose_meta, parameter_info);


    for (auto& pi : parameter_info) {
      cost_function->AddParameterBlock(pi.size);
    }

    // Add measurement
    cost_function->SetNumResiduals(1);
    // If we had any measurement parameters to set, this would be the place

    // Give residual block to estimator problem
    estimator.problem().AddResidualBlock(cost_function,
                                         &loss_function_,
                                         entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
  }

  // The loss function is not a pointer since the Problem does not take ownership.
  ceres::CauchyLoss loss_function_;

  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

}  // namespace measurements
}  // namespace kontiki

#endif //KONTIKI_RELATIVE_POSE_ORIENTATION_MEASUREMENT_H
