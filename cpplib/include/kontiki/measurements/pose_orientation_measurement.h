//
// Created by tuomas on 2022-06-03.
//

#ifndef KONTIKI_POSE_ORIENTATION_MEASUREMENT_H
#define KONTIKI_POSE_ORIENTATION_MEASUREMENT_H

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
class PoseOrientationMeasurement {
  using Quat = Eigen::Quaternion<double>;

 public:
  PoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t, const Quat &q)
    : pose_(pose), t(t), q_(q) {}
  PoseOrientationMeasurement(std::shared_ptr<PoseModel> pose, double t, const Eigen::Vector4d &qvec)
    : pose_(pose), t(t), q_(Eigen::Quaternion<double>(qvec(0), qvec(1), qvec(2), qvec(3))) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Quaternion<T> Measure(const type::Pose<PoseModel, T> &pose,
                               const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    Eigen::Quaternion<T> q_M_I = trajectory.Orientation(T(t));
    const Eigen::Quaternion<T> q_L_I = pose.relative_orientation();
    Eigen::Quaternion<T> q_M_L = q_M_I * q_L_I.conjugate();
    return q_M_L;
  }

  template<typename TrajectoryModel>
  Vector3 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Measure<TrajectoryModel, double>(*pose_, trajectory);
  };

  template<typename TrajectoryModel, typename T>
  T Error(const type::Pose<PoseModel, T> &pose, const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    Eigen::Quaternion<T> qhat = Measure<TrajectoryModel, T>(trajectory, pose);
    return q_.cast<T>().angularDistance(qhat);
  }

  // Measurement data
  std::shared_ptr<PoseModel> pose_;
  double t;
  Quat q_;

 protected:
  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const PoseOrientationMeasurement<PoseModel> &m) : measurement(m) {}

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto pose = entity::Map<PoseModel, T>(&params[offset], pose_meta);

      residual[0] = measurement.Error<TrajectoryModel, T>(trajectory, pose);
      return true;
    }

    const PoseOrientationMeasurement& measurement;
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
    estimator.AddTrajectoryForTimes({{t, t}}, residual->trajectory_meta, parameter_info);
    pose_->AddToProblem(estimator.problem(), {{t, t}}, residual->pose_meta, parameter_info);


    for (auto& pi : parameter_info) {
      cost_function->AddParameterBlock(pi.size);
    }

    // Add measurement
    cost_function->SetNumResiduals(1);
    // If we had any measurement parameters to set, this would be the place

    // Give residual block to estimator problem
    estimator.problem().AddResidualBlock(cost_function,
                                         nullptr,
                                         entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
  }

  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

}  // namespace measurements
}  // namespace kontiki

#endif //KONTIKI_POSE_ORIENTATION_MEASUREMENT_H
