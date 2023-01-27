//
// Created by tuomas on 2022-06-03.
//

#ifndef KONTIKI_POSE_POSITION_MEASUREMENT_H
#define KONTIKI_POSE_POSITION_MEASUREMENT_H

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
class PosePositionMeasurement {
  using Vector3 = Eigen::Matrix<double, 3, 1>;

 public:
  PosePositionMeasurement(std::shared_ptr<PoseModel> pose, double t, const Vector3 &p)
    : pose_(pose), t(t), p_(p), weight(1.0) {}

  PosePositionMeasurement(std::shared_ptr<PoseModel> pose, double t, const Vector3 &p, double weight)
    : pose_(pose), t(t), p_(p), weight(weight) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Measure(const type::Pose<PoseModel, T> &pose,
                                 const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    int flags = trajectories::EvaluationFlags::EvalPosition | trajectories::EvaluationFlags::EvalOrientation;
    auto T_M_I = trajectory.Evaluate(T(t), flags);

    const Eigen::Matrix<T, 3, 1> p_L_I = pose.relative_position();
    const Eigen::Quaternion<T> q_L_I = pose.relative_orientation();
    Eigen::Matrix<T, 3, 1> p_I_L = q_L_I.conjugate() * (-p_L_I);

    Eigen::Quaternion<T> q_W_I = q_L_I * T_M_I->orientation;
    Eigen::Matrix<T, 3, 1> p_W_I = q_L_I * T_M_I->position + p_L_I;
    Eigen::Matrix<T, 3, 1> p_W_L = q_W_I * p_I_L + p_W_I;

    return p_W_L;
  }

  template<typename TrajectoryModel>
  Vector3 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Measure<TrajectoryModel, double>(*pose_, trajectory);
  };

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Error(const type::Pose<PoseModel, T> &pose,
                               const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    Eigen::Matrix<T, 3, 1> p_M_L = p_.cast<T>();
    return T(weight) * (p_M_L - Measure<TrajectoryModel, T>(pose, trajectory));
  }

  template<typename TrajectoryModel>
  Eigen::Matrix<double, 3, 1> Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Error<TrajectoryModel, double>(*pose_, trajectory);
  }

  // Measurement data
  std::shared_ptr<PoseModel> pose_;
  double t;
  Vector3 p_;
  double weight;

 protected:
  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const PosePositionMeasurement<PoseModel> &m) : measurement(m) {}

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto pose = entity::Map<PoseModel, T>(&params[offset], pose_meta);

      Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residual);
      r = measurement.Error<TrajectoryModel, T>(pose, trajectory);
      return true;
    }

    const PosePositionMeasurement& measurement;
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
    cost_function->SetNumResiduals(3);
    // If we had any measurement parameters to set, this would be the place

    // Give residual block to estimator problem
    estimator.problem().AddResidualBlock(cost_function,
                                         nullptr,
                                         entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
  }

  // The loss function is not a pointer since the Problem does not take ownership.
  // ceres::CauchyLoss loss_function_;

  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

}  // namespace measurements
}  // namespace kontiki

#endif  //KONTIKI_POSE_POSITION_MEASUREMENT_H
