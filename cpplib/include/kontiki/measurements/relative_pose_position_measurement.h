//
// Created by tuomas on 2023-01-17.
//

#ifndef KONTIKI_RELATIVE_POSE_POSITION_MEASUREMENT_H
#define KONTIKI_RELATIVE_POSE_POSITION_MEASUREMENT_H

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
class RelativePosePositionMeasurement {
  using Vector3 = Eigen::Matrix<double, 3, 1>;

 public:
  RelativePosePositionMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Vector3 &p, double loss, Vector3 weight)
    : pose_(pose), t0(t0), t1(t1), p_(p), loss_function_(loss), weight(weight) {}
  RelativePosePositionMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Vector3 &p, double loss)
    : pose_(pose), t0(t0), t1(t1), p_(p), loss_function_(loss), weight(Vector3::Constant(1.0)) {}
  RelativePosePositionMeasurement(std::shared_ptr<PoseModel> pose, double t0, double t1, const Vector3 &p)
    : pose_(pose), t0(t0), t1(t1), p_(p), loss_function_(0.5), weight(Vector3::Constant(1.0)) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Measure(const type::Pose<PoseModel, T> &pose,
                                 const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    int flags = trajectories::EvaluationFlags::EvalPosition | trajectories::EvaluationFlags::EvalOrientation;
    auto T_M_I0 = trajectory.Evaluate(T(t0), flags);
    auto T_M_I1 = trajectory.Evaluate(T(t1), flags);
    Eigen::Quaternion<T> q_I0_I1 = T_M_I0->orientation.conjugate() * T_M_I1->orientation;
    Eigen::Matrix<T, 3, 1> p_I0_M = T_M_I0->orientation.conjugate() * (-T_M_I0->position);
    Eigen::Matrix<T, 3, 1> p_I0_I1 = T_M_I0->orientation.conjugate() * T_M_I1->position + p_I0_M;

    const Eigen::Matrix<T, 3, 1> p_L_I = pose.relative_position();
    const Eigen::Quaternion<T> q_L_I = pose.relative_orientation();
    Eigen::Matrix<T, 3, 1> p_I_L = q_L_I.conjugate() * (-p_L_I);

    Eigen::Quaternion<T> q_L0_I1 = q_L_I * q_I0_I1;
    Eigen::Matrix<T, 3, 1> p_L0_I1 = q_L_I * p_I0_I1 + p_L_I;
    Eigen::Matrix<T, 3, 1> p_L0_L1 = q_L0_I1 * p_I_L + p_L0_I1;

    return p_L0_L1;
  }

  template<typename TrajectoryModel>
  Vector3 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Measure<TrajectoryModel, double>(*pose_, trajectory);
  };

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Error(const type::Pose<PoseModel, T> &pose,
                               const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    Eigen::Matrix<T, 3, 1> p_M_L = p_.cast<T>();
    Eigen::Matrix<T, 3, 1> W = weight.cast<T>();
    return W.cwiseProduct(p_M_L - Measure<TrajectoryModel, T>(pose, trajectory));
  }

  template<typename TrajectoryModel>
  Eigen::Matrix<double, 3, 1> Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Error<TrajectoryModel, double>(*pose_, trajectory);
  }

  // Measurement data
  std::shared_ptr<PoseModel> pose_;
  double t0;
  double t1;
  Vector3 weight;
  Vector3 p_;

 protected:
  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const RelativePosePositionMeasurement<PoseModel> &m) : measurement(m) {}

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

    const RelativePosePositionMeasurement& measurement;
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
    cost_function->SetNumResiduals(3);
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

#endif  //KONTIKI_RELATIVE_POSE_POSITION_MEASUREMENT_H
