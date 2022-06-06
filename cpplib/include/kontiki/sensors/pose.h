//
// Created by tuomas on 2022-06-03.
//

#ifndef KONTIKI_POSE_H
#define KONTIKI_POSE_H

#include <memory>

#include "sensors.h"
#include <entity/paramstore/dynamic_pstore.h>

namespace kontiki {
namespace sensors {
namespace internal {

struct PoseMeta : public SensorMeta {
};

template<typename T>
struct PoseEvaluation {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
};

template<typename T, typename MetaType>
class PoseView : public SensorView<T, MetaType> {
 protected:
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<PoseEvaluation<T>>;
 public:
  using SensorView<T, MetaType>::SensorView;
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class PoseEntity : public SensorEntity<ViewTemplate, MetaType, StoreType> {
  using Vector3 = Eigen::Vector3d;
 public:
  // Inherit constructors
  using SensorEntity<ViewTemplate, MetaType, StoreType>::SensorEntity;
};


}  // namespace internal

class Pose : public internal::PoseEntity<internal::PoseView,
                                         internal::PoseMeta,
                                         entity::DynamicParameterStore<double>> {
 public:
  static constexpr const char* CLASS_ID = "Pose";
};

}  // namespace sensors

namespace type {
template<typename _Entity, typename T>
using Pose = typename entity::type::base::ForView<_Entity, sensors::internal::PoseView, T>;
} // namespace type

}  // namespace kontiki
#endif //KONTIKI_POSE_H
