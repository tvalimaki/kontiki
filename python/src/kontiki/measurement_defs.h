//
// Created by hannes on 2018-02-05.
//

#ifndef KONTIKIV2_MEASUREMENT_DEFS_H
#define KONTIKIV2_MEASUREMENT_DEFS_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include <kontiki/measurements/static_rscamera_measurement.h>
#include <kontiki/measurements/lifting_rscamera_measurement.h>
#include <kontiki/measurements/newton_rscamera_measurement.h>
#include <kontiki/measurements/position_measurement.h>
#include <kontiki/measurements/orientation_measurement.h>
#include <kontiki/measurements/gyroscope_measurement.h>
#include <kontiki/measurements/accelerometer_measurement.h>
#include <kontiki/measurements/pose_position_measurement.h>
#include <kontiki/measurements/pose_orientation_measurement.h>

#include "camera_defs.h"
#include "imu_defs.h"
#include "pose_defs.h"

// Add template-of-template support to hana
// Thanks to Jason Rice!
template <template <template <typename> class> class F>
struct template_template_t
{
  template <template <typename> class G>
  constexpr auto operator()(hana::basic_type<hana::template_t<G>>) const
  -> hana::type<F<G>>
  { return {}; }
};

template <template <template <typename> class> class F>
constexpr auto template_template = template_template_t<F>{};

// Begin actual
namespace TM = kontiki::measurements;

// Define Camera measurements
static auto cam_meas_templates = hana::tuple_t<
    hana::template_t<TM::StaticRsCameraMeasurement>,
    hana::template_t<TM::LiftingRsCameraMeasurement>,
    hana::template_t<TM::NewtonRsCameraMeasurement>
    >;


static auto make_cam_meas = [](auto mtype, auto ctype) {
  using MeasType = typename decltype(mtype)::type;
  auto mclass = MeasType();
  return mclass(ctype);
};

static auto camera_measurements = hana::ap(
    hana::make_tuple(make_cam_meas),
    cam_meas_templates,
    camera_types
);

// Define IMU measurements
static auto imu_meas_templates = hana::tuple_t<
    hana::template_t<TM::GyroscopeMeasurement>,
    hana::template_t<TM::AccelerometerMeasurement>
>;


static auto make_imu_meas = [](auto mtype, auto imutype) {
  using MeasType = typename decltype(mtype)::type;
  auto mclass = MeasType();
  return mclass(imutype);
};

static auto imu_measurements = hana::ap(
    hana::make_tuple(make_imu_meas),
    imu_meas_templates,
    imu_types
);

// Define pose measurements
static auto pose_meas_templates = hana::tuple_t<
    hana::template_t<TM::PosePositionMeasurement>,
    hana::template_t<TM::PoseOrientationMeasurement>
>;


static auto make_pose_meas = [](auto mtype, auto ptype) {
  using MeasType = typename decltype(mtype)::type;
  auto mclass = MeasType();
  return mclass(ptype);
};

static auto pose_measurements = hana::ap(
    hana::make_tuple(make_pose_meas),
    pose_meas_templates,
    pose_types
);

// Final list of measurement types
static auto measurement_types = hana::concat(
    hana::concat(
        hana::concat(camera_measurements,
                     imu_measurements),
        pose_measurements),
    hana::tuple_t<
        TM::PositionMeasurement,
        TM::OrientationMeasurement>
);

#endif //KONTIKIV2_MEASUREMENT_DEFS_H
