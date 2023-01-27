#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include <kontiki/measurements/pose_position_measurement.h>

#include "measurement_helper.h"
#include "../pose_defs.h"

namespace py = pybind11;

PYBIND11_MODULE(_pose_position_measurement, m) {
  m.doc() = "Pose position measurement";

  hana::for_each(pose_types, [&](auto pose_type) {
    using PoseModel = typename decltype(pose_type)::type;

    using Class = kontiki::measurements::PosePositionMeasurement<PoseModel>;
    std::string pyclass_name = "PosePositionMeasurement_" + std::string(PoseModel::CLASS_ID);
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());
    cls.def(py::init<std::shared_ptr<PoseModel>, double, const Eigen::Vector3d &>());
    cls.def(py::init<std::shared_ptr<PoseModel>, double, const Eigen::Vector3d, double &>());
    cls.def_readonly("pose", &Class::pose_);
    cls.def_readonly("t", &Class::t);
    cls.def_readonly("p", &Class::p_);

    declare_measurement_common<Class>(cls);
  });  // for_each(pose_types)
}
