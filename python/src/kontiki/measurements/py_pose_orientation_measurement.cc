#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include <kontiki/measurements/pose_orientation_measurement.h>

#include "measurement_helper.h"
#include "../pose_defs.h"

namespace py = pybind11;

PYBIND11_MODULE(_pose_orientation_measurement, m) {
  m.doc() = "Pose orientation measurement";

  hana::for_each(pose_types, [&](auto pose_type) {
    using PoseModel = typename decltype(pose_type)::type;

    using Class = kontiki::measurements::PoseOrientationMeasurement<PoseModel>;
    std::string pyclass_name = "PoseOrientationMeasurement_" + std::string(PoseModel::CLASS_ID);
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());
    cls.def(py::init<std::shared_ptr<PoseModel>, double, const Eigen::Vector4d &>());
    cls.def(py::init<std::shared_ptr<PoseModel>, double, const Eigen::Vector4d, double &>());
    cls.def(py::init<std::shared_ptr<PoseModel>, double, const Eigen::Vector4d, double, double &>());
    cls.def_readonly("pose", &Class::pose_);
    cls.def_readonly("t", &Class::t);
    cls.def_property_readonly("q", [](Class &self) {
               Eigen::Vector4d pyq(self.q_.w(), self.q_.x(), self.q_.y(), self.q_.z());
               return pyq;
             });

    declare_measurement_common<Class>(cls);
  });  // for_each(pose_types)
}
