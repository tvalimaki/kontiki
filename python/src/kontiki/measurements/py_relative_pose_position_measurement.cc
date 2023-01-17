#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include <kontiki/measurements/relative_pose_position_measurement.h>

#include "measurement_helper.h"
#include "../pose_defs.h"

namespace py = pybind11;

PYBIND11_MODULE(_relative_pose_position_measurement, m) {
  m.doc() = "Relative pose position measurement";

  hana::for_each(pose_types, [&](auto pose_type) {
    using PoseModel = typename decltype(pose_type)::type;

    using Class = kontiki::measurements::RelativePosePositionMeasurement<PoseModel>;
    std::string pyclass_name = "RelativePosePositionMeasurement_" + std::string(PoseModel::CLASS_ID);
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());
    cls.def(py::init<std::shared_ptr<PoseModel>, double, double, const Eigen::Vector3d &>());
    cls.def_readonly("pose", &Class::pose_);
    cls.def_readonly("t0", &Class::t0);
    cls.def_readonly("t1", &Class::t1);
    cls.def_readonly("p", &Class::p_);

    declare_measurement_common<Class>(cls);
  });  // for_each(pose_types)
}
