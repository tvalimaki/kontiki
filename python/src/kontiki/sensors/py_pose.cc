#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "sensors_help.h"
#include "kontiki/sensors/pose.h"

namespace py = pybind11;

namespace C = kontiki::sensors;

PYBIND11_MODULE(_pose, m) {
  using Class = C::Pose;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "Pose");
  cls.doc() = R"pbdoc(Pose sensor
  )pbdoc";

  declare_sensors_common<Class>(cls);
}