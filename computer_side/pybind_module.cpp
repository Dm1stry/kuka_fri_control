#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "main_controller.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kuka_fri_py, m)
{
    m.doc() = "Python bindings for the KUKA FRI controller";

    py::enum_<KUKA_CONTROL::control_mode>(m, "ControlMode")
        .value("JOINT_POSITION", KUKA_CONTROL::JOINT_POSITION)
        .value("TORQUE", KUKA_CONTROL::TORQUE)
        .value("WRENCH", KUKA_CONTROL::WRENCH)
        .export_values();

    py::class_<KukaController>(m, "KukaController")
        .def(py::init<KUKA_CONTROL::control_mode, std::string, bool>(),
             py::arg("mode") = KUKA_CONTROL::JOINT_POSITION,
             py::arg("urdf_path") = "../robots/iiwa.urdf",
             py::arg("use_task_space") = false)
        .def("start", &KukaController::start, py::call_guard<py::gil_scoped_release>())
        .def("stop", &KukaController::stop, py::call_guard<py::gil_scoped_release>())
        .def("get_observation", &KukaController::getObservation)
        .def("set_target", &KukaController::setTarget,
             py::arg("target_position"),
             py::arg("target_rotation"),
             py::call_guard<py::gil_scoped_release>());
}
