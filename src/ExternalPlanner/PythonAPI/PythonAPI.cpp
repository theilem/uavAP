//
// Created by mirco on 11.10.24.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>

#include "uavAP/Core/SensorData.h"
#include "uavAP/ExternalPlanner/PythonAPI/uavAPI.h"

namespace py = pybind11;

PYBIND11_MODULE(PythonAPI, m)
{
	m.doc() = "PythonAPI module";

	py::class_<SensorData>(m, "SensorData")
		.def(py::init<>())
		.def_readwrite("position", &SensorData::position)
		.def_readwrite("velocity", &SensorData::velocity)
		.def_readwrite("acceleration", &SensorData::acceleration)
		.def_readwrite("attitude", &SensorData::attitude)
		.def_readwrite("angular_rate", &SensorData::angularRate)
		.def_readwrite("airspeed", &SensorData::airSpeed)
		.def_readwrite("timestamp", &SensorData::timestamp);

	py::class_<QuarticSpline>(m, "QuarticSpline")
	        .def(py::init<Vector3, Vector3, Vector3, Vector3, Vector3, FloatingType>());

	py::class_<uavAPI>(m, "uavAPI")
	        .def(py::init<>())
			.def("getSensorData", &uavAPI::getSensorData)
			.def("setSplineSegment", &uavAPI::setSplineSegment);
}

