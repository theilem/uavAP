pybind11_add_module(PythonAPI PythonAPI.cpp
        uavAPI.cpp)

target_link_libraries(PythonAPI PRIVATE uavap_core
        ${CPS_CORE_LIB}
        uavap_flight_control
        ${BOOST}
        ${SYSTEM_LIBS}
        pthread)