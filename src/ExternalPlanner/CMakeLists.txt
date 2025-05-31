if (PYTHON)
    add_subdirectory(PythonAPI)
endif (PYTHON)

set(EXT_PLANNER_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/ExtConnector.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_executable(ExtPlanner ${EXT_PLANNER_SRC}
)

target_link_libraries(ExtPlanner uavap_core cps_core uavap_flight_control
)

install(TARGETS ExtPlanner
        RUNTIME DESTINATION bin/uavAP
        LIBRARY DESTINATION lib)
