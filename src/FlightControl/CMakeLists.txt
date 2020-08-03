set(FLIGHT_CONTROL_SRC)

add_subdirectory(Controller)
add_subdirectory(LocalPlanner)
add_subdirectory(SensingActuationIO)
add_subdirectory(ThrottleLimiter)

add_library(uavap_flight_control SHARED ${FLIGHT_CONTROL_SRC})

target_link_libraries(uavap_flight_control control
        uavap_core
        cps_core
        uavap_mission_control
        ${BOOST}
        rt
        pthread
        )

add_executable(FlightControl FlightControl.cpp
        )

target_link_libraries(FlightControl uavap_flight_control cps_core
        )

install(TARGETS FlightControl uavap_flight_control
        RUNTIME DESTINATION bin/uavAP
        LIBRARY DESTINATION lib)
