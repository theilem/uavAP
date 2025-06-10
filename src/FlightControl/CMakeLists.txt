set(FLIGHT_CONTROL_SRC)

add_subdirectory(Controller)
add_subdirectory(Safety)
add_subdirectory(LocalPlanner)
add_subdirectory(SensingActuationIO)
add_subdirectory(ThrottleLimiter)

add_library(uavap_flight_control ${FLIGHT_CONTROL_SRC})

target_link_libraries(uavap_flight_control uavap_control
        uavap_core
        ${CPS_CORE_LIB}
        uavap_mission_control
        ${BOOST}
        ${SYSTEM_LIBS}
        pthread
        )

add_executable(FlightControl FlightControl.cpp
        )

target_link_libraries(FlightControl uavap_flight_control cps_core
        )

install(TARGETS FlightControl uavap_flight_control
        RUNTIME DESTINATION bin/uavAP
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        )
