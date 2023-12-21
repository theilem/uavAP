set(FLIGHT_ANALYSIS_SRC)

#add_subdirectory (StateAnalysis)
#add_subdirectory (ManeuverAnalysis)
#add_subdirectory (DataHandling)
#add_subdirectory (TrimAnalysis)
add_subdirectory (ManeuverPlanner)
#add_subdirectory (WindAnalysis)
add_subdirectory (Condition)
add_subdirectory (SignalGenerator)

add_library(uavap_flight_analysis SHARED ${FLIGHT_ANALYSIS_SRC})

target_link_libraries(uavap_flight_analysis uavap_core
		uavap_mission_control
		uavap_flight_control
											)

add_executable(FlightAnalysis	FlightAnalysis.cpp
								)

target_link_libraries(FlightAnalysis 	uavap_flight_analysis
										)

install(TARGETS FlightAnalysis uavap_flight_analysis
		RUNTIME DESTINATION bin/uavAP
		LIBRARY DESTINATION lib)
		