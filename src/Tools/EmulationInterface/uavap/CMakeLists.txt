add_executable(EmulationInterface	EmulationInterface.cpp
									EmulationDirectInterface.cpp
									)
									
target_link_libraries(EmulationInterface 	uavap_core
											cps_core
											ap_ext)

install(TARGETS EmulationInterface 
		RUNTIME DESTINATION bin/uavAP)
