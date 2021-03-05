add_executable(EmulationInterface	EmulationInterface.cpp
		EmulationDirectInterface.cpp
		)

target_link_libraries(EmulationInterface 	uavap_core
		cps_core
		ap_ext)

add_executable(EmulationInterfaceSync	EmulationInterfaceSync.cpp
		EmulationDirectInterface.cpp
		)

target_link_libraries(EmulationInterfaceSync 	uavap_core
		cps_core
		ap_ext)

install(TARGETS EmulationInterface EmulationInterfaceSync
		RUNTIME DESTINATION bin/uavAP)
