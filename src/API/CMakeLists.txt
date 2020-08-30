#add_executable(Simulation       Simulation/Simulation.cpp
#                                Simulation/Connector/SimulationConnector.cpp
#                                Simulation/Connector/Codec.cpp
#                                ChannelMixing.cpp
#                                )
#
#target_link_libraries(Simulation uavap_core)

add_library(ap_ext SHARED
        ap_ext/ap_ext.cpp
        ap_ext/ApExtManager.cpp
        ap_ext/LinearSensorManager.cpp
        ap_ext/UTM.cpp
        ChannelMixing.cpp)

target_link_libraries(ap_ext uavap_core
        uavap_api
        control
        cps_core)

add_library(uavap_api SHARED AutopilotAPI.cpp
        AggregatableAutopilotAPI.cpp
        ap_ext/UTM.cpp
        )

target_link_libraries(uavap_api uavap_core cps_core)

install(TARGETS ap_ext uavap_api
        RUNTIME DESTINATION bin/uavAP
        LIBRARY DESTINATION lib)


                                                
