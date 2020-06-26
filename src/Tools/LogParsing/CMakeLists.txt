add_executable(LogParser main.cpp
        LogParser.cpp
        )

target_link_libraries(LogParser uavap_core
        cps_core
        ap_ext)
