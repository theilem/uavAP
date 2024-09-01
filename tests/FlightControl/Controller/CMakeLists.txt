#NOTE - bottom line does not work because apparently PARENT_SCOPE here is not seen in global due to nesting

#set(TEST_SRCS ${TEST_SRCS}
#        ${CMAKE_CURRENT_SOURCE_DIR}/PIDControllerTest_Airplane.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/PIDControllerTest_Helicopter.cpp
#        PARENT_SCOPE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/P_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/P_Airplane.json COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/P_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/P_Helicopter.json COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/PD_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PD_Airplane.json COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/PD_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PD_Helicopter.json COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/PFF_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PFF_Airplane.json COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/PFF_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PFF_Helicopter.json COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/PI_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PI_Airplane.json COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/PI_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PI_Helicopter.json COPYONLY)
