set(TEST_SRCS ${TEST_SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ApExtTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/LinearSensorManagerTest.cpp
        PARENT_SCOPE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/ap_ext.json ${CMAKE_BINARY_DIR}/tests/API/config/ap_ext.json COPYONLY)