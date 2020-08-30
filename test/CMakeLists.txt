set(TEST_SRCS Test.cpp
        API/ApExtTest.cpp
        API/LinearSensorManagerTest.cpp
        Core/EnumAccess.cpp
        )

include_directories(${SUBMODULE_DIR}/Catch2/single_include
        ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(tests EXCLUDE_FROM_ALL ${TEST_SRCS})

target_link_libraries(tests cps_core
        uavap_core
        uavap_api
        ap_ext
        )



configure_file(${PROJECT_SOURCE_DIR}/test/API/config/ap_ext.json ${CMAKE_BINARY_DIR}/test/API/config/ap_ext.json COPYONLY)
