set(TEST_SRCS ${TEST_SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/Test.cpp
        )

add_subdirectory(API)
add_subdirectory(Core)
add_subdirectory(FlightAnalysis)
#add_subdirectory(FlightControl)
#add_subdirectory(Math)

include_directories(${SUBMODULE_DIR}/cpsCore/extern/Catch2/single_include
        ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(tests EXCLUDE_FROM_ALL ${TEST_SRCS})

target_link_libraries(tests ${CPS_CORE_LIBS}
        uavap_core
        uavap_api
        ap_ext
        )



configure_file(${PROJECT_SOURCE_DIR}/tests/API/config/ap_ext.json ${CMAKE_BINARY_DIR}/test/API/config/ap_ext.json COPYONLY)
