set(TEST_SRCS ${TEST_SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/OverrideTest.cpp
        PARENT_SCOPE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/overrides.json
        ${CMAKE_BINARY_DIR}/test/FlightAnalysis/config/overrides.json COPYONLY)