set(TEST_SRCS ${TEST_SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/DataHandlingTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EnumAccess.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FramesTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RectanguloidTest.cpp
        PARENT_SCOPE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/datahandling1.json ${CMAKE_BINARY_DIR}/tests/Core/config/datahandling1.json COPYONLY)