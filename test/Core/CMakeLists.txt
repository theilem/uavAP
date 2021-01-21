set(TEST_SRCS ${TEST_SRCS}
#        ${CMAKE_CURRENT_SOURCE_DIR}/DataHandlingTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ConversionUtilsTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EnumAccess.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FramedVector3Test.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/RectanguloidTest.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/FramesTest.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/OrientationTest.cpp
        PARENT_SCOPE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/datahandling1.json ${CMAKE_BINARY_DIR}/test/Core/config/datahandling1.json COPYONLY)