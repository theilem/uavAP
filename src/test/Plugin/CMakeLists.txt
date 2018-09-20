set(TEST_SOURCES 	${TEST_SOURCES}
					${CMAKE_CURRENT_SOURCE_DIR}/PluginTest.cpp
					PARENT_SCOPE)


add_library(test_plugin SHARED TestControllerPlugin.cpp)

target_link_libraries(test_plugin 	uavap_core
									uavap_flight_control)
