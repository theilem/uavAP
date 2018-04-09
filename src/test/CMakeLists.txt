#-------------------------------------------------------------------------------
# Copyright (C) 2018 University of Illinois Board of Trustees
# 
# This file is part of uavAP.
# 
# uavAP is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# uavAP is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#-------------------------------------------------------------------------------
set (TEST_SOURCES 	TestRunner.cpp
			Core/APLoggerTest.cpp
			Core/RunnerTest.cpp
                        Core/DataPresentationTest.cpp
                        Core/IPCTest.cpp
                        Core/SchedulerTest.cpp
			#FlightControl/Controller/PIDControllerTest_Airplane.cpp
			#FlightControl/Controller/PIDControllerTest_Helicopter.cpp
			)

set (TEST_DEPENDENCIES 	boost_system
        		boost_unit_test_framework.a
        		boost_serialization
        		boost_date_time
        		pthread
        		rt
        		${AP_FLIGHT_CONTROL}
        		${AP_CORE}
        		)

add_executable (tests EXCLUDE_FROM_ALL ${TEST_SOURCES})
target_link_libraries (tests ${TEST_DEPENDENCIES})

configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/P_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/P_Airplane.json COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/PD_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PD_Airplane.json COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/PFF_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PFF_Airplane.json COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/PI_Airplane.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PI_Airplane.json COPYONLY)

configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/P_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/P_Helicopter.json COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/PD_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PD_Helicopter.json COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/PFF_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PFF_Helicopter.json COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/test/FlightControl/Controller/config/PI_Helicopter.json ${CMAKE_BINARY_DIR}/test/FlightControl/Controller/config/PI_Helicopter.json COPYONLY)


#Real Time tests
set (RT_TEST_SOURCES    TestRunner.cpp
                        Core/SchedulerRTTest.cpp
                        )
                        
add_executable (rttests EXCLUDE_FROM_ALL ${RT_TEST_SOURCES})
target_link_libraries (rttests ${TEST_DEPENDENCIES})

add_subdirectory(Messages)
