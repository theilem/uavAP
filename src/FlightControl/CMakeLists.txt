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
set(FLIGHT_CONTROL_SRC)

add_subdirectory (Controller)
add_subdirectory (LocalPlanner)
add_subdirectory (SensingActuationIO)

add_library(uavap_flight_control SHARED ${FLIGHT_CONTROL_SRC})

target_link_libraries(uavap_flight_control 	control
											uavap_core
											uavap_mission_control
											${BOOST}
											rt
											pthread
											)

add_executable(FlightControl    FlightControl.cpp
                                )

target_link_libraries(FlightControl	uavap_flight_control
									)

install(TARGETS FlightControl uavap_flight_control OPTIONAL
		RUNTIME DESTINATION bin/uavAP
		LIBRARY DESTINATION lib )
