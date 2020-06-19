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
set(FLIGHT_ANALYSIS_SRC)

add_subdirectory (StateAnalysis)
add_subdirectory (ManeuverAnalysis)
add_subdirectory (DataHandling)
add_subdirectory (TrimAnalysis)

add_library(uavap_flight_analysis SHARED ${FLIGHT_ANALYSIS_SRC})

target_link_libraries(uavap_flight_analysis uavap_core
											uavap_mission_control
											)

add_executable(FlightAnalysis	FlightAnalysis.cpp
								)

target_link_libraries(FlightAnalysis 	uavap_flight_analysis
										)

install(TARGETS FlightAnalysis uavap_flight_analysis
		RUNTIME DESTINATION bin/uavAP
		LIBRARY DESTINATION lib)
		