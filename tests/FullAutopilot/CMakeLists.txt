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

add_executable(autopilot EXCLUDE_FROM_ALL   Autopilot.cpp
                                )

target_link_libraries(autopilot	uavap_flight_analysis
									uavap_flight_control
									)

install(TARGETS autopilot  OPTIONAL
		RUNTIME DESTINATION bin/uavAP
		LIBRARY DESTINATION lib)
		
install(FILES autopilot.json OPTIONAL DESTINATION config) 

		
configure_file(${PROJECT_SOURCE_DIR}/tests/FullAutopilot/autopilot.json ${CMAKE_BINARY_DIR}/test/FullAutopilot/autopilot.json COPYONLY)