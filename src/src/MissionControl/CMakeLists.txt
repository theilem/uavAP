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
set(MISSION_CONTROL_SRC "")
set(MISSION_CONTROL_GEO_SRC "")

add_subdirectory(MissionPlanner)
add_subdirectory(GlobalPlanner)
add_subdirectory(DataHandling)
add_subdirectory(ManeuverPlanner)
add_subdirectory(ConditionManager)
add_subdirectory(LocalFrameManager)
add_subdirectory(Geofencing)

set(MISSION_CONTROL_SRC ${MISSION_CONTROL_SRC}
						MissionControlPrint.cpp
						Polygon.cpp)

add_library(uavap_mission_control SHARED ${MISSION_CONTROL_SRC})

target_link_libraries(uavap_mission_control 	uavap_core
												uavap_core_proto)

add_library(uavap_mission_control_geo SHARED EXCLUDE_FROM_ALL ${MISSION_CONTROL_GEO_SRC})

target_link_libraries(uavap_mission_control_geo 	uavap_core
												uavap_core_proto
												${ARB})

add_executable(MissionControl   MissionControl.cpp
								DataHandling/MissionControlDataHandling.cpp)

target_link_libraries(MissionControl    uavap_flight_analysis
										uavap_mission_control
										uavap_mission_control_geo
                                        )
                                        
install(TARGETS uavap_mission_control 
		RUNTIME DESTINATION bin/uavAP
		LIBRARY DESTINATION lib)
		
install(TARGETS MissionControl uavap_mission_control uavap_mission_control_geo
		RUNTIME DESTINATION bin/uavAP
		LIBRARY DESTINATION lib)
