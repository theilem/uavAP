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
#add_executable(Simulation       Simulation/Simulation.cpp
#                                Simulation/Connector/SimulationConnector.cpp
#                                Simulation/Connector/Codec.cpp
#                                ChannelMixing.cpp
#                                )
#
#target_link_libraries(Simulation uavap_core)

add_library(ap_ext SHARED	ap_ext/ap_ext.cpp
							ap_ext/ApExtManager.cpp
							ap_ext/UTM.cpp
							ChannelMixing.cpp)

target_link_libraries(ap_ext	uavap_core
								uavap_api
								control
								cps_core)

add_library(uavap_api SHARED	AutopilotAPI.cpp
							AggregatableAutopilotAPI.cpp
							ap_ext/UTM.cpp
								)

target_link_libraries(uavap_api	uavap_core)

install(TARGETS ap_ext uavap_api
		RUNTIME DESTINATION bin/uavAP
		LIBRARY DESTINATION lib)


                                                
