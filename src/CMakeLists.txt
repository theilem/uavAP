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
add_subdirectory(FlightControl)
add_subdirectory(FlightAnalysis)
add_subdirectory(Core)
add_subdirectory(MissionControl) 
add_subdirectory(Communication) 
add_subdirectory(API) 
add_subdirectory(Watchdog)
if (WITH_TOOLS)
    add_subdirectory(Tools)
endif (WITH_TOOLS)
add_subdirectory(ExternalPlanner)
add_subdirectory(Simulation)

