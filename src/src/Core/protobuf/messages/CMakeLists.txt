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
file(GLOB pb_src "*.pb.cc")
file(GLOB pb_header "*.pb.h")		


							
#set_target_properties(ap_pb_messages PROPERTIES PUBLIC_HEADER "${pb_header}")

#install(TARGETS ap_pb_messages
#		ARCHIVE DESTINATION ${UAVAP_PATH}
#		PUBLIC_HEADER DESTINATION ${UAVAP_PATH}/protobuf/messages)
