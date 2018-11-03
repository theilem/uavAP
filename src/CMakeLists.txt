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
cmake_minimum_required (VERSION 2.8.12.2)
project (uavAP)

#######Build configuration########

set(LIBRARY_ROOT_DIR "/" CACHE FILEPATH "Library root for eigen, boost, protobuf")

#Set Variable inputs
set(EIGEN_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/eigen3/")
set(Boost_LIBRARY_DIRS "${LIBRARY_ROOT_DIR}/usr/local/lib/")
set(Boost_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/")
set(Protobuf_LIBRARY_DIRS "${LIBRARY_ROOT_DIR}/usr/local/lib/")
set(Protobuf_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/")

if(CMAKE_COMPILER_IS_GNUCC)
	if(CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.9.9)
		message(WARNING "gcc version too new, setting gcc-5 as C compiler")
		set(CMAKE_C_COMPILER /usr/bin/gcc-5)
	endif(CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.9.9)

	if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.9.9)
		message(WARNING "g++ version too new, setting g++-5 as CXX compiler")
		set(CMAKE_CXX_COMPILER /usr/bin/g++-5)
	endif(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.9.9)

endif(CMAKE_COMPILER_IS_GNUCC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -std=c++11 -fPIC -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall")

message(INFO ${LIBRARY_ROOT_DIR})

include_directories(	${CMAKE_SOURCE_DIR}
						${Boost_INCLUDE_DIRS}
						${EIGEN_INCLUDE_DIRS}
						${Protobuf_INCLUDE_DIRS})
			

link_directories(	${Boost_LIBRARY_DIRS}
					${Protobuf_LIBRARY_DIRS}
					)
					
if(LINK_STATIC)
	set(BOOST 	libboost_filesystem.a
				libboost_thread.a
				libboost_system.a
				libboost_date_time.a
				libboost_chrono.a
				)
	set(PROTOBUF libprotobuf.a)
	set(ARB 	libarb.a
				libmpfr.a
				libgmp.a
				libflint.a)
else(LINK_STATIC)
	set(BOOST 	boost_filesystem
				boost_thread
				boost_system
				boost_date_time
				boost_chrono
				)
	set(PROTOBUF protobuf)
	set(ARB 	arb
				mpfr
				gmp
				flint)
endif(LINK_STATIC)

if(NODEBUG)
	add_definitions(-DNODEBUG)
endif(NODEBUG)
		
add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY uavAP DESTINATION include) 
install(DIRECTORY config DESTINATION .) 
