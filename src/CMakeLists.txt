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

#set(LIBRARY_ROOT_DIR "/" CACHE FILEPATH "Library root for eigen, boost, protobuf")

option(NO_DL "Deactivate dynamic linking in the Plugin Manager." OFF)

if(NO_DL)
	add_definitions(-DNO_DL)
endif(NO_DL)

#Set Variable inputs
set(EIGEN_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/eigen3/")
set(Boost_LIBRARY_DIRS "${LIBRARY_ROOT_DIR}/usr/local/lib/")
set(Boost_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/")

if(CMAKE_COMPILER_IS_GNUCC)

	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0.0)
		message(STATUS "g++ version older than 5.0.0. Cannot use c++14 features (e.g. shared_mutex), setting c++11 as CXX compiler flag")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	else(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0.0)
		message(STATUS "Using c++14")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
	endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0.0)
else(CMAKE_COMPILER_IS_GNUCC)
		message(STATUS "Using c++14")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif(CMAKE_COMPILER_IS_GNUCC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -fPIC -Wall -Wno-int-in-bool-context -Wno-parentheses")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall")

include_directories(	${CMAKE_SOURCE_DIR}
						${Boost_INCLUDE_DIRS}
						${EIGEN_INCLUDE_DIRS})
			

link_directories(	${Boost_LIBRARY_DIRS}
					)
					
if(LINK_STATIC)
	set(BOOST 	libboost_filesystem.a
				libboost_system.a
				)
	set(ARB 	libarb.a
				libmpfr.a
				libgmp.a
				libflint.a)
else(LINK_STATIC)
	set(BOOST 	boost_filesystem
				boost_system
				)
	set(ARB 	arb
				mpfr
				gmp
				flint)
endif(LINK_STATIC)

set(REDIS 	libcpp_redis.a
			tacopie
			)

if(NODEBUG)
	add_definitions(-DNODEBUG)
endif(NODEBUG)

set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)
		
add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY uavAP DESTINATION include) 
install(DIRECTORY config DESTINATION .) 
