cmake_minimum_required (VERSION 3.10)
project (uavAP)

#######Build configuration########

option(NO_DL "Deactivate dynamic linking in the Plugin Manager." OFF)

if(NO_DL)
	add_definitions(-DNO_DL)
endif(NO_DL)

#Set Variable inputs
set(EIGEN_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/eigen3/"
		"${LIBRARY_ROOT_DIR}/usr/include/eigen3/")
set(Boost_LIBRARY_DIRS "${LIBRARY_ROOT_DIR}/usr/local/lib/"
		"${LIBRARY_ROOT_DIR}/usr/lib/")
set(Boost_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/"
		"${LIBRARY_ROOT_DIR}/usr/include/")

if(CMAKE_COMPILER_IS_GNUCC)

	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0.0)
		message(STATUS "g++ version older than 6.0.0. Cannot use c++17 features (e.g. shared_mutex), setting c++11 as CXX compiler flag")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	else(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0.0)
		message(STATUS "Using c++17")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")
	endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0.0)
else(CMAKE_COMPILER_IS_GNUCC)
		message(STATUS "Using c++17")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")
endif(CMAKE_COMPILER_IS_GNUCC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -fPIC -Wall -Wno-int-in-bool-context -Wno-parentheses")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall")

include_directories(	${CMAKE_SOURCE_DIR}/include
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

set(REDIS 	cpp_redis
			tacopie
			)

if(NODEBUG)
	add_definitions(-DNODEBUG)
endif(NODEBUG)

set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)
		
add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY include/uavAP DESTINATION include)
install(DIRECTORY config DESTINATION .) 
