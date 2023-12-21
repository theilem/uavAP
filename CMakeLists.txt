cmake_minimum_required (VERSION 3.0.0)
project (uavAP)

#######Build configuration########

option(NO_DL "Deactivate dynamic linking in the Plugin Manager." OFF)

if(NO_DL)
	add_definitions(-DNO_DL)
endif(NO_DL)

if (DEPLOY_PATH)
	message(STATUS "Deploying to ${DEPLOY_PATH}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -static-libstdc++")
	link_directories(BEFORE	${DEPLOY_PATH}/lib/			)
	set(CMAKE_INSTALL_PREFIX ${DEPLOY_PATH})
	set(LINK_STATIC)
endif(DEPLOY_PATH)

#Set Variable inputs
#set(EIGEN_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/eigen3/"
#		"${LIBRARY_ROOT_DIR}/usr/include/eigen3/")
#set(Boost_LIBRARY_DIRS "${LIBRARY_ROOT_DIR}/usr/local/lib/"
#		"${LIBRARY_ROOT_DIR}/usr/lib/")
#set(Boost_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/"
#		"${LIBRARY_ROOT_DIR}/usr/include/")

include_directories(/usr/include/eigen3/)
#message(INFO ${Boost_INCLUDE_DIR})
#message(INFO ${Eigen3_DIR})

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

include_directories(	${CMAKE_SOURCE_DIR}/include)
			

link_directories(	${Boost_LIBRARY_DIRS}
					)
					
if(LINK_STATIC)
	set(ARB 	libarb.a
				libmpfr.a
				libgmp.a
				libflint.a)
else(LINK_STATIC)
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
set(SUBMODULE_DIR ${CMAKE_SOURCE_DIR}/extern)
		
add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY include/uavAP DESTINATION include)
install(DIRECTORY config DESTINATION .) 
