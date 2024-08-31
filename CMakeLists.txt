cmake_minimum_required(VERSION 3.13)
project(uavAP)

#######Build configuration########

option(NO_DL "Deactivate dynamic linking in the Plugin Manager." OFF)
option(GS "Build the Ground Station." OFF)
option(XPLANE "Build the XPlane Plugin." OFF)
option(CORE "Always build cps_core." OFF)

if (NO_DL)
    add_definitions(-DNO_DL)
endif (NO_DL)

if (DEPLOY_PATH)
    message(STATUS "Deploying to ${DEPLOY_PATH}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -static-libstdc++")
    link_directories(BEFORE ${DEPLOY_PATH}/lib/)
    set(CMAKE_INSTALL_PREFIX ${DEPLOY_PATH})
    set(LINK_STATIC)
endif (DEPLOY_PATH)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})


find_library(CPS_CORE_LIBRARY cpsCore)
if (CPS_CORE_LIBRARY AND NOT CORE)
	set(CPS_CORE_LIB ${CPS_CORE_LIBRARY})
else(CPS_CORE_LIBRARY AND NOT CORE)
	set(CPS_CORE_LIB cps_core)
endif (CPS_CORE_LIBRARY AND NOT CORE)

#Set Variable inputs
#set(EIGEN_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/eigen3/"
#		"${LIBRARY_ROOT_DIR}/usr/include/eigen3/")
#set(Boost_LIBRARY_DIRS "${LIBRARY_ROOT_DIR}/usr/local/lib/"
#		"${LIBRARY_ROOT_DIR}/usr/lib/")
#set(Boost_INCLUDE_DIRS "${LIBRARY_ROOT_DIR}/usr/local/include/"
#		"${LIBRARY_ROOT_DIR}/usr/include/")

#message(INFO ${Boost_INCLUDE_DIR})
#message(INFO ${Eigen3_DIR})

#if (CMAKE_COMPILER_IS_GNUCC)
#
#    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0.0)
#        message(STATUS "g++ version older than 6.0.0. Cannot use c++17 features (e.g. shared_mutex), setting c++11 as CXX compiler flag")
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    else (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0.0)
#        message(STATUS "Using c++17")
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")
#    endif (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0.0)
#else (CMAKE_COMPILER_IS_GNUCC)
#    message(STATUS "Using c++17")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")
#endif (CMAKE_COMPILER_IS_GNUCC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z -static-libstdc++ -fPIC -Wall -Wno-int-in-bool-context -Wno-parentheses")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall")

include_directories(${CMAKE_SOURCE_DIR}/include)


link_directories(${Boost_LIBRARY_DIRS}
)

if (LINK_STATIC)
    set(ARB libarb.a
            libmpfr.a
            libgmp.a
            libflint.a)
else (LINK_STATIC)
    set(ARB arb
            mpfr
            gmp
            flint)
endif (LINK_STATIC)

set(REDIS cpp_redis
        tacopie
)

if (NODEBUG)
    add_definitions(-DNODEBUG)
endif (NODEBUG)

if (NOT APPLE)
    set(SYSTEM_LIBS ${SYSTEM_LIBS} rt)
    set(SYSTEM_LIBS_STATIC ${SYSTEM_LIBS_STATIC} rt.a)
endif(NOT APPLE)

set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)
set(SUBMODULE_DIR ${CMAKE_SOURCE_DIR}/extern)
include_directories(${SUBMODULE_DIR}/cpsCore/include)

add_subdirectory(${SUBMODULE_DIR}/cpsCore)
add_subdirectory(src)
add_subdirectory(test)

if (GS)
    add_subdirectory(${SUBMODULE_DIR}/uavGS)
endif (GS)

if (XPLANE)
    add_subdirectory(${SUBMODULE_DIR}/uavEE)
    include_directories(${SUBMODULE_DIR}/uavEE/include)
endif (XPLANE)

install(DIRECTORY include/uavAP DESTINATION include)
install(DIRECTORY config DESTINATION .) 
