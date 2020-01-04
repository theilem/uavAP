#!/bin/bash

CORES=$(grep -c ^processor /proc/cpuinfo)
ECLIPSE=4.7.2

SETUP_ROOT_DIR=$(pwd)
    
#Check System and install packages
if [ `cat /proc/version | grep -c "Ubuntu"` -gt 0 ]; then
	sudo apt-get --assume-yes install gcc-multilib autoconf g++-multilib cmake autogen libtool curl lzip
elif [ `cat /proc/version | grep -c "ARCH"` -gt 0 ]; then
	if [ ! `sudo pacman -Qs gcc-multilib | grep -c "gcc-multilib"` -gt 0 ]; then
	    	sudo pacman -S --noconfirm gcc-multilib
	fi

	if [ ! `sudo pacman -Qs cmake | grep -c "cmake"` -gt 0 ]; then
	    	sudo pacman -S --noconfirm cmake
	fi
else
	printf "\nSetup: Unsupported Operating System\n\n"
	exit
fi

if [ -d "tmp" ]; then
	read -p "tmp folder found. override? [y/n] " override
	if [ "$override" != "y" ]; then
		printf "\nExit setup\n"
		exit
	else
		printf "\nRemoving tmp folder\n"
		rm -rf tmp
	fi
fi

mkdir tmp
cd $SETUP_ROOT_DIR/tmp

#Get Boost 1.68.0
wget "https://dl.bintray.com/boostorg/release/1.65.0/source/boost_1_65_0.tar.gz"
printf "\nExtracting boost 1.65.0\n"
tar -xzf boost_1_65_0.tar.gz
cd boost_1_65_0
./bootstrap.sh --with-libraries=system,filesystem
sudo ./b2 -scxxflags=-fPIC -scflags=-fPIC -j$CORES install
cd ${SETUP_ROOT_DIR}/tmp

#Get Boost 1.65.1
if [ `cat /proc/version | grep -c "ARCH"` -gt 0 ]; then
	wget "https://dl.bintray.com/boostorg/release/1.65.1/source/boost_1_65_1.tar.gz"
	printf "\nExtracting boost 1.65.1\n"
	tar -xzf boost_1_65_1.tar.gz
	cd boost_1_65_1
	./bootstrap.sh --with-libraries=system,filesystem
	sudo ./b2 -scxxflags=-fPIC -scflags=-fPIC -j$CORES install
	cd ${SETUP_ROOT_DIR}/tmp
fi

#Get Eigen headers
wget "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz"
printf "\nExtracting eigen\n"
tar -xzf 3.3.4.tar.gz 
mkdir eigen-build
cd eigen-build
cmake ../eigen-eigen-5a0156e40feb
sudo make install -j$CORES
cd ${SETUP_ROOT_DIR}/tmp

#Redis
git clone --recurse-submodules "https://github.com/cpp-redis/cpp_redis.git"
cd cpp_redis
mkdir bld
cd bld
cmake ../
make -j$CORES
sudo make install -j$CORES
cd ${SETUP_ROOT_DIR}/tmp

#Remove tmp dir
cd ${SETUP_ROOT_DIR}
sudo rm -rf tmp

#Create bld paths
if [ -d "bld" ]; then
	rm -rf bld
fi

mkdir -p bld/core_only
cd bld/core_only

cmake -G "Eclipse CDT4 - Unix Makefiles" -DLIBRARY_ROOT_DIR="/" -DCMAKE_BUILD_TYPE=Release -DCMAKE_SKIP_INSTALL_ALL_DEPENDENCY=1 ../../src/

printf "\nSetup completed\n\n"


