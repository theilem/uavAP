#!/bin/bash

CORES=$(grep -c ^processor /proc/cpuinfo)

SETUP_ROOT_DIR=$(pwd)/..
    
#Check System and install packages
if [ `cat /proc/version | grep -c "Ubuntu"` -gt 0 ]; then
	sudo apt-get --assume-yes install gcc-multilib autoconf g++-multilib cmake autogen libtool curl
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

#Get Boost
wget "https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz"
printf "\nExtracting boost\n"
tar -xzf boost_1_64_0.tar.gz
cd boost_1_64_0
./bootstrap.sh --with-libraries=system,test,thread,filesystem,date_time,chrono
sudo ./b2 -scxxflags=-fPIC -scflags=-fPIC -j$CORES install
cd ${SETUP_ROOT_DIR}/tmp

#Get Protobuf
git clone "https://github.com/google/protobuf.git" protobuf_install
cd protobuf_install
./autogen.sh
./configure CFLAGS="-fPIC" CXXFLAGS="-fPIC"
make -j$CORES
make check -j$CORES
sudo make install -j$CORES
cd ${SETUP_ROOT_DIR}/tmp

#Get Eigen headers
wget "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz"
printf "\nExtracting eigen\n"
tar -xzf 3.3.4.tar.gz 
mkdir eigen-build
cd eigen-build
cmake ../eigen-eigen-5a0156e40feb
sudo make install -j$CORES

cd ${SETUP_ROOT_DIR}
sudo rm -rf tmp


#Create bld paths
if [ -d "bld" ]; then
	rm -rf bld
fi

mkdir bld
cd bld
mkdir Release
mkdir Debug

cd Release 
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../../src/

cd ../Debug
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ../../src/

printf "\nSetup completed\n\n"


