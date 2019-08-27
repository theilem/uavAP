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
./bootstrap.sh --with-libraries=system,test,thread,filesystem,date_time,chrono
sudo ./b2 -scxxflags=-fPIC -scflags=-fPIC -j$CORES install
cd ${SETUP_ROOT_DIR}/tmp

#Get Boost 1.65.1
if [ `cat /proc/version | grep -c "ARCH"` -gt 0 ]; then
	wget "https://dl.bintray.com/boostorg/release/1.65.1/source/boost_1_65_1.tar.gz"
	printf "\nExtracting boost 1.65.1\n"
	tar -xzf boost_1_65_1.tar.gz
	cd boost_1_65_1
	./bootstrap.sh
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

#ARB LIB Dependencies
#GMP
wget "https://gmplib.org/download/gmp/gmp-6.1.2.tar.lz"
tar -xf gmp-6.1.2.tar.lz
cd gmp-6.1.2
./configure
make -j$CORES
sudo make install -j$CORES
cd ${SETUP_ROOT_DIR}/tmp

#MPFR
wget "https://www.mpfr.org/mpfr-current/mpfr-4.0.2.tar.xz"
tar -xf mpfr-4.0.2.tar.xz
cd mpfr-4.0.2
./configure
make -j$CORES
sudo make install -j$CORES
cd ${SETUP_ROOT_DIR}/tmp

#Flint2
git clone "https://github.com/fredrik-johansson/flint2"
cd flint2
./configure
make -j$CORES
sudo make install -j$CORES
cd ${SETUP_ROOT_DIR}/tmp

#Arb
git clone "https://github.com/fredrik-johansson/arb.git"
cd arb
./configure
make -j$CORES
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

printf "\nGenerating protobuf files\n"
./generate_proto.sh

#Create bld paths
if [ -d "bld" ]; then
	rm -rf bld
fi

mkdir bld
cd bld
mkdir Release
mkdir Debug

cd Release 
cmake -G "Eclipse CDT4 - Unix Makefiles" -DLIBRARY_ROOT_DIR="/" -DCMAKE_BUILD_TYPE=Release -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE ../../src/

cd ../Debug
cmake -G "Eclipse CDT4 - Unix Makefiles" -DLIBRARY_ROOT_DIR="/" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE ../../src/

printf "\nSetup completed\n\n"


