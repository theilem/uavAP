#!/bin/bash

#Get Eigen headers
CORES=$(grep -c ^processor /proc/cpuinfo)

read -p "Provide build and lib directory to be setup: " INSTALL_DIR
    
echo $INSTALL_DIR

SOURCE_DIR=$(pwd)/src

if [ -d "$INSTALL_DIR" ]; then
	rm -rf $INSTALL_DIR
fi
mkdir -p $INSTALL_DIR

cd $INSTALL_DIR

#Get Boost
wget "https://dl.bintray.com/boostorg/release/1.65.0/source/boost_1_65_0.tar.gz"
printf "\nExtracting boost\n"
tar -xzf boost_1_65_0.tar.gz
rm boost_1_65_0.tar.gz
cd boost_1_65_0
./bootstrap.sh --with-libraries=system,test,thread,filesystem,date_time,chrono --prefix=$INSTALL_DIR/usr/local
#change the used compiler
sed -i '/using gcc ;/c\using gcc : aarch : aarch64-linux-gnu-g++-7 ;' project-config.jam
./bjam cxxflags="-fPIC" cflags="-fPIC" -j$CORES install
cd $INSTALL_DIR

#Get Protobuf
wget "https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-all-3.6.1.tar.gz"
printf "\nExtracting protobuf\n"
tar -xzf protobuf-all-3.6.1.tar.gz
rm protobuf-all-3.6.1.tar.gz
cd protobuf-3.6.1
./autogen.sh
./configure --host=arm-linux CC=aarch64-linux-gnu-gcc-7 CXX=aarch64-linux-gnu-g++-7 CFLAGS="-fPIC" CXXFLAGS="-fPIC" --prefix=$INSTALL_DIR/usr/local
make -j$CORES
make check -j$CORES
make install -j$CORES
cd $INSTALL_DIR

#Get Eigen headers
wget "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz"
printf "\nExtracting eigen\n"
tar -xzf 3.3.4.tar.gz 
mkdir eigen-build
cd eigen-build
cmake ../eigen-eigen-5a0156e40feb
make DESTDIR=$INSTALL_DIR install -j$CORES
cd $INSTALL_DIR

#Redis
git clone "https://github.com/cpp-redis/cpp_redis.git"
cd cpp_redis
mkdir bld
cd bld
cmake -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++-7 -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc-7 -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/usr/local ../
make -j$CORES
make install -j$CORES
cd $INSTALL_DIR

git clone "https://github.com/Cylix/tacopie.git"
cd tacopie
mkdir bld
cd bld
cmake -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++-7 -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc-7 -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/usr/local ../
make -j$CORES
make install -j$CORES
cd $INSTALL_DIR

#Create bld paths
if [ -d "bld" ]; then
	rm -rf bld
fi

mkdir bld
cd bld
mkdir Release

#TODO Adapt arguments so that it automatically installs to the correct device
cd Release 
cmake -DCMAKE_BUILD_TYPE=Release -DLIBRARY_ROOT_DIR=$INSTALL_DIR -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++-7 -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc-7 -DLINK_STATIC=1 -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/install $SOURCE_DIR

printf "\nSetup completed\n\n"


