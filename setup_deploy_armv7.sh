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
wget "https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz"
printf "\nExtracting boost\n"
tar -xzf boost_1_64_0.tar.gz
rm boost_1_64_0.tar.gz
cd boost_1_64_0
./bootstrap.sh --with-libraries=system,test,thread,filesystem,date_time,chrono --prefix=$INSTALL_DIR/usr/local
#change the used compiler
sed -i '/using gcc ;/c\using gcc : arm : arm-linux-gnueabi-g++ ;' project-config.jam
./bjam cxxflags="-fPIC" cflags="-fPIC" -j$CORES install
cd $INSTALL_DIR

#Get Protobuf
git clone "https://github.com/google/protobuf.git" protobuf_install
cd protobuf_install
./autogen.sh
./configure --host=arm-linux CC=arm-linux-gnueabi-gcc CXX=arm-linux-gnueabi-g++ CFLAGS="-fPIC" CXXFLAGS="-fPIC" --prefix=$INSTALL_DIR/usr/local
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

#Create bld paths
if [ -d "bld" ]; then
	rm -rf bld
fi

mkdir bld
cd bld
mkdir Release

#TODO Adapt arguments so that it automatically installs to the correct device
cd Release 
cmake -DCMAKE_BUILD_TYPE=Release -DLIBRARY_ROOT_DIR=$INSTALL_DIR -DCMAKE_CXX_COMPILER=arm-linux-gnueabi-g++ -DCMAKE_C_COMPILER=arm-linux-gnueabi-gcc -DCMAKE_CXX_FLAGS="-march=armv7-a -mtune=cortex-a9 -mfpu=neon" -DLINK_STATIC=1 -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/install $SOURCE_DIR

printf "\nSetup completed\n\n"


