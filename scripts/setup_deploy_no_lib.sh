#!/bin/bash

CORES=$(grep -c ^processor /proc/cpuinfo)

read -p "Provide build and lib directory to be setup: " INSTALL_DIR
    
echo $INSTALL_DIR

SOURCE_DIR=$(pwd)/src

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
cmake -DCMAKE_BUILD_TYPE=Release -DLIBRARY_ROOT_DIR=$INSTALL_DIR -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32 -DLINK_STATIC=1 -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/install $SOURCE_DIR

printf "\nSetup completed\n\n"


