#Get Eigen headers
SOURCE_DIR=$(pwd)/src
INSTALL_DIR=/home/uav/Desktop/UAV_Libs
wget "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz"
printf "\nExtracting eigen\n"
tar -xzf 3.3.4.tar.gz 
mkdir eigen-build
cd eigen-build
cmake ../eigen-eigen-5a0156e40feb
make DESTDIR=/home/uav/Desktop/UAV_Libs install -j$CORES
cd /home/uav/Desktop/UAV_Libs

#Create bld paths
if [ -d "bld" ]; then
	rm -rf bld
fi

mkdir bld
cd bld
mkdir Release

#TODO Adapt arguments so that it automatically installs to the correct device
cd Release 
cmake -DCMAKE_BUILD_TYPE=Release -DLIBRARY_ROOT_DIR=/home/uav/Desktop/UAV_Libs -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ -DCMAKE_C_COMPILER=arm-linux-gnueabihf-gcc -DLINK_STATIC=1 -DCMAKE_INSTALL_PREFIX=/home/uav/Desktop/UAV_Libs/install $SOURCE_DIR

printf "\nSetup completed\n\n"


