#!/bin/bash

# Declare variables
CORES=$(grep -c ^processor /proc/cpuinfo)
DEPLOY_DIR=$(pwd)/bld/Deploy
SOURCE_DIR=$(pwd)/src

# Create deployment folder
printf "\nCreating deployment folder...\n\n"
if [ -d $DEPLOY_DIR ]; then
	printf "\nDeployment Folder Exists. Quit.\n\n"
	exit
fi
mkdir -p $DEPLOY_DIR
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Install Boost
cd $DEPLOY_DIR
printf "\nInstalling Boost...\n\n"
wget "https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz"
tar -xzf boost_1_64_0.tar.gz
rm boost_1_64_0.tar.gz
cd boost_1_64_0
./bootstrap.sh --with-libraries=system,test,thread,filesystem,date_time,chrono --prefix=$DEPLOY_DIR/usr/local
./bjam cxxflags="-fPIC -m32" cflags="-fPIC -m32" address-model=32 -j$CORES install
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Install Eigen
cd $DEPLOY_DIR
printf "\nInstalling Eigen...\n\n"
wget "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz"
printf "\nExtracting eigen\n"
tar -xzf 3.3.4.tar.gz 
mkdir eigen-build
cd eigen-build
cmake ../eigen-eigen-5a0156e40feb
make DESTDIR=$DEPLOY_DIR install -j$CORES
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Install GMP
cd $DEPLOY_DIR
printf "\nInstalling GMP...\n\n"
wget "https://gmplib.org/download/gmp/gmp-6.1.2.tar.xz"
tar -xf gmp-6.1.2.tar.xz
cd gmp-6.1.2
./configure CFLAGS="-fPIC -m32" CXXFLAGS="-fPIC -m32" ABI=32 --disable-shared --prefix=$DEPLOY_DIR/usr/local/
make -j$CORES
make install -j$CORES
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Install MPFR
cd $DEPLOY_DIR
printf "\nInstalling MPFR...\n\n"
wget "https://www.mpfr.org/mpfr-4.0.1/mpfr-4.0.1.tar.xz"
tar -xf mpfr-4.0.1.tar.xz
cd mpfr-4.0.1
./configure CFLAGS="-fPIC -m32" CXXFLAGS="-fPIC -m32" --with-gmp=$DEPLOY_DIR/usr/local/ --prefix=$DEPLOY_DIR/usr/local/
make -j$CORES
make install -j$CORES
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Install Flint
cd $DEPLOY_DIR
printf "\nInstalling Flint...\n\n"
wget "http://www.flintlib.org/flint-2.5.2.tar.gz"
tar -xf flint-2.5.2.tar.gz
cd flint-2.5.2
./configure CFLAGS="-fPIC -m32" CXXFLAGS="-fPIC -m32" --with-gmp=$DEPLOY_DIR/usr/local/ --with-mpfr=$DEPLOY_DIR/usr/local/ --prefix=$DEPLOY_DIR/usr/local/ ABI=32
sed -i "s/-Wl,//g" Makefile.subdirs
make -j$CORES
make install -j$CORES
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Install Arb
cd $DEPLOY_DIR
printf "\nInstalling Arb...\n\n"
git clone https://github.com/fredrik-johansson/arb.git
cd arb
git reset --hard 1465e566ddfa034dd64ad08c24db14f61ef24442
./configure CFLAGS="-fPIC -m32" --with-gmp=$DEPLOY_DIR/usr/local/ --with-mpfr=$DEPLOY_DIR/usr/local/ --prefix=$DEPLOY_DIR/usr/local/ ABI=32
make -j$CORES
make install -j$CORES
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Install Redis
cd $DEPLOY_DIR
printf "\nInstalling Redis...\n\n"
git clone --recurse-submodules "https://github.com/cpp-redis/cpp_redis.git"
cd cpp_redis
mkdir bld
cd bld
cmake -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_INSTALL_PREFIX=$DEPLOY_DIR/usr/local/ ../
make -j$CORES
make install -j$CORES
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi
cd ../tacopie
mkdir bld
cd bld
cmake -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_INSTALL_PREFIX=$DEPLOY_DIR/usr/local/ ../
make -j$CORES
make install -j$CORES
cd $DEPLOY_DIR
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

# Create build folders
printf "\nCreating build folders...\n\n"
if [ -d "bld" ]; then
	printf "\nRemoving existing build folders...\n\n"
	rm -rfv bld
fi
mkdir bld
cd bld
mkdir Release
cd Release

# Deploy
printf "\nDeploying...\n\n"
cmake -DCMAKE_BUILD_TYPE=Release -DLIBRARY_ROOT_DIR=$DEPLOY_DIR -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32 -DLINK_STATIC=1 -DCMAKE_INSTALL_PREFIX=$DEPLOY_DIR/install $SOURCE_DIR
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nDeployment failed. Quit.\n\n"
	exit $RETURN
fi

printf "\nDeployment Completed.\n\n"
