# uavAP
<b>uavAP</b> is a modular autopilot framework for the control of unmanned aerial vehicles. While the main focus of the implemented control and planning algorithms is on fixed-wing UAVs, this framework can be used for other types of UAVs (e.g. Quadcopters). Main contricutions of this project are:
<ul>
  <li>A Core framework for Object Interaction</li>
  <li>Utility classes for Inter-Process and Inter-Device Communication etc</li>
  <li>A fully implemented Planning and Control Stack</li>
</ul>

The required operating system is a Linux distribution. The provided setup script runs on Debian/Ubuntu distributions as well as Arch-Linux.


## Dependencies
For uavAP with cpsCore
```shell script
sudo apt install build-essential cmake g++ libeigen3-dev libboost-system-dev
```

For uavGS 
```shell script
sudo apt install libqt5svg5-dev qtbase5-dev
```

For XPlane Interface
```shell script
sudo apt install redis-server
```

## Cloning, Compiling, and Installing

For full installation of uavAP with the GroundStation (uavGS) and XPlane Interface
```shell script
git clone https://github.com/theilem/uavAP.git --recurse-submodules

cd uavAP
mkdir bld && cd bld

cmake -DCMAKE_BUILD_TYPE=Release -DGS=1 -DXPLANE=1 ../
make -j$(nproc)
sudo make install
```

To not compile uavGS or the XPlane Interface, set the respective flags to 0. The default is 0 for both.

### Setup the XPlane Interface
After installing X-Plane 11, the XPlane Interface can be set up by creating symbolic links to the installed uavEE plugin.
Navigate to the X-Plane 11 directory and create a symbolic link to the installed plugin.
```shell script
# cd to X-Plane 11 directory
# link the plugin to the X-Plane 11 directory
mkdir -p Resources/plugins/uavAP/64
ln -s /usr/local/lib/lin.xpl ./Resources/plugins/uavAP/64/lin.xpl
# link the configurations directory
ln -s /usr/local/config/uavEE/config ./uavEEConfig
```

## Running the Autopilot (in X-Plane 11)
To run the autopilot, start the X-Plane 11 simulator and load the desired aircraft (Cessna Skyhawk). Select Piatt County and a 3nm approach. After launching the simulation, activate the plugin:
- Press P to pause the simulation
- Click on plugins -> uavEE -> Start Node -> .../sitl.json
- Start the autopilot with the following command in a separate terminal:
```shell script
cd /usr/local/bin
./uavAP/Watchdog ../config/cessna_xplane.json
```
- Start the GroundStation with the following command in a separate terminal:
```shell script
cd /usr/local/bin
./uavGS ../config/uavGS/cessna_xplane.json
```
- Click on plugins -> uavEE -> Enable Autopilot
- Press P to resume the simulation

### Using uavGS
In the GroundStation, first request the parameters from the autopilot:
- Click "Request Params" in the top left field -- this will request the PID cascade parameters from the autopilot
- Click "Update Lists" on the bottom left -- this will update the lists of possible missions (and maneuvers)
- Select a mission, e.g., validation, and click "Send Mission"
- Click on "Request Mission" and "Request Trajectory" on the top right and zoom out to zoom level 12
- Watch it fly 

## Common Issues
### Shared Memory
When the autopilot or other components are not functioning as expected or the sensor data is jumping randomly, there may be some shared memory issue.
The issue arises when uavAP is not properly shut down and the shared memory is not released. To fix this issue, the shared memory can be manually removed.
```shell script
# remove shared memory (this will remove all shared memory)
rm /dev/shm/*
```

## Control Stack

The planning and control stack of uavAP is aimed to be as modular as possible, however, its interfaces are mainly focused on fixed-wing UAV control. The stack is structured as follows:
<ul>
  <li>MissionPlanner creates/reads a collection of waypoints as a Mission</li>
  <li>GlobalPlanner takes the Mission and connects the Waypoints with Trajectory pieces (Lines, Orbits, Cubic Splines)</li>
  <li>LocalPlanner selects a current path section from the trajectory and evaluates the necessary control target to converge towards the section</li>
  <li>Controller calculates the ControllerOutput based on the ControllerTarget provided by the LocalPlanner</li>
</ul>

## References

Theile, M., Dantsker, O., Nai, R., Caccamo, M., & Yu, S. (2020). uavAP: A Modular Autopilot Framework for UAVs. In AIAA AVIATION 2020 FORUM (p. 3268).
