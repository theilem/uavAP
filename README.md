# uavAP
<b>uavAP</b> is a modular autopilot framework for the control of unmanned aerial vehicles. While the main focus of the implemented control and planning algorithms is on fixed-wing UAVs, this framework can be used for other types of UAVs (e.g. Quadcopters). Main contricutions of this project are:
<ul>
  <li>A Core framework for Object Interaction</li>
  <li>Utility classes for Inter-Process and Inter-Device Communication etc</li>
  <li>A fully implemented Planning and Control Stack</li>
</ul>

The required operating system is a Linux distribution. The provided setup script runs on Debian/Ubuntu distributions as well as Arch-Linux.


## Dependencies
 - cpsCore (https://github.com/theilem/cpsCore.git)

## Cloning and Compiling

```shell script
git clone https://github.com/theilem/uavAP.git

cd uavAP
mkdir -p bld/release && cd bld/release

cmake -DCMAKE_BUILD_TYPE=Release ../../
make
make install
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
