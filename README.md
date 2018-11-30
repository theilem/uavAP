# uavAP
<b>uavAP</b> is a modular autopilot framework for the control of unmanned aerial vehicles. While the main focus of the implemented control and planning algorithms is on fixed-wing UAVs, this framework can be used for other types of UAVs (e.g. Quadcopters). Main contricutions of this project are:
<ul>
  <li>A Core framework for Object Interaction</li>
  <li>Utility classes for Inter-Process and Inter-Device Communication etc</li>
  <li>A fully implemented Planning and Control Stack</li>
</ul>

The required operating system is a Linux distribution. The provided setup script runs on Debian/Ubuntu distributions as well as Arch-Linux.


<b>Setup</b>

For the setup of uavAP the setup.sh script can be executed. It installs necessary dependencies from the regular debian repository. Additionally, libraries (such as boost, eigen, protobuf etc.) are compiled from source and installed on the system. 
The setup script adds the build folders as ./bld/Release and ./bld/Debug. Entering the required path and executing make and (sudo) make install, installs the libraries and executables of uavAP.

If the .proto files are changed, the generate_proto.sh script in the repository root folder has to be executed.


<b>Core Framework</b>

The Core of uavAP is meant to be used for different projects and is thus highly generalized. The main idea is that a specific Helper (e.g. FlightControlHelper) reads in a configuration file and based on that creates specified Objects using Factories. These Objects are aggregated, meaning that they are interconnected to meet the dependencies of each object, and run through 3 RunStages (INIT, NORMAL, FINAL). During these stages the dependencies are evaluated, schedule prepared, and finally started. 


<b>Utility Classes</b>

Main utility classes implemented in uavAP are:
<ul>
  <li>Inter-Process Communcation (Using Shared Memory or Message Queues)</li>
  <li>Inter-Device Communcation (Serial Communication)</li>
  <li>Data Presentation for serialization of structs and objects</li>
  <li>Scheduler for scheduled execution of tasks (Multi-threaded)</li>
  <li>APLogger for logging of debug/error messages</li>
  <li>PropertyMapper for parsing of .json configuration files and mapping the content to objects</li>
</ul>


<b>Control Stack</b>

The planning and control stack of uavAP is aimed to be as modular as possible, however, its interfaces are mainly focused on fixed-wing UAV control. The stack is structured as follows:
<ul>
  <li>MissionPlanner creates/reads a collection of waypoints as a Mission</li>
  <li>GlobalPlanner takes the Mission and connects the Waypoints with Trajectory pieces (Lines, Orbits, Cubic Splines)</li>
  <li>LocalPlanner selects a current path section from the trajectory and evaluates the necessary control target to converge towards the section</li>
  <li>Controller calculates the ControllerOutput based on the ControllerTarget provided by the LocalPlanner</li>
</ul>

