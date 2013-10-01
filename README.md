octocopter
==========

This repository hosts a set of programs used for research of autonomous exploration of outdoor environments using unmanned aerial vehicles. The basic idea is that the user defines a virtual bounding box around the UAVs start-position (eg. 500*500 meters wide and 50 meters high) and lets the UAV create a 3d map of that environment.

We use a UAV equipped with an INS and (multiple) LIDAR sensors to first create a point cloud of outdoor environments. This has been working since Febuary 2012.

Next, gaps are detected in the pointcloud using a GPU-accelerated algorithm based on particles testing the cloud's watertightness. Then, an occupancy grid is populated and used to create a safe path between waypoints close to gaps found in the previous step. During flight, the path is continually checked for obstacles: whenever parts become obstructed, it is re-planned to avoid collisions.

In detail, the subdirectories contain:

* __3d-data__ used in basestation and simulator for visualization of the UAV.
* __basestation__ A GUI-application executed on the base (notebook) in the field. It connects to the rover (which is running "koptercontrol") using TCP/IP via WiFi, receives the point cloud, generates waypoints and checks their safety during flight. basestation can also connect to the "simulator" instead to allow tests is simulation. Furthermore, it can open log files created by "koptercontrol" and replay flights, showing position and attitude, point cloud capture, waypoints in real-time (or faster).
* __cloudcompare__ A utility program comparing two point clouds, used to analyze precision of different INS/LIDAR sensor fusion approaches.
* __common__ contains classes.
* __fclogconverter__ converts bainry flightcontroller logs into CSV files for inspection using e.g. matlab.
* __filewriter__ is a program used for stress-testing of logging. Since the rover's mass-storage is a SD-card, writing too much log-data can make the system unresponsive, which must be avoided as flight-control is running on the same machine with hard real-time requirements.
* __hokuyoid__ is a helper program to allow udev to assign unique device filenames (based on serial number) to multiple Hokuyo SCIP-2.0 compliant laser scanners.
* __koptercontrol__ runs on the rover/UAV. It controls the UAV itself (kopter.cpp/flightcontoller.cpp), GNSS/IMU/INS board (gnssdevice.cpp/sbfparser.cpp), multiple laser scanners (hokuyo.cpp) and keeps a connection to basestation.
* __lrflogwriter__ and lrfreader are test implementations for efficient logfile storage of LIDAR data.
* __portlink__ allows linking unlimited numbers of serial/usb/network-connections with each other. This is useful in many situations, e.g. forwarding differential corrections from the RTK base station from TCP/IP ports to the INS's USB-port or allowing direct access to the INS's data&command-ports remotely via WiFi instead of using serial cables.
* __rtkbase__ is currently unused. It configures a septentrio receiver to become a RTK base station and forwards the differential corrections coming on a serial port to the network.
* __scanconverter__ converts previously captured laser scanner logs into binary format.
* __simulator__ implements the same network protocol as "koptercontrol", so that "basestation" can connect to it. This way, many mechanisms can be tested during bad weather without having to fly outside. In Hamburg/Germany, this is important :)

All of the software below is written in C++ and designed to work in real-time on UAV and base station. It is developed and used on (Ubuntu) linux. The requirements are:

* Qt 5.1
* A graphics subsystem providing OpenGL 3.3 
* assimp (for 3d model loading)
* SDL (for sound)
* DevIL (loading images as textures into OpenGL)
* Ogre (for the simulator)

For complete functionality, basestation further requires:

* CUDA 5.0
* NVIDIA GPU with at least 1GB VRAM and compute capability 2.0 or higher

If you have any questions, find me at http://tams.informatik.uni-hamburg.de/people/adler/. There's also a youtube channel containing videos that are mostly related: http://www.youtube.com/user/kernelpnc