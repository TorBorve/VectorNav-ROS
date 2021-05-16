Vectornav ROS
====================

**[To do-list at the bottom](#to-do-list)**

A ROS interface for VectorNav.

This package use the C++ library provided by VectorNav together with ROS.
It starts a ROS node wich publishes IMU, Odometry and status messages. This repository is built upon Dereck Wonnacotts repository: https://github.com/dawonn/vectornav. 
It is developed for the student organisation DNV Fuel Fighter. It is used to provide odometry for the autnomous car.

Setup
----------------

First we need to make sure we have accses to the USB port wich the VectorNav unit is connected to.
To do this we need to add your user to the dailout group. This will give us permision to talk to the USB port. This is done by running:

```
sudo adduser "your username" dialout
```
It is necesary to log out and in or restart your computer for the changes to take effect.

Now we need to clone this repository in your desired workspace and build it. For example:
```
cd ~catkin_ws/src
git clone https://gitlab.stud.idi.ntnu.no/fuelfighter/autonomous/odometry/vectornav.git
cd ..
catkin build (or catkin_make)
```

Now we are ready to run the launch file. This launches the vectornav node which will publish the data recived by the vectornav unit.

```
source devel/setup.bash
roslaunch vectornav vn300.launch
```

Overview 
--------

The code is launched from the file vn300.launch using the vn300 executable. The launch file uses several parameters such as USB port and frame_id. These parameters can be changed as desired. The code is written in C++ and uses the VectorNav C++ library. On top of this we have made the class VnRos wich is used to write data to the VectorNav unit and publish the recived data. The code is written for the vn300 model. It should also work for the other models, but there could be problems.

Known issues
------------

### crashes after a few seconds
The code sometimes crashes on startup. This often happens the first time you run the code for the day and when you changes parameters such as serial_baud. The solution is to simply rerun it, then it should work.

### crashes instanly
This might be due to your user is not a part of the dailout group. Thus you don't have permission to communicate over USB. See setup section on how to solve this.
If that doesn't work you could try
```
sudo chmod a+rw /dev/ttyUSB0
```
Note that this is not permanent so it need to be done each time you turn on your computer. Therefore i recommend the solution described in the setup section.

References 
----------
References used for this repository:

- http://www.vectornav.com/ "VectorNav"
- https://github.com/dawonn/vectornav "orginal ROS interface"

To do-list
===========

- [x] Add IMU topic
- [ ] Create function to set ENU flag
- [x] Implement antenna layout setting (`VnSensor::writeGpsAntennaOffset`, `VnSensor::writeGpsCompassBaseline`)
- [ ] TF + SLAM harmony
