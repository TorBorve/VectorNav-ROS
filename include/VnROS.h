#pragma once

#include <iostream>
#include <cmath>

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

class VnParams {
public:
    boost::array<double, 9ul> linearAccelCovariance;
    boost::array<double, 9ul> angularVelCovariance;
    boost::array<double, 9ul> orientationCovariance;
    std::string mapFrameId;
    std::string frameId;
    bool nedToEnu;
    //std::string sensorPort;
    //int sensorBaudRate;
    //int asyncOutputRate;
    //int sensorImuRate;
};

class VnROS {
public:
    VnROS(ros::NodeHandle* pn);
    void connect();
    void disconnect();
    void callback(Packet& p, size_t index);
    ~VnROS();
private:
    void pubOdom(CompositeData& cd);
    void pubImu(CompositeData& cd);
    void broadcastTf(const ros::TimerEvent& event);
    void writeSettings();
    void getParams(int& asyncRate, int& imuRate, GpsCompassBaselineRegister& baseline,
                    vec3f& antennaOffset, bool& nedToEnu, int& baudRate);
    void printSettings();
    // sensor
    vn::sensors::VnSensor vnSensor;

    // Nodehandle to get params
    ros::NodeHandle* pn;
    
    // publishers
    ros::Publisher odomPub;
    ros::Publisher imuPub;

    // params
    VnParams params;
    
    bool initialPositonSet = false;
    vec3d initialPosition;

    ros::Timer tfTimer;
    tf2_ros::TransformBroadcaster br;
    nav_msgs::Odometry odomMsg;
};