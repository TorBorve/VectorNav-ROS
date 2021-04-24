#pragma once

#include <iostream>
#include <cmath>

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

#include <ros/ros.h>

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
    bool tfNedToEnu;
    bool frameBasedEnu;
    std::string sensorPort;
    int sensorBaudRate;
    int asyncOutputRate;
    int sensorImuRate;
};

class VnROS {
public:
    VnROS(VnParams param);
    void connect();
    void callback(Packet& p, size_t index);
    ~VnROS();
private:
    void pubOdom(CompositeData& cd);
    // sensor
    vn::sensors::VnSensor vnSensor;
    
    // publishers
    ros::Publisher odomPub;

    // params
    VnParams params;
    
    bool initialPositonSet = false;
    vec3d initialPosition;
};