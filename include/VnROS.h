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

/// @brief parameters for VnRos class
class VnParams {
public:
    boost::array<double, 9ul> linearAccelCovariance;
    boost::array<double, 9ul> angularVelCovariance;
    boost::array<double, 9ul> orientationCovariance;
    std::string mapFrameId;
    std::string frameId;
    bool nedToEnu;
};

/// @brief ROS interface class for vectornav
class VnRos {
public:
    /// @brief constructor for VnRos
    /// @param[in] pn pointer to private nodehandle used for reading parameters given in launch file
    VnRos(ros::NodeHandle* pn);

    /// @brief connect to vn sensor
    /// @exception runtime_error thrown if unable to connect
    void connect();

    /// @brief disconnect from vn sensor
    void disconnect();

    ~VnRos();
private:
    /// @brief callback function for async data.
    /// @param[in] userData pointer to VnRos class
    /// @param[in] p data recived from sensor
    /// @param[in] index number for packet?
    static void callback(void* userData, Packet& p, size_t index);

    static void startupCallback(void* userData, Packet& p, size_t index);

    /// @brief destructor for VnRos class. Disconnects if sensor is connected
    /// @brief publishes odom message to odom topic. 
    /// @param[in] cd Data recived from sensor.
    void pubOdom(CompositeData& cd);

    /// @brief publishes imu message to imu topic
    /// @param[in] cd Data recived from sensor
    void pubImu(CompositeData& cd);

    /// @brief callback function for tf broadcaster
    /// @param[in] event contains info about the time
    void broadcastTf(const ros::TimerEvent& event);

    /// @brief writes settings given in launch file to vnSensor.
    void writeSettings();

    /// @brief reads from private nodehandle into params given.
    /// @param[in] asyncRate rate for async callback data
    /// @param[in] imuRate rate for imu data?
    /// @param[in] baseline vector from antenna A too B and uncertainty
    /// @param[in] antannaOffset vector form sensor too antenna A
    /// @param[in] baudRate baudrate for sensor
    void getParams(int& asyncRate, int& imuRate, GpsCompassBaselineRegister& baseline,
                    vec3f& antennaOffset, int& baudRate);

    /// @brief prints current settings of vnSensor
    void printSettings();
    
    /// @brief VnSensor class for communicatin with sensor
    vn::sensors::VnSensor vnSensor;

    /// @brief private nodehande to get params from launch file
    ros::NodeHandle* pn;
    
    /// @brief publisher for odometry
    ros::Publisher odomPub;
    
    /// @brief publisher for Imu
    ros::Publisher imuPub;

    ros::Publisher insStatusPub;

    /// @brief parameters for VnRos
    VnParams params;
    
    /// @brief bool for recived a position
    bool initialPositonSet = false;
    /// @brief initial position vector recived from sensor
    vec3d initialPosition;

    /// @brief timer for tf broadcaster 
    ros::Timer tfTimer;
    /// @brief tf broadcaster for odom->baselink
    tf2_ros::TransformBroadcaster br;

    /// @brief latest odomMsg recived form sensor
    nav_msgs::Odometry odomMsg;
};