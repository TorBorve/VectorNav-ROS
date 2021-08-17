#pragma once

/// include vectornav files
#include "vn/sensors.h"
#include "vn/compositedata.h"

/// include ros files
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace vnRos {

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
        /// @param[in] index number for packet
        static void callback(void* userData, vn::protocol::uart::Packet& p, size_t index);

        /// @brief callback function for async data used on startup. 
        /// Changes too normal callback when startup is complete.
        /// @param[in] userData pointer to VnRos class
        /// @param[in] p data recived from sensor
        /// @param[in] index number for packet
        static void startupCallback(void* userData, vn::protocol::uart::Packet& p, size_t index);

        /// @brief destructor for VnRos class. Disconnects if sensor is connected
        /// @brief publishes odom message to odom topic. 
        /// @param[in] cd Data recived from sensor.
        void pubOdom(vn::sensors::CompositeData& cd);

        /// @brief publishes imu message to imu topic
        /// @param[in] cd Data recived from sensor
        void pubImu(vn::sensors::CompositeData& cd);

        /// @brief publishes InsStatus message to ins status topic
        /// @param[in] cd Data recived from sensor
        void pubStatus(vn::sensors::CompositeData& cd);

        /// @brief callback function for tf broadcaster
        /// @param[in] event contains info about the time
        void broadcastTf(const ros::TimerEvent& event);

        /// @brief writes settings given in launch file to vnSensor.
        void writeSettings();

        /// @brief reads from private nodehandle into params given.
        /// @param[in/out] asyncRate rate for async callback data
        /// @param[in/out] imuRate rate for imu data
        /// @param[in/out] baseline vector from antenna A too B and uncertainty
        /// @param[in/out] antannaOffset vector form sensor too antenna A
        /// @param[in/out] baudRate baudrate for sensor
        void getParams(int& asyncRate, int& imuRate, vn::sensors::GpsCompassBaselineRegister& baseline,
                        vn::math::vec3f& antennaOffset, int& baudRate);

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

        /// @brief publisher for ins status
        ros::Publisher insStatusPub;

        /// @brief parameters for VnRos
        VnParams params;
        
        /// @brief bool for recived a position
        bool initialPositonSet = false;
        /// @brief initial position vector recived from sensor
        vn::math::vec3d initialPosition;

        /// @brief timer for tf broadcaster 
        ros::Timer tfTimer;
        /// @brief tf broadcaster for odom->baselink
        tf2_ros::TransformBroadcaster br;

        /// @brief latest odomMsg recived form sensor
        nav_msgs::Odometry odomMsg;
    };

}