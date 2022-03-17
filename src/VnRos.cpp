/// include ros interface files
#include "VnRos.h"
#include "utilities.h"

/// include standar C++ files
#include <bitset>

/// include ros files
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "vectornav/InsStatus.h"

/// include vectornav files from C++ library
#include <vn/sensors.h>
#include <vn/compositedata.h>
#include <vn/util.h>
#include <vn/thread.h>

// Basic loop so we can initilize our covariance parameters
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    boost::array<double, 9ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 9; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}

namespace vnRos {

    VnRos::VnRos(ros::NodeHandle* pn) : pn{pn} {
        // Load all params
        pn->param<std::string>("map_frame_id", params.mapFrameId, "map");
        pn->param<std::string>("frame_id", params.frameId, "vectornav");
        pn->param<bool>("tf_ned_to_enu", params.nedToEnu, false);

        //Call to set covariances
        XmlRpc::XmlRpcValue rpc_temp;
        if(pn->getParam("linear_accel_covariance",rpc_temp)){
            params.linearAccelCovariance = setCov(rpc_temp);
        }
        if(pn->getParam("angular_vel_covariance",rpc_temp)){
            params.angularVelCovariance = setCov(rpc_temp);
        }
        if(pn->getParam("orientation_covariance",rpc_temp)){
            params.orientationCovariance = setCov(rpc_temp);
        }
        int asyncOutputRate;
        std::string ns;
        pn->param<int>("async_output_rate", asyncOutputRate, 40);
        pn->param<std::string>("ns", ns, "vectornav");

        ros::NodeHandle nh;
        odomPub = nh.advertise<nav_msgs::Odometry>(ns + "/Odom", 100);
        imuPub = nh.advertise<sensor_msgs::Imu>(ns + "/Imu", 100);
        insStatusPub = nh.advertise<vectornav::InsStatus>(ns + "/InsStatus", 100);
        twistPub = nh.advertise<geometry_msgs::TwistStamped>(ns + "/twist", 100);
        tfTimer = nh.createTimer(ros::Duration(1.0/asyncOutputRate), &VnRos::broadcastTf, this);
        
        // fix invalid quaterinon = [0, 0, 0, 0]
        odomMsg.pose.pose.orientation.w = 1;
        odomMsg.child_frame_id = params.frameId;
        odomMsg.header.frame_id = params.mapFrameId;
        ROS_INFO("VnROS initialized");
    }

    void VnRos::connect(){
        // parameters for connection
        std::string sensorPort;
        int sensorBaudRate;

        // get parameters from nodehandle
        pn->param<std::string>("serial_port", sensorPort, "/dev/ttyUSB0");
        pn->param<int>("serial_baud", sensorBaudRate, 115200);

        ROS_INFO("Connecting to: %s @ %d Baud", sensorPort.c_str(), sensorBaudRate);

        // update response timeout to avoid timeout exception
        vnSensor.setResponseTimeoutMs(1000);
        vnSensor.setRetransmitDelayMs(50);
        // prioritize connecting to sensorBaudrate and defuault baudrate
        std::vector<uint32_t> baudrateQueue = {(uint32_t)sensorBaudRate, 115200};
        // append supported baudrates
        for (const auto& baudrate : vnSensor.supportedBaudrates()) { baudrateQueue.push_back(baudrate);}
        // try connecting to the baudrates
        for (const auto& baudrate : baudrateQueue){
            ROS_INFO("Connecting with baudrate: %d", baudrate);
            try {
                vnSensor.connect(sensorPort, baudrate);
            } catch (...){ }
            // check if connection was sucessfull
            if (vnSensor.verifySensorConnectivity()){
                break;
            }
        }
        if (vnSensor.verifySensorConnectivity()){
            ROS_INFO("Device connection established");
        } else {
            ROS_ERROR("No device comminication");
            ROS_WARN("Please input a valid baud rate. Valid are:");
            ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
            ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
            throw std::runtime_error("could not connect to sensor");
        }
        vnSensor.setResponseTimeoutMs(5000);
        vnSensor.setRetransmitDelayMs(200);
        try{
            writeSettings();
            printSettings();
            ROS_INFO("Finished");
        } catch (std::exception& e){
            ROS_ERROR("Exception while writing and reading settings from vectornav sensor.\n Error: %s", e.what());
            throw std::runtime_error("could not write and/or read settings from vectornav sensor");
        } catch (...){
            ROS_ERROR("Exception while writing and reading settings from vectornav sensor.");
            throw std::runtime_error("could not write and/or read settings form vectornav sensor.");
        }
        
        // decide if you want the startup callback function or go straight to callback function
        // vnSensor.registerAsyncPacketReceivedHandler(this, VnRos::startupCallback);
        vnSensor.registerAsyncPacketReceivedHandler(this, VnRos::callback);
        return;
    }

    void VnRos::callback(void* userData, vn::protocol::uart::Packet& p, size_t index){
        VnRos* vnRos = static_cast<VnRos*>(userData);
        auto cd = vn::sensors::CompositeData::parse(p);
        vnRos->pubOdom(cd);
        vnRos->pubImu(cd);
        vnRos->pubStatus(cd);
        return;
    }

    void VnRos::pubOdom(vn::sensors::CompositeData& cd){
        odomMsg.header.stamp = ros::Time::now();
        if (cd.hasPositionEstimatedEcef() && (cd.positionEstimatedEcef() != vn::math::vec3d{0.0})){
            vn::math::vec3d pos = cd.positionEstimatedEcef();

            if (!initialPositonSet){
                initialPositonSet = true;
                initialPosition = pos;
            }
            odomMsg.pose.pose.position = utilities::toMsg(pos - initialPosition);
        }

        if (cd.hasQuaternion()){
            vn::math::vec4f q = cd.quaternion();
            tf2::Quaternion tf2_quat(q[1], q[0], -q[2], q[3]);
            odomMsg.pose.pose.orientation = tf2::toMsg(tf2_quat);
        }
        if (cd.hasVelocityEstimatedBody()){
            odomMsg.twist.twist.linear = utilities::toMsg(cd.velocityEstimatedBody());
        }
        if (cd.hasAngularRate()){
            odomMsg.twist.twist.angular = utilities::toMsg(cd.angularRate());
        }
        geometry_msgs::TwistStamped twist;
        twist.header = odomMsg.header;
        twist.twist = odomMsg.twist.twist;
        twistPub.publish(twist);
        odomPub.publish(odomMsg);
        return;
    }

    void VnRos::pubImu(vn::sensors::CompositeData& cd){
        // return if no subsribers
        if (cd.hasQuaternion() && imuPub.getNumSubscribers() != 0) 
            // && cd.hasAngularRate() && cd.hasAcceleration())
        {
            sensor_msgs::Imu imuMsg;
            imuMsg.header.frame_id = params.frameId;
            imuMsg.header.stamp = ros::Time::now();
            if (cd.hasQuaternion()){
                imuMsg.orientation = utilities::toMsg(cd.quaternion());
            }
            if (cd.hasAttitudeUncertainty()){
                // large uncertainty on startup
                vn::math::vec3f orientationStdDev = cd.attitudeUncertainty();
                imuMsg.orientation_covariance[0] = orientationStdDev[2]*orientationStdDev[2]*M_PI/180; // Convert to radians pitch
                imuMsg.orientation_covariance[4] = orientationStdDev[1]*orientationStdDev[1]*M_PI/180; // Convert to radians Roll
                imuMsg.orientation_covariance[8] = orientationStdDev[0]*orientationStdDev[0]*M_PI/180; // Convert to radians Yaw
            }
            if (cd.hasAngularRate()){
                imuMsg.angular_velocity = utilities::toMsg(cd.angularRate());
            }
            if (cd.hasAcceleration()){
                imuMsg.linear_acceleration = utilities::toMsg(cd.acceleration());
            }

            imuMsg.angular_velocity_covariance = params.angularVelCovariance;
            imuMsg.linear_acceleration_covariance = params.linearAccelCovariance;
            imuPub.publish(imuMsg);
        }
        return;
    }

    void VnRos::pubStatus(vn::sensors::CompositeData& cd){
        if (cd.hasInsStatus() && cd.hasQuaternion() && insStatusPub.getNumSubscribers() != 0){
            utilities::InsStatus status{cd.insStatus()};
            vectornav::InsStatus statusMsg = utilities::toMsg(status);
            statusMsg.header.stamp = ros::Time::now();
            statusMsg.header.frame_id = params.frameId;
            insStatusPub.publish(statusMsg);
        }
        return;
    }

    void VnRos::broadcastTf(const ros::TimerEvent& event){
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = params.mapFrameId;
        transform.header.stamp = event.current_real;
        transform.child_frame_id = params.frameId;
        transform.transform.translation = utilities::PointToVec3(odomMsg.pose.pose.position);
        transform.transform.rotation = odomMsg.pose.pose.orientation;
        br.sendTransform(transform);
        return;
    }

    void VnRos::writeSettings(){
        int sensorBaudRate;
        int asyncOutputRate;
        int sensorImuRate;
        vn::sensors::GpsCompassBaselineRegister baseline;
        vn::math::vec3f antennaOffset;
        getParams(asyncOutputRate, sensorImuRate, baseline, antennaOffset, sensorBaudRate);
        // write setting to sensor
        vnSensor.changeBaudRate(sensorBaudRate);

        using namespace vn::protocol::uart;
        // specify what data we want the sensor to send
        vn::sensors::BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            sensorImuRate / asyncOutputRate,
            COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION |
            COMMONGROUP_INSSTATUS | COMMONGROUP_VELOCITY,	// Note use of binary OR to configure flags.
            TIMEGROUP_NONE | TIMEGROUP_TIMESTARTUP,
            IMUGROUP_NONE | IMUGROUP_ANGULARRATE | IMUGROUP_ACCEL,
            GPSGROUP_NONE,
            ATTITUDEGROUP_NONE | ATTITUDEGROUP_VPESTATUS | ATTITUDEGROUP_YPRU,
            INSGROUP_NONE | INSGROUP_POSECEF |INSGROUP_VELNED| INSGROUP_VELBODY ,
            GPSGROUP_NONE);
        vnSensor.writeBinaryOutput1(bor);
        vn::xplat::Thread::sleepSec(1);

        // write antenna offset
        vnSensor.writeGpsAntennaOffset(antennaOffset);
        vn::xplat::Thread::sleepSec(1);

        // write baseline.
        vnSensor.writeGpsCompassBaseline(baseline);
        vn::xplat::Thread::sleepSec(1);

        // Set Data output Freq [Hz]
        vnSensor.writeAsyncDataOutputFrequency(asyncOutputRate);
        vn::xplat::Thread::sleepSec(1);
        // write reference frame / mounting of sensor
        // mat3f rotation = mat3f::identity();
        // if (params.nedToEnu){
        //     rotation = {0, 1, 0,
        //                 1, 0, 0,
        //                 0, 0, -1};
        // }
        // vnSensor.writeReferenceFrameRotation(rotation);
        return;
    }

    void VnRos::getParams(int& asyncRate, int& imuRate, vn::sensors::GpsCompassBaselineRegister& baseline,
                        vn::math::vec3f& antennaOffset, int& baudRate)
    {
        // read params from nodehandle
        pn->param<int>("serial_baud", baudRate, 115200);
        pn->param<int>("async_output_rate", asyncRate, 40);
        pn->param<int>("fixed_imu_rate", imuRate, 400);

        // load vector parameters into tempVec
        std::vector<float> tempVec;
        pn->param("baseline", tempVec, {1, 0, 0});
        baseline.position = utilities::toVnVec<3>(tempVec);

        pn->param("baseline_uncertainty", tempVec, {0.2, 0.2, 0.2});
        baseline.uncertainty = utilities::toVnVec<3>(tempVec);

        pn->param("antenna_offset", tempVec, {0, 0, 0});
        antennaOffset = utilities::toVnVec<3>(tempVec);
        return;
    }

    void VnRos::printSettings(){
        std::stringstream ss;
        // Query the sensor's model number.
        std::string mn = vnSensor.readModelNumber();
        std::string fv = vnSensor.readFirmwareVersion();
        uint32_t hv = vnSensor.readHardwareRevision();
        uint32_t sn = vnSensor.readSerialNumber();
        ss << "VnROS settings:" << std::endl; 
        ss << "\tModel Number: " << mn << ", Firmware version: " << fv << std::endl;
        ss << "\tHardware Revision: " << hv << ", Serial number: " << sn << std::endl;

        // read settings form sensor
        int sensorBaudRate = vnSensor.readSerialBaudRate();
        int asyncOutputRate = vnSensor.readAsyncDataOutputFrequency();
        vn::sensors::GpsCompassBaselineRegister baseline = vnSensor.readGpsCompassBaseline();
        vn::math::mat3f rotation = vnSensor.readReferenceFrameRotation();
        vn::math::vec3f antennaOffset = vnSensor.readGpsAntennaOffset();
        ss << "\tBaudrate: " << sensorBaudRate << ", Async rate: " << asyncOutputRate << std::endl;
        ss << "\tBaseline: " << str(baseline.position) << std::endl;
        ss << "\tBaseline uncertainty: " << str(baseline.uncertainty) << std::endl;
        ss << "\tAntenna offset: " << str(antennaOffset) << std::endl;
        ss << "\tRefrence frame rotation: " << str(rotation) << std::endl;
        ROS_INFO("%s", ss.str().c_str());
        return;
    }

    void VnRos::disconnect(){
        vnSensor.unregisterAsyncPacketReceivedHandler();
        vnSensor.disconnect();
        return;
    }

    void VnRos::startupCallback(void* userData, vn::protocol::uart::Packet& p, size_t index){
        // static variables for printing status to console
        static const ros::Duration printPeriod{5}; 
        static ros::Time lastPrint = ros::Time::now() - printPeriod;

        VnRos* vnRos = static_cast<VnRos*>(userData);
        auto cd = vn::sensors::CompositeData::parse(p);

        // check if the cd is valid
        if (cd.hasInsStatus() && cd.hasQuaternion()){
            vnRos->pubStatus(cd);
            utilities::InsStatus status{cd.insStatus()};

            // check if it is time for printing
            if (ros::Duration{ros::Time::now() - lastPrint} >= printPeriod){
                ROS_INFO_STREAM(status); 
                ROS_INFO_STREAM(std::bitset<16>(cd.insStatus()));
                lastPrint = ros::Time::now();
            }
            // check if startup is complete.
            if (status.isOk()){
                // changes too new callback function
                vnRos->vnSensor.unregisterAsyncPacketReceivedHandler();
                vnRos->vnSensor.registerAsyncPacketReceivedHandler(userData, VnRos::callback);
                ROS_INFO("Changed callback function too VnRos::callback");
            };
        }
        return;
    }

    VnRos::~VnRos(){
        // check if sensor is connected

        // might not be needed due to destructor of VnSenor.
        if (vnSensor.verifySensorConnectivity()){
            vnSensor.unregisterAsyncPacketReceivedHandler();
            ros::Duration(0.5).sleep();
            ROS_INFO ("Unregisted the Packet Received Handler");
            vnSensor.disconnect();
            ros::Duration(0.5).sleep();
            ROS_INFO ("%s is disconnected successfully", vnSensor.readModelNumber().c_str());
        }
    }

}