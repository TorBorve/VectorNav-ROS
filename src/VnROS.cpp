#include "VnROS.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void callBackVectornav(void* userData, Packet& p, size_t index){
    static_cast<VnROS*>(userData)->callback(p, index);
    return;
}

VnROS::VnROS(VnParams param) : params{param} {
    ros::NodeHandle nh;
    odomPub = nh.advertise<nav_msgs::Odometry>("vectornav/Odom", 100);
    tfTimer = nh.createTimer(ros::Duration(1.0/params.asyncOutputRate), &VnROS::broadcastTf, this);
    ROS_INFO("VnROS initialized");
}

void VnROS::connect(){
    ROS_INFO("Connecting to: %s @ %d Baud", params.sensorPort.c_str(), params.sensorBaudRate);
    int defaultBaudRate;
    bool connected = false;
    while(!connected){
        static int i = vnSensor.supportedBaudrates().size() - 1;
        defaultBaudRate = vnSensor.supportedBaudrates()[i];
        ROS_INFO("Connecting with default at %d", defaultBaudRate);
        vnSensor.setResponseTimeoutMs(1000);
        vnSensor.setRetransmitDelayMs(50);
        try{
            if (defaultBaudRate != 128000 && params.sensorBaudRate != 128000){
                vnSensor.connect(params.sensorPort, defaultBaudRate);
                vnSensor.changeBaudRate(params.sensorBaudRate);
                
                connected = true;
                ROS_INFO("Connected baud rate is %d", vnSensor.baudrate());
            }
        } catch(...){
            // disconnect could throw exception if no sensor connected
            vnSensor.disconnect();
            ros::Duration(0.2).sleep();
        }
        i--;
        if (i < 0) {break;}
    }
    if (vnSensor.verifySensorConnectivity()){
        ROS_INFO("Device connection established");
    } else {
        ROS_ERROR("No device comminication");
        ROS_WARN("Please input a valid baud rate. Valid are:");
        ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
        ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
        throw runtime_error("could not connect too sensor");
    }

    // Query the sensor's model number.
    string mn = vnSensor.readModelNumber();
    string fv = vnSensor.readFirmwareVersion();
    uint32_t hv = vnSensor.readHardwareRevision();
    uint32_t sn = vnSensor.readSerialNumber();
    ROS_INFO("Model Number: %s, Firmware Version: %s", mn.c_str(), fv.c_str());
    ROS_INFO("Hardware Revision : %d, Serial Number : %d", hv, sn);

    vnSensor.writeAsyncDataOutputFrequency(params.asyncOutputRate);

    BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		1000 / 200,
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION,	// Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
        GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE | INSGROUP_POSECEF,
        GPSGROUP_NONE);

    vnSensor.writeBinaryOutput1(bor);
    // Set Data output Freq [Hz]
    vnSensor.writeAsyncDataOutputFrequency(params.asyncOutputRate);
    vnSensor.registerAsyncPacketReceivedHandler(this, callBackVectornav);
}

void VnROS::callback(Packet& p, size_t index){
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
    pubOdom(cd);
}

void VnROS::pubOdom(CompositeData& cd){
    // return if no subscribers to topic
    // if (odomPub.getNumSubscribers() == 0){
    //     return;
    // }
    odomMsg.header.stamp = ros::Time::now();
    odomMsg.child_frame_id = params.frameId;
    odomMsg.header.frame_id = params.mapFrameId;

    if (cd.hasPositionEstimatedEcef()){
        vec3d pos = cd.positionEstimatedEcef();

        if (!initialPositonSet){
            initialPositonSet = true;
            initialPosition.x = pos[0];
            initialPosition.y = pos[1];
            initialPosition.z = pos[2];
        }
        odomMsg.pose.pose.position.x = pos[0] - initialPosition[0];
        odomMsg.pose.pose.position.y = pos[1] - initialPosition[1];
        odomMsg.pose.pose.position.z = pos[2] - initialPosition[2];
    }

    if (cd.hasQuaternion()){
        vec4f q = cd.quaternion();
        tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
        // Create a rotation from NED -> ENU
        tf2::Quaternion q_rotate;
        q_rotate.setRPY(M_PI, 0.0, M_PI/2);
        tf2_quat = q_rotate * tf2_quat;
        tf2::Quaternion q2(sqrt(2)/2, sqrt(2)/2, 0, 0);
        tf2_quat = tf2_quat * q2;
        geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf2_quat);
        odomMsg.pose.pose.orientation = quat_msg;
        // odomMsg.pose.pose.orientation.x = q[0];
        // odomMsg.pose.pose.orientation.y = q[1];
        // odomMsg.pose.pose.orientation.z = q[2];
        // odomMsg.pose.pose.orientation.w = q[3];
    }
    if (cd.hasVelocityEstimatedBody()){
        vec3f vel = cd.velocityEstimatedBody();
        odomMsg.twist.twist.linear.x = vel[0];
        odomMsg.twist.twist.linear.y = vel[1];
        odomMsg.twist.twist.linear.z = vel[2];
    }
    if (cd.hasAngularRate()){
        vec3f ar = cd.angularRate();
        odomMsg.twist.twist.angular.x = ar[0];
        odomMsg.twist.twist.angular.y = ar[1];
        odomMsg.twist.twist.angular.z = ar[2];
    }
    odomPub.publish(odomMsg);
    return;
}

void VnROS::pubImu(CompositeData& cd){
    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = params.frameId;
    imuMsg.header.stamp = ros::Time::now();
    if (cd.hasQuaternion()){
        vec4f quat = cd.quaternion();
        imuMsg.orientation.x = quat[0];
        imuMsg.orientation.y = quat[1];
        imuMsg.orientation.z = quat[2];
        imuMsg.orientation.w = quat[3];
    } else {
        // fix invalid quaternion;
        imuMsg.orientation.w = 1;
    }
    if (cd.hasAttitudeUncertainty()){
        vec3f orientationStdDev = cd.attitudeUncertainty();
        imuMsg.orientation_covariance[0] = orientationStdDev[2]*orientationStdDev[2]*M_PI/180; // Convert to radians pitch
        imuMsg.orientation_covariance[4] = orientationStdDev[1]*orientationStdDev[1]*M_PI/180; // Convert to radians Roll
        imuMsg.orientation_covariance[8] = orientationStdDev[0]*orientationStdDev[0]*M_PI/180; // Convert to radians Yaw
    }
    if (cd.hasAngularRate()){
        vec3f ar = cd.angularRate();
        imuMsg.angular_velocity.x = ar[0];
        imuMsg.angular_velocity.y = ar[1];
        imuMsg.angular_velocity.z = ar[2];
    }
    // if ()
    if (cd.hasAcceleration()){
        vec3f al = cd.acceleration();
        imuMsg.linear_acceleration.x = al[0];
        imuMsg.linear_acceleration.y = al[1];
        imuMsg.linear_acceleration.z = al[2];
    }
    // imuPub.publish(imuMsg);
}

void VnROS::broadcastTf(const ros::TimerEvent& event){
    // return if no data recived after startup.
    if (!initialPositonSet){return;}
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = params.mapFrameId;
    transform.header.stamp = event.current_real;
    transform.child_frame_id = params.frameId;
    transform.transform.translation.x = odomMsg.pose.pose.position.x;
    transform.transform.translation.y = odomMsg.pose.pose.position.y;
    transform.transform.translation.z = odomMsg.pose.pose.position.z;
    transform.transform.rotation.x = odomMsg.pose.pose.orientation.x;
    transform.transform.rotation.y = odomMsg.pose.pose.orientation.y;
    transform.transform.rotation.z = odomMsg.pose.pose.orientation.z;
    transform.transform.rotation.w = odomMsg.pose.pose.orientation.w;
    br.sendTransform(transform);
    return;
}

VnROS::~VnROS(){
    // Node has been terminated
    vnSensor.unregisterAsyncPacketReceivedHandler();
    ros::Duration(0.5).sleep();
    ROS_INFO ("Unregisted the Packet Received Handler");
    vnSensor.disconnect();
    ros::Duration(0.5).sleep();
    ROS_INFO ("%s is disconnected successfully", vnSensor.readModelNumber().c_str());
}