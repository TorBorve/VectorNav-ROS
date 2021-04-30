#include "VnROS.h"
#include "matVecMult.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <thread>

// initial callback from vectornav. userData is a pointer to our class.
void callBackVectornav(void* userData, Packet& p, size_t index){
    static_cast<VnROS*>(userData)->callback(p, index);
    return;
}

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

VnROS::VnROS(ros::NodeHandle* pn) : pn{pn} {
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
    pn->param<int>("async_output_rate", asyncOutputRate, 40);

    ros::NodeHandle nh;
    odomPub = nh.advertise<nav_msgs::Odometry>("vectornav/Odom", 100);
    imuPub = nh.advertise<sensor_msgs::Imu>("vectornav/Imu", 100);
    tfTimer = nh.createTimer(ros::Duration(1.0/asyncOutputRate), &VnROS::broadcastTf, this);
    ROS_INFO("VnROS initialized");
    // fix invalid quaterinon [0, 0, 0, 0]
    odomMsg.pose.pose.orientation.w = 1;
    odomMsg.child_frame_id = params.frameId;
    odomMsg.header.frame_id = params.mapFrameId;
}

void VnROS::connect(){
    // parameters for connection
    std::string sensorPort;
    int sensorBaudRate;
    int asyncOutputRate;
    int sensorImuRate;
    GpsCompassBaselineRegister baseline;
    vec3f antennaOffset;
    bool nedToEnu;
    // get parameters from nodehandle
    pn->param<std::string>("serial_port", sensorPort, "/dev/ttyUSB0");
    pn->param<int>("serial_baud", sensorBaudRate, 115200);
    pn->param<int>("async_output_rate", asyncOutputRate, 40);
    pn->param<int>("fixed_imu_rate", sensorImuRate, 400);
    // load vector parameters into tempVec
    vector<float> tempVec;
    pn->param("baseline", tempVec, {1, 0, 0});
    ROS_ASSERT(tempVec.size() == 3);
    baseline.position[0] = tempVec[0];
    baseline.position[1] = tempVec[1];
    baseline.position[2] = tempVec[2];
    pn->param("antenna_offset", tempVec, {0, 0, 0});
    ROS_ASSERT(tempVec.size() == 3);
    antennaOffset[0] = tempVec[0];
    antennaOffset[1] = tempVec[1];
    antennaOffset[2] = tempVec[2];
    pn->param("ned_to_enu", nedToEnu, false);
    ROS_INFO("Baseline: {%f, %f, %f}", baseline.position[0], baseline.position[1], baseline.position[2]);
    ROS_INFO("Antenna offset: {%f, %f, %f}", antennaOffset[0], antennaOffset[1], antennaOffset[2]);

    ROS_INFO("Connecting to: %s @ %d Baud", sensorPort.c_str(), sensorBaudRate);
    int defaultBaudRate;
    bool connected = false;
    while(!connected){
        static int i = vnSensor.supportedBaudrates().size() - 1;
        defaultBaudRate = vnSensor.supportedBaudrates()[i];
        ROS_INFO("Connecting with default at %d", defaultBaudRate);
        vnSensor.setResponseTimeoutMs(1000);
        vnSensor.setRetransmitDelayMs(50);
        try{
            if (defaultBaudRate != 128000 && sensorBaudRate != 128000){
                vnSensor.connect(sensorPort, defaultBaudRate);
                vnSensor.changeBaudRate(sensorBaudRate);
                
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

    vnSensor.writeAsyncDataOutputFrequency(asyncOutputRate);

    BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		sensorImuRate / asyncOutputRate,
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION,	// Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
        GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE | INSGROUP_POSECEF,
        GPSGROUP_NONE);

    vnSensor.writeBinaryOutput1(bor);
    // Set Data output Freq [Hz]
    vnSensor.writeAsyncDataOutputFrequency(asyncOutputRate);
    mat3f rotation = mat3f::identity();
    if (nedToEnu){
        rotation = {{0, 1, 0}, 
                    {1, 0, 0}, 
                    {0,0, -1}};
    }
    baseline.position = rotation * baseline.position;
    antennaOffset = rotation * antennaOffset;
    // write baseline.
    vnSensor.writeGpsCompassBaseline(baseline);
    // write antenna offset
    vnSensor.writeGpsAntennaOffset(antennaOffset);
    // write reference frame / mounting of sensor
    vnSensor.writeReferenceFrameRotation(rotation);

    // register callback function
    vnSensor.registerAsyncPacketReceivedHandler(this, callBackVectornav);
    return;
}

void VnROS::callback(Packet& p, size_t index){
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
    pubOdom(cd);
    pubImu(cd);
    return;
}

void VnROS::pubOdom(CompositeData& cd){
    odomMsg.header.stamp = ros::Time::now();
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
        tf2::Quaternion tf2_quat(q[1], q[0], -q[2], q[3]);
        // Create a rotation from NED -> ENU
        // tf2::Quaternion q_rotate;
        // q_rotate.setRPY(M_PI, 0.0, M_PI/2);
        // tf2_quat = q_rotate * tf2_quat;
        // tf2::Quaternion q2(sqrt(2)/2, sqrt(2)/2, 0, 0);
        // tf2_quat = tf2_quat * q2;
        geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf2_quat);
        odomMsg.pose.pose.orientation = quat_msg;
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
    // return if no subsribers
    if (imuPub.getNumSubscribers() == 0){
        return;
    }
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
        // fix invalid quaternion
        imuMsg.orientation.w = 1;
    }
    if (cd.hasAttitudeUncertainty()){
        // large uncertainty on startup
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
    if (cd.hasAcceleration()){
        vec3f al = cd.acceleration();
        imuMsg.linear_acceleration.x = al[0];
        imuMsg.linear_acceleration.y = al[1];
        imuMsg.linear_acceleration.z = al[2];
    }
    imuMsg.angular_velocity_covariance = params.angularVelCovariance;
    imuMsg.linear_acceleration_covariance = params.linearAccelCovariance;
    imuPub.publish(imuMsg);
    return;
}

void VnROS::broadcastTf(const ros::TimerEvent& event){
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

void VnROS::disconnect(){
    vnSensor.unregisterAsyncPacketReceivedHandler();
    vnSensor.disconnect();
    return;
}

VnROS::~VnROS(){
    // Node has been terminated
    // check if sensor is connected
    if (vnSensor.verifySensorConnectivity()){
        vnSensor.unregisterAsyncPacketReceivedHandler();
        ros::Duration(0.5).sleep();
        ROS_INFO ("Unregisted the Packet Received Handler");
        vnSensor.disconnect();
        ros::Duration(0.5).sleep();
        ROS_INFO ("%s is disconnected successfully", vnSensor.readModelNumber().c_str());
    }
}