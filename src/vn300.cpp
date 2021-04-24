/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <iostream>

// ROS Libraries
#include "ros/ros.h"

#include "VnROS.h"

using namespace std;

// Basic loop so we can initilize our covariance parameters above
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


int main(int argc, char *argv[])
{

    // ROS node init
    ros::init(argc, argv, "vectornav");
    ros::NodeHandle pn("~");

    VnParams params;
    // Load all params
    pn.param<std::string>("map_frame_id", params.mapFrameId, "map");
    pn.param<std::string>("frame_id", params.frameId, "vectornav");
    pn.param<bool>("tf_ned_to_enu", params.tfNedToEnu, false);
    pn.param<bool>("frame_based_enu", params.frameBasedEnu, false);
    pn.param<int>("async_output_rate", params.asyncOutputRate, 40);
    pn.param<std::string>("serial_port", params.sensorPort, "/dev/ttyUSB0");
    pn.param<int>("serial_baud", params.sensorBaudRate, 115200);
    pn.param<int>("fixed_imu_rate", params.sensorImuRate, 800);

    //Call to set covariances
    XmlRpc::XmlRpcValue rpc_temp;
    if(pn.getParam("linear_accel_covariance",rpc_temp))
    {
        params.linearAccelCovariance = setCov(rpc_temp);
    }
    if(pn.getParam("angular_vel_covariance",rpc_temp))
    {
        params.angularVelCovariance = setCov(rpc_temp);
    }
    if(pn.getParam("orientation_covariance",rpc_temp))
    {
        params.orientationCovariance = setCov(rpc_temp);
    }

    VnROS vnROS{params};
    vnROS.connect();
    
    ros::spin(); 
    return 0;
}