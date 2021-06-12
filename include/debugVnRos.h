#pragma once

#include <iostream>
#include <cmath>

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <chrono>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

void debug(CompositeData& cd){
    ROS_INFO("debug");
    stringstream ss;
    if (cd.hasTimeInfo()){
        ss << "Time info";
    }
    if (cd.hasTemperature()){
        ss << ", Temp";
    }
    if (cd.hasWeek()){
        ss << ", Week";
    }
    if (cd.hasVelocityEstimatedEcef()){
        ss << ", vecEcef";
    }
    if (cd.hasAcceleration()){
        ss << ", accel";
    }
    if (cd.hasAccelerationNed()){
        ss << ", accelNed";
    }
    if (cd.hasVelocityEstimatedNed()){
        vec3f vec = cd.velocityEstimatedNed();
        ss << ", velestNED: {" << vec[0] << ", " << vec[1] << ", " << vec[2] << "}";
    }
    if (cd.hasInsStatus()){
        ss << ", insStatus: " <<  cd.insStatus();
    }
    if (cd.hasTimeStartup()){
        uint64_t now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
        ss << ", startup: " << (now - cd.timeStartup()) / 1000000000;
    }
    if (cd.hasQuaternion()){
        ss << ", quaterinon";
    }

    ROS_WARN("%s", ss.str().c_str());
}

void getInsMode(CompositeData& cd){
    if (cd.hasInsStatus()){
        uint16_t status = cd.insStatus();
        uint8_t mode = (status & 0x0003);//(status & 0xC000) >> 14;
        uint8_t GNSSFix = (status & 0x0004) >> 2; //(status & 0x2000) >> 13;
        uint8_t compassActive = (status & 0x0100) >> 8; //(status & 0x0080) >> 7;
        uint8_t compassAiding = (status & 0x0200) >> 9; //(status & 0x0040) >> 6;
        ROS_WARN("mode: %d, GNSSFix: %d, compAct: %d, compAid: %d", 
            mode, GNSSFix, compassActive, compassAiding);        
    }
}

// class for rotation composite data draft...

// class CompositeDataNedToEnu : public vn::sensors::CompositeData {
// public:
//   CompositeDataNedToEnu(vn::sensors::CompositeData cd, bool nedToEnu = false);
//   CompositeDataNedToEnu(vn::protocol::uart::Packet& p, bool nedToEnu = false);
//   void setNedToEnu(bool enu);
//   bool getNedToEnu() const;

  

// private:
//   bool nedToEnu;
// };

// CompositeDataNedToEnu::CompositeDataNedToEnu(CompositeData cd, bool nedToEnu) :
//     CompositeData{cd}, nedToEnu{nedToEnu} {}

// CompositeDataNedToEnu::CompositeDataNedToEnu(vn::protocol::uart::Packet& p, bool nedToEnu) :
//     CompositeData{CompositeData::parse(p)}, nedToEnu{nedToEnu} {}

// void CompositeDataNedToEnu::setNedToEnu(bool enu) {
//     nedToEnu = enu;
// }

// bool CompositeDataNedToEnu::getNedToEnu() const {
//     return nedToEnu;
// }


