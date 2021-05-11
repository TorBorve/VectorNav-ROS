#pragma once

#include <iostream>
#include <stdint.h>

#include <vn/vector.h>

#include "vectornav/InsStatus.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

namespace utilities {

struct InsStatus {
    uint8_t mode;
    bool GNSSFix;
    bool compassActive;
    bool compassAiding;
    InsStatus() = default;
    InsStatus(uint16_t status);
    bool isOk() const;
    static InsStatus parse(uint16_t status);
};

std::ostream& operator<<(std::ostream& os, const InsStatus& status);

inline vectornav::InsStatus toMsg(const InsStatus& rhs){
    vectornav::InsStatus lhs;
    lhs.mode = rhs.mode;
    lhs.GNSSFix = rhs.GNSSFix;
    lhs.compassActive = rhs.compassActive;
    lhs.compassAiding = rhs.compassAiding;
    return lhs;
}

inline geometry_msgs::Vector3 toMsg(const vn::math::vec3f& rhs){
    geometry_msgs::Vector3 lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
}

/// Convert from vn::math::vec4f to geometry_msgs::msgs::Quaternion
inline geometry_msgs::Quaternion toMsg(const vn::math::vec4f &rhs) {
    geometry_msgs::Quaternion lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    lhs.w = rhs[3];
    return lhs;
  }

/// Convert from vn::math::vec3d to geometry_msgs::msgs::Point
inline geometry_msgs::Point toMsg(const vn::math::vec3d &rhs) {
    geometry_msgs::Point lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }
}