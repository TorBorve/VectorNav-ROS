#pragma once

#include <iostream>
#include <stdint.h>

#include "vectornav/InsStatus.h"

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
}