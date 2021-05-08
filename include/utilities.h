#pragma once

#include <iostream>
#include <stdint.h>

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

InsStatus::InsStatus(uint16_t status) :
    mode{(uint8_t)(status & 0x0003)}, 
    GNSSFix{(bool)((status & 0x0004) >> 2)},
    compassActive{(bool)((status & 0x0100) >> 8)},
    compassAiding{(bool)((status & 0x0200) >> 9)}
{}

InsStatus InsStatus::parse(uint16_t status){
    return InsStatus{status};
}

std::ostream& operator<<(std::ostream& os, const InsStatus& status){
    os << "Mode: " << (int)status.mode << ", GNSSFix: " << status.GNSSFix;
    os << ", Compass Active: " << status.compassActive << ", Compass Aiding: " << status.compassAiding;
    return os;
}

bool InsStatus::isOk() const {
    return mode == 2 && GNSSFix && compassActive && compassAiding;
}

}