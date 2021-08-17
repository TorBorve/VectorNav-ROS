#include "utilities.h"

namespace vnRos {
namespace utilities {

    InsStatus::InsStatus(uint16_t status) :
        mode{(uint8_t)(status & 0x0003)}, 
        GNSSFix{(bool)((status & 0x0004) >> 2)},
        compassAiding{(bool)((status & 0x0100) >> 8)},
        compassActive{(bool)((status & 0x0200) >> 9)}
    {}

    InsStatus InsStatus::parse(uint16_t status){
        return InsStatus{status};
    }

    std::ostream& operator<<(std::ostream& os, const InsStatus& status){
        os << "Mode: " << static_cast<int>(status.mode) << ", GNSSFix: " << status.GNSSFix;
        os << ", Compass Active: " << status.compassActive << ", Compass Aiding: " << status.compassAiding;
        return os;
    }

    bool InsStatus::isOk() const {
        return mode == 2 && GNSSFix && compassActive && compassAiding;
    }

}
}