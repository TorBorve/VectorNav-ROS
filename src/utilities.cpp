#include "utilities.h"

namespace vnRos {
namespace utilities {

InsStatus::InsStatus(uint16_t status) : mode{(uint8_t)(status & 0x0003)},
                                        GNSSFix{(bool)((status & 0x0004) >> 2)},
                                        compassAiding{(bool)((status & 0x0100) >> 8)},
                                        compassActive{(bool)((status & 0x0200) >> 9)} {}

InsStatus InsStatus::parse(uint16_t status) {
    return InsStatus{status};
}

std::ostream& operator<<(std::ostream& os, const InsStatus& status) {
    os << "Mode: " << static_cast<int>(status.mode) << ", GNSSFix: " << status.GNSSFix;
    os << ", Compass Active: " << status.compassActive << ", Compass Aiding: " << status.compassAiding;
    return os;
}

bool InsStatus::isOk() const {
    return mode == 2 && GNSSFix && compassActive && compassAiding;
}

vn::math::vec3d ecef2enu(const vn::math::vec3d& pos) {
    // vn::math::vec3d dragvollPos = {2.81436e6, 518433, 5.68116e6};
    // constexpr double phi = 63.419*M_PI/180; // lat
    // constexpr double lambda = 10.4375*M_PI/180; // lon
    // vn::math::vec3d dragvollPos = {2.81453e+06, 516524, 5.68105e+06};
    // constexpr double phi = 63.42 * M_PI / 180.0;
    // constexpr double lambda = 10.3992 * M_PI / 180.0;
    vn::math::vec3d dragvollPos = {3142576.652179022785276, 341592.127677080105059, 5521194.435504955239594};
    constexpr double phi = 60.372931647959632 * M_PI / 180.0;
    constexpr double lambda = 6.203586811060537 * M_PI / 180.0;

    constexpr double sin_lam = sin(lambda);
    constexpr double cos_lam = cos(lambda);
    constexpr double sin_phi = sin(phi);
    constexpr double cos_phi = cos(phi);

    vn::math::vec3d posDiff = pos - dragvollPos;

    vn::math::vec3d enu;
    enu[0] = -sin_lam * posDiff[0] + cos_lam * posDiff[1];
    enu[1] = -sin_phi * cos_lam * posDiff[0] - sin_phi * sin_lam * posDiff[1] + cos_phi * sin_lam * posDiff[2];
    enu[2] = cos_phi * posDiff[1] + sin_phi * posDiff[2];
    enu[2] = 0;
    return enu;
}

}  // namespace utilities
}  // namespace vnRos