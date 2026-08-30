#pragma once
#include <cstdint>
#include <array>

namespace Hexapod {

    struct Vec3 {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    };

    struct TelemetryFrame {
        uint32_t timestamp_ms{0};
        Vec3 body_pos{0.0f, 0.0f, 0.0f};      // Rumpfposition im Raum {X, Y, Z}
        Vec3 body_rot{0.0f, 0.0f, 0.0f};      // Körperneigung {Roll, Pitch, Yaw} in Grad
        std::array<Vec3, 6> knees_pos{};      // 3D-Position der 6 Kniegelenke
        std::array<Vec3, 6> feet_pos{};       // 3D-Position der 6 Fußspitzen
    };

} // namespace Hexapod