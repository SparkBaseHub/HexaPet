#pragma once
#include <cstdint>

namespace Hexapod {

    enum class LegID : uint8_t {
        FrontRight = 0,
        MiddleRight = 1,
        RearRight = 2,
        RearLeft = 3,
        MiddleLeft = 4,
        FrontLeft = 5
    };

    struct Vector3D {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    };

    struct JointAngles {
        float coxa{0.0f};   // Grad
        float femur{0.0f};  // Grad
        float tibia{0.0f};  // Grad
    };

    // Pose des Roboter-Rumpfes relativ zur neutralen Standflaeche.
    // Wird spaeter aus ToF-Bodenabstand (Roll/Pitch) und ggf. der AI-Kamera
    // (Yaw-Korrektur) befuellt; bislang nur strukturell vorbereitet.
    struct BodyPose {
        float x{0.0f};         // mm, Translation Rumpfzentrum
        float y{0.0f};         // mm
        float z{0.0f};         // mm, positiv = Rumpf anheben
        float roll_deg{0.0f};  // Rotation um X (Kippen seitlich)
        float pitch_deg{0.0f}; // Rotation um Y (Kippen vor/zurueck)
        float yaw_deg{0.0f};   // Rotation um Z
    };

} // namespace Hexapod