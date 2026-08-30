#include "kinematics/kinematics.hpp"
#include "core/config.hpp"
#include <cmath>
#include <algorithm>

namespace Hexapod {

    Vector3D Kinematics::transform_to_leg_frame(const Vector3D& world_pos, LegID leg_id) {
        const auto& m = Config::MOUNTS[static_cast<size_t>(leg_id)];
        float dx = world_pos.x - m.x;
        float dy = world_pos.y - m.y;
        float dz = world_pos.z - m.z;

        float yaw_rad = m.yaw_deg * (static_cast<float>(M_PI) / 180.0f);
        float cos_y = std::cos(-yaw_rad);
        float sin_y = std::sin(-yaw_rad);

        return Vector3D{
            .x = dx * cos_y - dy * sin_y,
            .y = dx * sin_y + dy * cos_y,
            .z = dz
        };
    }

    std::optional<JointAngles> Kinematics::calculate_ik(const Vector3D& leg_pos) {
        float x = leg_pos.x;
        float y = leg_pos.y;
        float z = leg_pos.z;

        float rho2 = x * x + y * y;
        if (rho2 < (Config::COXA_OFFSET * Config::COXA_OFFSET)) {
            return std::nullopt;
        }

        float radial = std::sqrt(rho2 - Config::COXA_OFFSET * Config::COXA_OFFSET);
        float coxa = std::atan2(y, x) - std::atan2(Config::COXA_OFFSET, radial);
        float reach = radial - Config::COXA_LEN;
        float d = std::sqrt(reach * reach + z * z);

        if (d > (Config::FEMUR_LEN + Config::TIBIA_LEN) || d < std::abs(Config::FEMUR_LEN - Config::TIBIA_LEN)) {
            return std::nullopt;
        }

        float cos_tibia = (Config::FEMUR_LEN * Config::FEMUR_LEN + Config::TIBIA_LEN * Config::TIBIA_LEN - d * d)
                          / (2.0f * Config::FEMUR_LEN * Config::TIBIA_LEN);
        float tibia = std::acos(std::clamp(cos_tibia, -1.0f, 1.0f)) - static_cast<float>(M_PI);

        float cos_femur = (Config::FEMUR_LEN * Config::FEMUR_LEN + d * d - Config::TIBIA_LEN * Config::TIBIA_LEN)
                          / (2.0f * Config::FEMUR_LEN * d);
        float femur = std::atan2(z, reach) + std::acos(std::clamp(cos_femur, -1.0f, 1.0f));

        return JointAngles{
            .coxa = coxa,
            .femur = femur,
            .tibia = tibia
        };
    }

    Vector3D Kinematics::apply_body_pose(const Vector3D& foot_world_neutral, const BodyPose& pose) {
        // Fuss relativ zum (noch ungekippten) Rumpfzentrum
        float x = foot_world_neutral.x - pose.x;
        float y = foot_world_neutral.y - pose.y;
        float z = foot_world_neutral.z - pose.z;

        float roll  = pose.roll_deg  * (static_cast<float>(M_PI) / 180.0f);
        float pitch = pose.pitch_deg * (static_cast<float>(M_PI) / 180.0f);
        float yaw   = pose.yaw_deg   * (static_cast<float>(M_PI) / 180.0f);

        // Inverse Rotation (Fuss bleibt am Boden, Rumpf kippt relativ dazu weg) -
        // daher werden hier die negativen Winkel angewendet, in Reihenfolge Yaw -> Pitch -> Roll.
        float cy = std::cos(-yaw),   sy = std::sin(-yaw);
        float x1 = x * cy - y * sy;
        float y1 = x * sy + y * cy;
        float z1 = z;

        float cp = std::cos(-pitch), sp = std::sin(-pitch);
        float x2 = x1 * cp + z1 * sp;
        float y2 = y1;
        float z2 = -x1 * sp + z1 * cp;

        float cr = std::cos(-roll),  sr = std::sin(-roll);
        float x3 = x2;
        float y3 = y2 * cr - z2 * sr;
        float z3 = y2 * sr + z2 * cr;

        return Vector3D{x3, y3, z3};
    }

} // namespace Hexapod