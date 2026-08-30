#pragma once
#include "core/types.hpp"
#include "core/config.hpp"
#include <cmath>
#include <optional>

namespace Hexapod {

    class Kinematics {
    public:
        // Rechnet Zielkoordinate im Bein-System in Gelenkwinkel um
        static std::optional<JointAngles> calculate_ik(const Vector3D& target);

        // Transformiert Welt-/Chassis-Schrittvektor in das lokale Bein-System
        static Vector3D transform_to_leg_frame(const Vector3D& world_vec, LegID leg_id);

        // Wendet eine Rumpf-Pose (Translation + Roll/Pitch/Yaw) auf eine Fusspunkt-
        // Zielposition an, die im neutralen Welt-Frame (Rumpf level, Ursprung im
        // Rumpfzentrum) angegeben ist. Ergebnis ist die Fussposition relativ zum
        // (jetzt gekippten/verschobenen) Rumpf - so, wie sie anschliessend ganz
        // normal per transform_to_leg_frame() + calculate_ik() weiterverarbeitet wird.
        //
        // Vorzeichenlogik: Wenn der Rumpf um +roll kippt, muss der Fuss (der ja am
        // Boden bleibt) relativ zum Rumpf die Gegenrotation erfahren - daher wird
        // hier mit der inversen Rotation gearbeitet.
        static Vector3D apply_body_pose(const Vector3D& foot_world_neutral, const BodyPose& pose);
    };

} // namespace Hexapod