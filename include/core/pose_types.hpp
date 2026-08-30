#pragma once
#include "core/config.hpp"
#include "core/types.hpp"
#include <array>
#include <cstdint>
#include <cstring>

namespace Hexapod {

    // Zwei Wege, eine Pose zu definieren:
    //
    // STANCE: parametrisch ueber IK - stance_radius (Fussabstand von der Coxa-
    //   Achse), standing_height (Rumpfhoehe) und eine BodyPose (6-DoF-Offset).
    //   Deckt alle symmetrischen Steh-/Orientierungsposen ab (Home, Crouched,
    //   Tall, Wide, alle Pure-Surge/Sway/Heave/Roll/Pitch/Yaw-Posen). Wird ueber
    //   TripodGait::compute_static_pose() in Servo-Winkel umgerechnet.
    //
    // RAW: direkte Servo-Kommandowinkel pro Bein (Grad, 0-270) - fuer Posen, die
    //   sich nicht als symmetrische IK-Standflaeche ausdruecken lassen (z.B.
    //   Compact/Tuck, Sleep, spaeter Manipulations-Posen). Genau das Prinzip,
    //   das die bestehende 'Blume'-Pose in main.cpp schon (hartkodiert) nutzt.
    enum class PoseType : uint8_t { STANCE, RAW };

    struct PoseDefinition {
        char name[24]{};
        PoseType type{PoseType::STANCE};

        // --- STANCE-Felder ---
        float stance_radius{Config::STANCE_RADIUS};
        float standing_height{Config::STANDING_HEIGHT};
        BodyPose body{};

        // --- RAW-Felder --- (Reihenfolge wie ueberall: 0=VR,1=VL,2=ML,3=HL,4=HR,5=MR)
        std::array<float, 6> tibia_deg{135.0f, 135.0f, 135.0f, 135.0f, 135.0f, 135.0f};
        std::array<float, 6> femur_deg{135.0f, 135.0f, 135.0f, 135.0f, 135.0f, 135.0f};
        std::array<float, 6> coxa_deg {135.0f, 135.0f, 135.0f, 135.0f, 135.0f, 135.0f};

        bool name_is(const char* other) const { return std::strcmp(name, other) == 0; }
    };

    // Ein Gangmuster als Daten statt Hardcodierung: duty_factor (Anteil der
    // Zykluszeit, den ein Bein am Boden verbringt) + phase_offsets pro Bein
    // (0..1, wann innerhalb des Zyklus das jeweilige Bein seine Standphase
    // beginnt). Ersetzt/generalisiert Config::PHASE_GROUPS+DUTY_FACTOR:
    //   Tripod: duty=0.5,  offsets abwechselnd {0, 0.5, 0, 0.5, 0, 0.5}
    //   Wave:   duty=0.833, offsets gleichmaessig verteilt {0,1/6,2/6,3/6,4/6,5/6}
    //   Ripple: duty=0.667, offsets in 3er-Gruppen {0,1/3,2/3,1/3,2/3,0}
    struct GaitDefinition {
        char name[16]{};
        float duty_factor{0.5f};
        std::array<float, 6> phase_offsets{0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};

        bool name_is(const char* other) const { return std::strcmp(name, other) == 0; }
    };

    // Ein Schritt in einer Bewegungssequenz. Drei Arten:
    //   POSE:  faehrt sanft zu einer benannten Pose (safe_transition_to_pose)
    //   GAIT:  laesst den Gangzyklus fuer duration_s Sekunden mit fester
    //          Geschwindigkeit/Rotation laufen (Wippen, Schieben, Tanz-Moves)
    //   ORBIT: kontinuierliche Kreisbewegung des Rumpfes (Roll+Pitch als
    //          Sinus/Cosinus ueber die Zeit, KEINE Zwischenziele/Bremspunkte -
    //          im Unterschied zu einer Kette von POSE-Schritten entlang
    //          derselben Kreisbahn, die an jedem Wegpunkt kurz auf Geschwindigkeit
    //          Null abbremst und dadurch "einzelbewegungshaft" wirkt).
    // repeat wiederholt DIESEN einzelnen Schritt entsprechend oft (z.B. 4x
    // hin-und-her-wippen ohne 8 Schritte einzeln auflisten zu muessen).
    enum class MotionStepType : uint8_t { POSE, GAIT, ORBIT };

    struct MotionStep {
        MotionStepType type{MotionStepType::POSE};
        float duration_s{1.0f};
        uint16_t repeat{1};

        // --- POSE-Felder ---
        char pose_name[24]{};

        // --- GAIT-Felder --- (gait_name leer = aktuell aktiver Gait nutzen)
        char gait_name[16]{};
        float velocity_x{0.0f};          // mm/s, Welt-X (vorwaerts)
        float velocity_y{0.0f};          // mm/s, Welt-Y (seitwaerts)
        float rotation_deg_per_sec{0.0f};

        // Optionale Rumpfneigung WAEHREND des GAIT-Schritts (Grad) - fuer
        // fliessendere Bewegungen, bei denen sich der Koerper in die Bewegung
        // reinlegt (wie ein Taenzer/Skater in der Kurve), statt nur die Beine
        // zu bewegen. 0/0 = kein Effekt (Default, wie bisher).
        float lean_roll_deg{0.0f};
        float lean_pitch_deg{0.0f};

        // --- ORBIT-Felder ---
        // Amplitude (Grad) der kreisenden Roll/Pitch-Bewegung, wie viele volle
        // Umdrehungen ueber duration_s, und Drehrichtung. Start/Ende werden
        // automatisch weich ein-/ausgeblendet (kein manuelles Ramping noetig).
        float orbit_amplitude_deg{12.0f};
        float orbit_revolutions{1.0f};
        bool orbit_clockwise{false};
    };

    struct MotionDefinition {
        char name[24]{};
        static constexpr size_t MAX_STEPS = 12;
        std::array<MotionStep, MAX_STEPS> steps{};
        size_t step_count{0};

        bool name_is(const char* other) const { return std::strcmp(name, other) == 0; }
    };

} // namespace Hexapod