#pragma once
#include "core/config.hpp"
#include "core/types.hpp"
#include "core/pose_types.hpp"
#include "drivers/servo_driver.hpp"
#include "kinematics/kinematics.hpp"
#include <array>
#include <cmath>
#include <cstring>

namespace Hexapod {

    // Omnidirektionaler Tripod-Gait auf Basis von 3D-Fusspunkt-Zielen im
    // Welt-/Rumpf-Frame. Ersetzt HexapodController: Uebersetzt Geschwindigkeit
    // (x/y, mm/s) + Rotation (deg/s) in Fusstrajektorien, die ueber Kinematics
    // in Gelenkwinkel und ueber ServoDriver in Servo-Kommandos umgesetzt werden.
    //
    // Unterschied zur alten HexapodController-Gangberechnung: Translation UND
    // Rotation wirken gleichzeitig (echtes Omnidirectional-Walking), weil der
    // Rotationsanteil ueber den tatsaechlichen Radiusvektor jedes Beins zum
    // Rumpfzentrum berechnet wird (Tangentialvektor), nicht nur additiv grob genaehert.
    class TripodGait {
    public:
        // Der bisherige feste Tripod-Gang (Config::DUTY_FACTOR + PHASE_GROUPS)
        // als GaitDefinition ausgedrueckt - Default fuer alle Aufrufer, die kein
        // eigenes Gangmuster angeben (Abwaertskompatibilitaet zu vor der
        // Posen-/Gait-Bibliothek).
        static GaitDefinition default_gait() {
            GaitDefinition g{};
            std::strncpy(g.name, "tripod", sizeof(g.name) - 1);
            g.duty_factor = Config::DUTY_FACTOR;
            for (uint8_t i = 0; i < 6; ++i) {
                g.phase_offsets[i] = Config::PHASE_GROUPS[i] * 0.5f;
            }
            return g;
        }

        explicit TripodGait(ServoDriver& driver) : driver_(driver) {
            compute_neutrals();
        }

        // Aktuelle Rumpf-Pose (Roll/Pitch/Yaw/Hoehe) fuer Leveling - wird spaeter
        // aus ToF-Bodenabstand/IMU gespeist. Default = Identitaet (kein Effekt).
        void set_body_pose(const BodyPose& pose) { body_pose_ = pose; }
        const BodyPose& get_body_pose() const { return body_pose_; }

        // Fuehrt alle 6 Beine in die neutrale Standposition (Phase 0, keine Bewegung).
        void apply_stand(const std::array<float, 6>& coxa_base_deg = Config::COXA_REST_DEG) {
            compute_and_apply(0.0f, Vector3D{0.0f, 0.0f, 0.0f}, 0.0f, coxa_base_deg);
            driver_.update();
        }

        void reset_to_home() { apply_stand(Config::COXA_REST_DEG); }

        // Berechnet fuer eine explizite Gangphase (0..1) und Coxa-Basis alle
        // Gelenkwinkel und sendet sie an den ServoDriver (OHNE driver_.update() -
        // der Aufrufer entscheidet, wann geflusht wird, damit man z.B. beim
        // kontrollierten Stop-Zyklus in main.cpp die letzte Phase noch selbst timen kann).
        // Nutzt den festen Tripod-Gang - siehe compute_and_apply(..., gait) fuer
        // ein beliebiges (z.B. aus der PoseLibrary geladenes) Gangmuster.
        void compute_and_apply(float phase, const Vector3D& world_velocity,
                                float rotation_deg_per_sec, const std::array<float, 6>& coxa_base_deg) {
            static const GaitDefinition kDefault = default_gait();
            compute_and_apply(phase, world_velocity, rotation_deg_per_sec, coxa_base_deg, kDefault);
        }

        // Wie oben, aber mit explizitem Gangmuster (duty_factor + phase_offsets
        // pro Bein) statt des fest einkompilierten Tripod-Gangs - Grundlage fuer
        // Wave/Ripple/Quadruped & Co. aus der JSON-Bibliothek.
        void compute_and_apply(float phase, const Vector3D& world_velocity,
                                float rotation_deg_per_sec, const std::array<float, 6>& coxa_base_deg,
                                const GaitDefinition& gait) {
            for (uint8_t i = 0; i < 6; ++i) {
                auto leg = static_cast<LegID>(i);
                Vector3D world_target = get_foot_position(i, phase, world_velocity, rotation_deg_per_sec,
                                                            gait.duty_factor, gait.phase_offsets[i]);
                world_target = Kinematics::apply_body_pose(world_target, body_pose_);
                Vector3D leg_target = Kinematics::transform_to_leg_frame(world_target, leg);
                auto angles = Kinematics::calculate_ik(leg_target);

                if (angles) {
                    driver_.set_leg_angles(leg, *angles, coxa_base_deg[i]);
                } else {
                    // Unerreichbares Ziel: Bein in sichere Ruhelage statt undefiniertem Sprung.
                    driver_.set_command_angle(i,      Config::TIBIA_REST_DEG);
                    driver_.set_command_angle(i + 6,  Config::FEMUR_REST_DEG);
                    driver_.set_command_angle(i + 12, coxa_base_deg[i]);
                }
            }
        }

        // Reine Zielberechnung - identische IK-Logik wie compute_and_apply(),
        // aber OHNE jeglichen Hardware-Seiteneffekt (kein driver_.set_command_angle()-
        // Aufruf, also auch kein Slew-Limiter-Schritt, keine physische Bewegung).
        // Fuer main.cpp's get_stand_pose(): dort wird nur das ZIEL fuer eine
        // nachfolgende safe_transition_to_pose()-Rampe gebraucht - wuerde diese
        // Berechnung stattdessen ueber compute_and_apply() laufen, haette schon
        // das blosse "Ziel ausrechnen" den Servo einen sichtbaren Schritt bewegt,
        // BEVOR die eigentliche sanfte Rampe ueberhaupt beginnt (Ruck + falscher
        // Zwischenwert als "Ziel" zurueckgegeben - siehe Bugfix-Historie).
        // Nutzt den festen Tripod-Gang - siehe die Ueberladung mit 'gait' fuer
        // ein beliebiges Gangmuster.
        std::array<float, Config::NUM_SERVOS> compute_command_angles(
                float phase, const Vector3D& world_velocity,
                float rotation_deg_per_sec, const std::array<float, 6>& coxa_base_deg) const {
            static const GaitDefinition kDefault = default_gait();
            return compute_command_angles(phase, world_velocity, rotation_deg_per_sec, coxa_base_deg, kDefault);
        }

        // Wie oben, aber mit explizitem Gangmuster statt des fest einkompilierten
        // Tripod-Gangs.
        std::array<float, Config::NUM_SERVOS> compute_command_angles(
                float phase, const Vector3D& world_velocity,
                float rotation_deg_per_sec, const std::array<float, 6>& coxa_base_deg,
                const GaitDefinition& gait) const {
            std::array<float, Config::NUM_SERVOS> out{};
            for (uint8_t i = 0; i < 6; ++i) {
                auto leg = static_cast<LegID>(i);
                Vector3D world_target = get_foot_position(i, phase, world_velocity, rotation_deg_per_sec,
                                                            gait.duty_factor, gait.phase_offsets[i]);
                world_target = Kinematics::apply_body_pose(world_target, body_pose_);
                Vector3D leg_target = Kinematics::transform_to_leg_frame(world_target, leg);
                auto angles = Kinematics::calculate_ik(leg_target);

                if (angles) {
                    auto cmd = ServoDriver::compute_joint_command_degrees(i, *angles, coxa_base_deg[i]);
                    out[i]      = cmd[0]; // tibia
                    out[i + 6]  = cmd[1]; // femur
                    out[i + 12] = cmd[2]; // coxa
                } else {
                    out[i]      = Config::TIBIA_REST_DEG;
                    out[i + 6]  = Config::FEMUR_REST_DEG;
                    out[i + 12] = coxa_base_deg[i];
                }
            }
            return out;
        }

        // Reine Zielberechnung fuer eine STATISCHE Pose aus der PoseLibrary
        // (kein Gangzyklus, keine Bewegung - ein einzelnes IK-Ziel pro Bein).
        // STANCE-Posen (parametrisch: stance_radius/standing_height/body) werden
        // hier per IK berechnet; RAW-Posen (direkte Servo-Winkel, z.B. 'Blume'-
        // artige Posen) werden 1:1 durchgereicht, ganz ohne IK. Genau wie
        // compute_command_angles() OHNE jeglichen Hardware-Seiteneffekt.
        std::array<float, Config::NUM_SERVOS> compute_static_pose(
                const PoseDefinition& pose, const std::array<float, 6>& coxa_base_deg) const {
            std::array<float, Config::NUM_SERVOS> out{};

            if (pose.type == PoseType::RAW) {
                for (uint8_t i = 0; i < 6; ++i) {
                    out[i]      = pose.tibia_deg[i];
                    out[i + 6]  = pose.femur_deg[i];
                    out[i + 12] = pose.coxa_deg[i];
                }
                return out;
            }

            for (uint8_t i = 0; i < 6; ++i) {
                auto leg = static_cast<LegID>(i);
                float yaw = Config::MOUNTS[i].yaw_deg * (static_cast<float>(M_PI) / 180.0f);
                Vector3D world_target{
                    Config::MOUNTS[i].x + pose.stance_radius * std::cos(yaw),
                    Config::MOUNTS[i].y + pose.stance_radius * std::sin(yaw),
                    pose.standing_height
                };
                world_target = Kinematics::apply_body_pose(world_target, pose.body);
                Vector3D leg_target = Kinematics::transform_to_leg_frame(world_target, leg);
                auto angles = Kinematics::calculate_ik(leg_target);

                if (angles) {
                    auto cmd = ServoDriver::compute_joint_command_degrees(i, *angles, coxa_base_deg[i]);
                    out[i]      = cmd[0];
                    out[i + 6]  = cmd[1];
                    out[i + 12] = cmd[2];
                } else {
                    out[i]      = Config::TIBIA_REST_DEG;
                    out[i + 6]  = Config::FEMUR_REST_DEG;
                    out[i + 12] = coxa_base_deg[i];
                }
            }
            return out;
        }

        // Ein Update-Tick des kontinuierlichen Gangzyklus (fuer den 50-Hz-Loop).
        // gait_phase wird vom Aufrufer gehalten (main.cpp braucht Kontrolle darueber
        // fuer den "sauber auslaufen lassen"-Zustandsautomaten beim Stoppen).
        void update(float phase, const Vector3D& world_velocity, float rotation_deg_per_sec) {
            compute_and_apply(phase, world_velocity, rotation_deg_per_sec, Config::COXA_WALK_DEG);
            driver_.update();
        }

        // Wie oben, aber mit explizitem Gangmuster.
        void update(float phase, const Vector3D& world_velocity, float rotation_deg_per_sec,
                     const GaitDefinition& gait) {
            compute_and_apply(phase, world_velocity, rotation_deg_per_sec, Config::COXA_WALK_DEG, gait);
            driver_.update();
        }

        // Weiche Ueberblendung der Coxa-Basiswinkel (z.B. REST -> WALK beim Losgehen,
        // WALK -> REST beim Anhalten), waehrend die Beine sonst in Stand-Pose bleiben.
        // speed_factor skaliert die Gesamtdauer (>1 = schneller, <1 = langsamer),
        // siehe main.cpp 'speed'-Befehl.
        void transition_coxa_base(const std::array<float, 6>& from, const std::array<float, 6>& to,
                                   float duration_s = 0.4f, uint32_t steps = 20, float speed_factor = 1.0f) {
            if (speed_factor < 0.01f) speed_factor = 0.01f; // Division durch 0/negativ verhindern
            float effective_duration_s = duration_s / speed_factor;
            uint32_t dt_ms = static_cast<uint32_t>((effective_duration_s * 1000.0f) / steps);
            for (uint32_t step = 1; step <= steps; ++step) {
                float t = static_cast<float>(step) / static_cast<float>(steps);
                float smooth_t = (1.0f - std::cos(t * static_cast<float>(M_PI))) * 0.5f;

                std::array<float, 6> cur_base{};
                for (size_t i = 0; i < 6; ++i) {
                    cur_base[i] = from[i] + (to[i] - from[i]) * smooth_t;
                }
                apply_stand(cur_base);
                sleep_ms(dt_ms);
            }
        }

    private:
        ServoDriver& driver_;
        BodyPose body_pose_{};
        std::array<Vector3D, 6> neutrals_{};

        void compute_neutrals() {
            for (size_t i = 0; i < 6; ++i) {
                float yaw = Config::MOUNTS[i].yaw_deg * (static_cast<float>(M_PI) / 180.0f);
                neutrals_[i] = Vector3D{
                    Config::MOUNTS[i].x + Config::STANCE_RADIUS * std::cos(yaw),
                    Config::MOUNTS[i].y + Config::STANCE_RADIUS * std::sin(yaw),
                    Config::STANDING_HEIGHT
                };
            }
        }

        // Liefert die 3D-Fussposition im Welt-/Rumpf-Frame fuer ein Bein bei
        // gegebener globaler Phase, Translationsgeschwindigkeit, Rotationsrate
        // und Gangmuster (duty_factor + phase_offset dieses Beins, 0..1) -
        // generalisiert gegenueber vorher fest verdrahtetem Config::DUTY_FACTOR/
        // PHASE_GROUPS, damit beliebige Gaits (Tripod/Wave/Ripple/...) aus der
        // PoseLibrary hier ankommen koennen.
        Vector3D get_foot_position(uint8_t leg_idx, float global_phase,
                                    const Vector3D& world_velocity, float rotation_deg_per_sec,
                                    float duty_factor, float phase_offset) const {
            const Vector3D& n = neutrals_[leg_idx];

            float stride_x = world_velocity.x * Config::CYCLE_TIME;
            float stride_y = world_velocity.y * Config::CYCLE_TIME;

            // Rotationsanteil: Tangentialvektor am tatsaechlichen Radiusvektor
            // dieses Beins zum Rumpfzentrum (0,0) - macht Translation + Rotation
            // gleichzeitig moeglich (echtes Omnidirectional-Walking).
            if (std::abs(rotation_deg_per_sec) > 0.001f) {
                float rot_rad = (rotation_deg_per_sec * Config::CYCLE_TIME) * (static_cast<float>(M_PI) / 180.0f);
                stride_x += -n.y * rot_rad;
                stride_y +=  n.x * rot_rad;
            }

            float lp = std::fmod(global_phase + phase_offset, 1.0f);
            if (lp < 0.0f) lp += 1.0f;

            if (lp < duty_factor) {
                // Stance-Phase: Fuss am Boden, schiebt den Rumpf voran (linear).
                float progress = 0.5f - (lp / duty_factor);
                return Vector3D{
                    n.x + stride_x * progress,
                    n.y + stride_y * progress,
                    Config::STANDING_HEIGHT
                };
            } else {
                // Swing-Phase: Fuss in der Luft, parabolische Anhebung.
                float s = (lp - duty_factor) / (1.0f - duty_factor);
                return Vector3D{
                    n.x + stride_x * (s - 0.5f),
                    n.y + stride_y * (s - 0.5f),
                    Config::STANDING_HEIGHT + Config::STEP_HEIGHT * 4.0f * s * (1.0f - s)
                };
            }
        }
    };

} // namespace Hexapod