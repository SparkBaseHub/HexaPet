#pragma once
#include "pico/sync.h"
#include <cstdint>
#include <cstring>

namespace Hexapod {

    enum class RobotMode : uint8_t {
        STAND,
        WALK,
        BLUME,
        OFF
    };

    // Laufrichtung fuer den WALK-Modus - ersetzt den vorher fest hartkodierten
    // Vorwaerts-Vektor. Seitwaerts nutzt den y-Anteil des Geschwindigkeitsvektors,
    // genau wie die Rotation den z-Anteil (Yaw) nutzt - beides war architektonisch
    // schon vorbereitet (Omnidirectional-Gait), nur nie ueber einen Befehl angesteuert.
    enum class WalkDirection : uint8_t { FORWARD, BACKWARD, LEFT, RIGHT };

    struct SharedState {
        critical_section_t crit_sec;
        RobotMode requested_mode{RobotMode::STAND};
        bool sleep_requested{false};
        bool recording{false};
        bool diag_requested{false};
        bool calzero_requested{false};

        // Fuer 'pose <name>' / 'gait <name>' / 'motion <name>' (Posen-/Gait-
        // Bibliothek, siehe pose_library.hpp): Core 1 traegt hier nur den
        // GEWUENSCHTEN NAMEN ein, Core 0 sucht ihn in der PoseLibrary nach und
        // fuehrt die eigentliche Bewegung aus (kein Hardware-Zugriff auf Core 1).
        bool pose_requested{false};
        char pose_request_name[24]{};
        bool gait_requested{false};
        char gait_request_name[16]{};
        bool motion_requested{false};
        char motion_request_name[24]{};

        // 'walk forward'/'backward'/'left'/'right': WalkDirection wird direkt
        // (nicht ueber ein "request/take"-Einmal-Flag) gehalten, da der WALK-Loop
        // sie kontinuierlich bei jedem Tick liest, nicht nur einmalig abholt.
        WalkDirection walk_direction{WalkDirection::FORWARD};

        // 'turn <winkel>': einmalige Drehanfrage um einen Zielwinkel (Grad,
        // positiv = im Gegenuhrzeigersinn von oben, wie ueberall sonst in der
        // Kinematik). Gleiches "request/take"-Einmal-Prinzip wie pose/gait/motion.
        bool turn_requested{false};
        float turn_angle_deg{0.0f};

        // 'speed <faktor>': globaler Geschwindigkeits-Multiplikator fuer ALLE
        // Bewegungen (Gangzyklus-Tempo bei walk/turn, Dauer aller Posen-
        // Uebergaenge). 1.0 = normal, >1 = schneller, <1 = langsamer.
        bool speed_requested{false};
        float speed_factor_request{1.0f};

        SharedState() {
            critical_section_init(&crit_sec);
        }

        void request_mode(RobotMode mode) {
            critical_section_enter_blocking(&crit_sec);
            requested_mode = mode;
            critical_section_exit(&crit_sec);
        }

        RobotMode get_requested_mode() {
            critical_section_enter_blocking(&crit_sec);
            RobotMode m = requested_mode;
            critical_section_exit(&crit_sec);
            return m;
        }

        void set_sleep_request(bool v) {
            critical_section_enter_blocking(&crit_sec);
            sleep_requested = v;
            critical_section_exit(&crit_sec);
        }

        bool get_sleep_request() {
            critical_section_enter_blocking(&crit_sec);
            bool v = sleep_requested;
            critical_section_exit(&crit_sec);
            return v;
        }

        void set_recording(bool v) {
            critical_section_enter_blocking(&crit_sec);
            recording = v;
            critical_section_exit(&crit_sec);
        }

        bool get_recording() {
            critical_section_enter_blocking(&crit_sec);
            bool v = recording;
            critical_section_exit(&crit_sec);
            return v;
        }

        void set_diag_request(bool v) {
            critical_section_enter_blocking(&crit_sec);
            diag_requested = v;
            critical_section_exit(&crit_sec);
        }

        bool get_diag_request() {
            critical_section_enter_blocking(&crit_sec);
            bool v = diag_requested;
            critical_section_exit(&crit_sec);
            return v;
        }

        void set_calzero_request(bool v) {
            critical_section_enter_blocking(&crit_sec);
            calzero_requested = v;
            critical_section_exit(&crit_sec);
        }

        bool get_calzero_request() {
            critical_section_enter_blocking(&crit_sec);
            bool v = calzero_requested;
            critical_section_exit(&crit_sec);
            return v;
        }

        // name darf NICHT laenger als (Puffergroesse - 1) sein - wird sonst
        // stillschweigend abgeschnitten (kein Puffer-Overflow, aber ggf. ein
        // Name, der nicht mehr matcht - bewusste, einfache Grenze statt
        // Fehlerbehandlung fuer diesen internen Kommandokanal).
        void request_pose(const char* name) {
            critical_section_enter_blocking(&crit_sec);
            std::strncpy(pose_request_name, name, sizeof(pose_request_name) - 1);
            pose_request_name[sizeof(pose_request_name) - 1] = '\0';
            pose_requested = true;
            critical_section_exit(&crit_sec);
        }

        // Gibt true zurueck und kopiert den angeforderten Namen nach out (falls
        // eine Anfrage vorliegt) - loescht danach das Anfrage-Flag (einmalig
        // abgeholt, wie bei diag/calzero).
        bool take_pose_request(char* out, size_t out_size) {
            critical_section_enter_blocking(&crit_sec);
            bool v = pose_requested;
            if (v) {
                std::strncpy(out, pose_request_name, out_size - 1);
                out[out_size - 1] = '\0';
                pose_requested = false;
            }
            critical_section_exit(&crit_sec);
            return v;
        }

        void request_gait(const char* name) {
            critical_section_enter_blocking(&crit_sec);
            std::strncpy(gait_request_name, name, sizeof(gait_request_name) - 1);
            gait_request_name[sizeof(gait_request_name) - 1] = '\0';
            gait_requested = true;
            critical_section_exit(&crit_sec);
        }

        bool take_gait_request(char* out, size_t out_size) {
            critical_section_enter_blocking(&crit_sec);
            bool v = gait_requested;
            if (v) {
                std::strncpy(out, gait_request_name, out_size - 1);
                out[out_size - 1] = '\0';
                gait_requested = false;
            }
            critical_section_exit(&crit_sec);
            return v;
        }

        void request_motion(const char* name) {
            critical_section_enter_blocking(&crit_sec);
            std::strncpy(motion_request_name, name, sizeof(motion_request_name) - 1);
            motion_request_name[sizeof(motion_request_name) - 1] = '\0';
            motion_requested = true;
            critical_section_exit(&crit_sec);
        }

        bool take_motion_request(char* out, size_t out_size) {
            critical_section_enter_blocking(&crit_sec);
            bool v = motion_requested;
            if (v) {
                std::strncpy(out, motion_request_name, out_size - 1);
                out[out_size - 1] = '\0';
                motion_requested = false;
            }
            critical_section_exit(&crit_sec);
            return v;
        }

        void set_walk_direction(WalkDirection d) {
            critical_section_enter_blocking(&crit_sec);
            walk_direction = d;
            critical_section_exit(&crit_sec);
        }

        WalkDirection get_walk_direction() {
            critical_section_enter_blocking(&crit_sec);
            WalkDirection d = walk_direction;
            critical_section_exit(&crit_sec);
            return d;
        }

        void request_turn(float angle_deg) {
            critical_section_enter_blocking(&crit_sec);
            turn_angle_deg = angle_deg;
            turn_requested = true;
            critical_section_exit(&crit_sec);
        }

        bool take_turn_request(float& out_angle_deg) {
            critical_section_enter_blocking(&crit_sec);
            bool v = turn_requested;
            if (v) {
                out_angle_deg = turn_angle_deg;
                turn_requested = false;
            }
            critical_section_exit(&crit_sec);
            return v;
        }

        void request_speed(float factor) {
            critical_section_enter_blocking(&crit_sec);
            speed_factor_request = factor;
            speed_requested = true;
            critical_section_exit(&crit_sec);
        }

        bool take_speed_request(float& out_factor) {
            critical_section_enter_blocking(&crit_sec);
            bool v = speed_requested;
            if (v) {
                out_factor = speed_factor_request;
                speed_requested = false;
            }
            critical_section_exit(&crit_sec);
            return v;
        }
    };

} // namespace Hexapod