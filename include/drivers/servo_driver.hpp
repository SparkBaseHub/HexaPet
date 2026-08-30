#pragma once
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "core/config.hpp"
#include "core/types.hpp"
#include <algorithm>
#include <array>

namespace Hexapod {

    // Software-Bitbang-Ansteuerung fuer alle 18 Servos ueber GPIO0-17.
    //
    // WICHTIG: Auf dem RP2040 gibt es nur 8 PWM-Slices x 2 Kanaele (16 Kanaele).
    // Die GPIO->Slice/Kanal-Zuordnung wiederholt sich alle 16 Pins
    // (slice = gpio % 8, channel = gpio % 2), d.h. bei 18 Servos auf GPIO0-17
    // kollidieren z.B. GPIO1 und GPIO17 auf demselben PWM-Kanal. Hardware-PWM
    // (pwm_set_chan_level) funktioniert daher NICHT fuer alle 18 Kanaele
    // gleichzeitig - deshalb bewusst Software-Bitbang mit sortierten Pulsen.
    //
    // Pin-Layout (wie hardwareseitig verifiziert):
    //   Tibia:  GPIO 0-5   (Index 0-5,  = Bein-Index)
    //   Femur:  GPIO 6-11  (Index 6-11, = Bein-Index + 6)
    //   Coxa:   GPIO 12-17 (Index 12-17,= Bein-Index + 12)
    class ServoDriver {
    public:
        ServoDriver() { command_angles_.fill(135.0f); }

        void init() {
            for (uint8_t pin = 0; pin < Config::NUM_SERVOS; ++pin) {
                gpio_init(pin);
                gpio_set_dir(pin, GPIO_OUT);
                gpio_put(pin, 0);
            }
        }

        // Setzt den rohen Servo-Kommandowinkel (0-270 Grad Servospanne) fuer
        // einen einzelnen Kanal (0-17, siehe Pin-Layout oben). Wird sowohl fuer
        // IK-Ausgaben als auch fuer direkte Posen (Blume, sanfte Uebergaenge) genutzt.
        //
        // SICHERHEIT: Die tatsaechliche Aenderung wird hart auf
        // Config::MAX_SLEW_DEG_PER_SEC begrenzt (siehe Kommentar dort) - JEDER
        // Aufrufer, auch ein fehlerhafter, kann einen Servo dadurch nie abrupt
        // mit Vollkraft an eine weit entfernte Position springen lassen. Das ist
        // die unterste, immer aktive Sicherheitsebene; sauberes Code-seitiges
        // Rampen (z.B. safe_transition_to_pose in main.cpp) bleibt trotzdem
        // wichtig, damit das tatsaechliche Ziel auch zuverlaessig erreicht wird
        // statt an der Sicherheitsgrenze haengenzubleiben.
        void set_command_angle(uint8_t servo_idx, float angle_deg) {
            if (servo_idx >= Config::NUM_SERVOS) return;
            float requested = std::clamp(angle_deg, 0.0f, 270.0f);

            uint64_t now = time_us_64();
            float elapsed_ms;
            if (last_update_us_[servo_idx] == 0) {
                // Allererster Befehl fuer diesen Kanal (z.B. direkt nach dem Booten) -
                // konservativ nur minimales Bewegungsbudget zulassen, NICHT die
                // gesamte seit Objekterzeugung vergangene Zeit anrechnen.
                elapsed_ms = Config::MAX_SLEW_IDLE_CREDIT_MS;
            } else {
                elapsed_ms = std::min(
                    static_cast<float>(now - last_update_us_[servo_idx]) / 1000.0f,
                    Config::MAX_SLEW_IDLE_CREDIT_MS
                );
            }
            last_update_us_[servo_idx] = now;

            float max_step = Config::MAX_SLEW_DEG_PER_SEC * (elapsed_ms / 1000.0f);
            float current = command_angles_[servo_idx];
            float delta = requested - current;
            float limited = requested;
            if (delta > max_step) limited = current + max_step;
            else if (delta < -max_step) limited = current - max_step;

            command_angles_[servo_idx] = limited;
            pulses_us_[servo_idx] = static_cast<uint16_t>(500.0f + (limited / 270.0f) * 2000.0f);
        }

        // Reine Umrechnung IK-Ergebnis (Radiant, im Bein-Frame) -> Servo-
        // Kommandowinkel (Grad) unter Beruecksichtigung von Ruhelage/Offset/
        // Drehrichtung - OHNE jeglichen Hardware-Seiteneffekt (kein set_command_angle-
        // Aufruf!). Rueckgabe: {tibia_deg, femur_deg, coxa_deg}.
        //
        // WICHTIG: Diese Funktion existiert extra getrennt von set_leg_angles(),
        // damit eine reine "was WAERE der Zielwinkel"-Berechnung (z.B. fuer
        // get_stand_pose() in main.cpp, die nur als Rampen-Ziel dient) NIEMALS
        // selbst schon physisch bewegt. Fruehere Bugs (sichtbarer Ruck + falscher
        // Endwinkel bei Stand-Uebergaengen) kamen genau daher, dass die Zielberechnung
        // versehentlich ueber den echten (slew-limitierten) Hardware-Pfad lief.
        static std::array<float, 3> compute_joint_command_degrees(size_t leg_idx, const JointAngles& angles,
                                                                    float coxa_base_deg) {
            float coxa_deg  = coxa_base_deg + Config::COXA_ZERO_OFFSET_DEG[leg_idx]
                             + rad_to_deg(angles.coxa)  * Config::DIR_COXA[leg_idx];
            float femur_deg = Config::FEMUR_REST_DEG + Config::FEMUR_ZERO_OFFSET_DEG[leg_idx]
                             + rad_to_deg(angles.femur) * Config::DIR_FEMUR[leg_idx];
            float tibia_deg = Config::TIBIA_REST_DEG + Config::TIBIA_ZERO_OFFSET_DEG[leg_idx]
                             + rad_to_deg(angles.tibia) * Config::DIR_TIBIA[leg_idx];
            return {tibia_deg, femur_deg, coxa_deg};
        }

        // Bequemer Einstieg fuer die Kinematik: rechnet IK-Ergebnis um (siehe
        // compute_joint_command_degrees) und setzt die drei zugehoerigen Kanaele
        // WIRKLICH (physischer Hardware-Befehl, ueber den slew-limitierten Pfad).
        // coxa_base_deg kommt von aussen (COXA_REST_DEG/COXA_WALK_DEG bzw. eine
        // Ueberblendung dazwischen), weil das je nach Gangphase/Modus variiert.
        void set_leg_angles(LegID leg, const JointAngles& angles, float coxa_base_deg) {
            size_t idx = static_cast<size_t>(leg);
            auto cmd = compute_joint_command_degrees(idx, angles, coxa_base_deg);

            set_command_angle(static_cast<uint8_t>(idx),      cmd[0]); // tibia
            set_command_angle(static_cast<uint8_t>(idx + 6),  cmd[1]); // femur
            set_command_angle(static_cast<uint8_t>(idx + 12), cmd[2]); // coxa
        }

        // Schickt den aktuell gesetzten Frame per Bitbang an alle aktiven Kanaele.
        void update() {
            flush_frame();
        }

        // Momentaufnahme aller 18 Kommandowinkel - fuer PoseStorage (Flash) und
        // sanfte Interpolationen (z.B. safe_transition_to_pose in main.cpp).
        const std::array<float, Config::NUM_SERVOS>& get_command_angles() const {
            return command_angles_;
        }

        // Setzt den internen "Ist-Zustand" DIREKT, OHNE Slew-Limiter (kein
        // physischer Befehl an die Hardware!). Nur fuer den Boot-Vorgang gedacht:
        // dort wird eine angenommene/geladene Startpose vorgegeben, BEVOR die
        // erste echte Rampe (safe_transition_to_pose) beginnt - diese Annahme
        // darf nicht als "weiter Sprung" durch den Limiter gebremst werden,
        // sondern muss als legitimer Ausgangspunkt gelten. Setzt last_update_us_
        // pro Kanal zurueck auf 0, damit der naechste ECHTE set_command_angle()-
        // Aufruf korrekt als "erster Kontakt seit dieser Annahme" behandelt wird
        // (greift dann wieder ganz normal der ueber MAX_SLEW_IDLE_CREDIT_MS
        // begrenzte Sicherheitsschritt).
        void seed_current_angles(const std::array<float, Config::NUM_SERVOS>& angles) {
            for (uint8_t i = 0; i < Config::NUM_SERVOS; ++i) {
                float clamped = std::clamp(angles[i], 0.0f, 270.0f);
                command_angles_[i] = clamped;
                pulses_us_[i] = static_cast<uint16_t>(500.0f + (clamped / 270.0f) * 2000.0f);
                last_update_us_[i] = 0;
            }
        }

        void disable_all() {
            for (uint8_t pin = 0; pin < Config::NUM_SERVOS; ++pin) {
                pulses_us_[pin] = 0;
                gpio_put(pin, 0);
            }
        }

    private:
        std::array<uint16_t, Config::NUM_SERVOS> pulses_us_{};
        std::array<float, Config::NUM_SERVOS> command_angles_{};
        std::array<uint64_t, Config::NUM_SERVOS> last_update_us_{}; // 0 = "noch nie gesetzt"

        static float rad_to_deg(float rad) {
            return rad * (180.0f / static_cast<float>(M_PI));
        }

        void flush_frame() {
            struct PulseEvent {
                uint8_t pin;
                uint16_t pulse_us;
            };
            std::array<PulseEvent, Config::NUM_SERVOS> events{};
            uint32_t active_mask = 0;

            for (uint8_t i = 0; i < Config::NUM_SERVOS; ++i) {
                events[i] = {i, pulses_us_[i]};
                if (pulses_us_[i] > 0) {
                    active_mask |= (1u << i);
                }
            }

            if (active_mask == 0) return;

            std::sort(events.begin(), events.end(), [](const PulseEvent& a, const PulseEvent& b) {
                return a.pulse_us < b.pulse_us;
            });

            gpio_set_mask(active_mask);

            uint64_t start_time = time_us_64();
            for (const auto& ev : events) {
                if (ev.pulse_us > 0) {
                    while ((time_us_64() - start_time) < ev.pulse_us) {
                        tight_loop_contents();
                    }
                    gpio_clr_mask(1u << ev.pin);
                }
            }
        }
    };

} // namespace Hexapod