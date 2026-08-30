#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "hardware/xosc.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "core/config.hpp"
#include "core/robot_state.hpp"
#include "core/telemetry.hpp"
#include "drivers/servo_driver.hpp"
#include "drivers/pose_storage.hpp"
#include "drivers/tof_manager.hpp"
#include "drivers/camera_uart.hpp"
#include "drivers/motion_logger.hpp"
#include "kinematics/tripod_gait.hpp"
#include "core/pose_library.hpp"
#include "core/pose_data.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <array>
#include <cmath>
#include <algorithm>

#define UART_RX_PIN 5
constexpr uint USER_BUTTON_PIN = 23;
constexpr float PI_CONST = 3.14159265358979323846f;

Hexapod::ServoDriver* g_servos_ptr = nullptr;
Hexapod::TripodGait* g_gait_ptr = nullptr;
std::array<float, Hexapod::Config::NUM_SERVOS> g_current_angles{};

// Posen-/Gait-/Bewegungsbibliothek (siehe pose_library.hpp fuer das Schema).
// Wird einmalig in main() geladen, danach nur noch gelesen (kein Mutex noetig -
// beide Cores duerfen lesend gleichzeitig zugreifen). g_active_gait ist das
// aktuell per 'gait <name>' gewaehlte Gangmuster fuer den WALK-Loop.
Hexapod::PoseLibrary g_pose_library;
Hexapod::GaitDefinition g_active_gait = Hexapod::TripodGait::default_gait();

// Globaler Geschwindigkeits-Multiplikator (siehe 'speed <faktor>'-Befehl):
// 1.0 = normal, >1 = schneller, <1 = langsamer. Wirkt auf ALLE Bewegungen -
// Gangzyklus-Tempo (walk/turn) UND Dauer aller Posen-/Motion-Uebergaenge
// (safe_transition_to_pose). Nur von Core 0 geschrieben (nach take_speed_request),
// daher als einfache globale Variable ausreichend (kein Mutex noetig).
float g_speed_factor = 1.0f;

// Liefert die Servo-Kommandowinkel der neutralen Standpose - REIN berechnet,
// OHNE Hardware-Seiteneffekt (siehe TripodGait::compute_command_angles()).
// Vorher rief das gait.apply_stand() auf, was selbst schon (slew-limitiert,
// aber ungerampt) physisch bewegte, bevor die eigentliche Rampe ueberhaupt
// begann - das verursachte einen sichtbaren Ruck UND einen falschen
// Zwischenwert als "Ziel" fuer die nachfolgende Rampe.
std::array<float, Hexapod::Config::NUM_SERVOS> get_stand_pose(Hexapod::TripodGait& gait,
                                                                Hexapod::ServoDriver& servos) {
    (void)servos; // Parameter aus Kompatibilitaetsgruenden beibehalten, nicht mehr benoetigt
    return gait.compute_command_angles(0.0f, Hexapod::Vector3D{0.0f, 0.0f, 0.0f}, 0.0f, Hexapod::Config::COXA_REST_DEG);
}

Hexapod::SharedState shared_state;
void core1_entry();

bool get_user_button() {
    return !gpio_get(USER_BUTTON_PIN);
}

// Sanfte 50-Hz-Interpolation. WICHTIG: Startpunkt und Zwischenstaende kommen
// aus servos.get_command_angles() (dem tatsaechlich vom Slew-Limiter verwalteten
// Ist-Zustand), NICHT aus der globalen g_current_angles - sonst laeuft die
// Software-Rampe der tatsaechlichen (ggf. vom Limiter leicht gebremsten)
// Servo-Position auseinander: die Rampe wuerde immer weiter extrapolieren,
// waehrend der echte Servo hinterherhinkt und jeden Schritt gleich stark
// gekappt bekommt - Resultat: sichtbares Zucken beim Start und ein Endzustand,
// der weit vom eigentlichen Ziel entfernt haengen bleibt.
void safe_transition_to_pose(Hexapod::ServoDriver& servos,
                             const std::array<float, Hexapod::Config::NUM_SERVOS>& target_pose,
                             float duration_s = 1.5f) {
    constexpr uint32_t INTERVAL_MS = 20;
    float speed = g_speed_factor < 0.01f ? 0.01f : g_speed_factor; // Division durch 0/negativ verhindern
    float effective_duration_s = duration_s / speed;
    uint32_t total_steps = static_cast<uint32_t>((effective_duration_s * 1000.0f) / INTERVAL_MS);
    if (total_steps == 0) total_steps = 1;

    std::array<float, Hexapod::Config::NUM_SERVOS> start_pose = servos.get_command_angles();

    for (uint32_t step = 1; step <= total_steps; ++step) {
        float t = static_cast<float>(step) / total_steps;
        float smooth_t = (1.0f - std::cos(t * PI_CONST)) * 0.5f;

        for (uint8_t i = 0; i < Hexapod::Config::NUM_SERVOS; ++i) {
            float angle = start_pose[i] + (target_pose[i] - start_pose[i]) * smooth_t;
            servos.set_command_angle(i, angle);
        }
        servos.update();
        g_current_angles = servos.get_command_angles(); // Spiegel fuer PoseStorage/andere Nutzer
        sleep_ms(INTERVAL_MS);
    }
    g_current_angles = servos.get_command_angles(); // tatsaechlicher Endzustand, nicht blind target_pose
}

// Sichere 2-Stufen-Sequenz für die Blume (Verhindert Verhaken am Boden/Rumpf)
void safe_transition_to_blume(Hexapod::ServoDriver& servos, Hexapod::TripodGait& gait) {
    // Stufe 1: Sicherstellen, dass der Roboter im neutralen Stand steht
    safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.0f);

    // Stufe 2: Beine erst anheben/einklappen, dann Coxa ausrichten
    std::array<float, Hexapod::Config::NUM_SERVOS> intermediate_pose = g_current_angles;
    for (uint8_t i = 0; i < 6; ++i) {
        intermediate_pose[i]     = 135.0f; // Tibia entlasten
        intermediate_pose[i + 6] = 90.0f;  // Femur halb anheben
    }
    safe_transition_to_pose(servos, intermediate_pose, 0.8f);

    // Stufe 3: Vollständige Blume einnehmen
    std::array<float, Hexapod::Config::NUM_SERVOS> blume_final{};
    for (uint8_t i = 0; i < 6; ++i) {
        blume_final[i]      = 135.0f; // Tibia
        blume_final[i + 6]  = 45.0f;  // Femur
        blume_final[i + 12] = 135.0f; // Coxa
    }
    safe_transition_to_pose(servos, blume_final, 0.8f);
    Hexapod::PoseStorage::save_pose(g_current_angles, Hexapod::RobotMode::BLUME); // sicherer Checkpoint fuer den naechsten Boot
}

// Liefert den Welt-Geschwindigkeitsvektor fuer eine Laufrichtung (Betrag aus
// Config::WALK_STRIDE_MM). Ersetzt den vorher an zwei Stellen hartkodierten
// Vector3D{35.0f, 0.0f, 0.0f}.
Hexapod::Vector3D get_walk_velocity(Hexapod::WalkDirection dir) {
    using Hexapod::WalkDirection;
    switch (dir) {
        case WalkDirection::FORWARD:  return {Hexapod::Config::WALK_STRIDE_MM, 0.0f, 0.0f};
        case WalkDirection::BACKWARD: return {-Hexapod::Config::WALK_STRIDE_MM, 0.0f, 0.0f};
        case WalkDirection::LEFT:     return {0.0f, Hexapod::Config::WALK_STRIDE_MM, 0.0f};
        case WalkDirection::RIGHT:    return {0.0f, -Hexapod::Config::WALK_STRIDE_MM, 0.0f};
    }
    return {Hexapod::Config::WALK_STRIDE_MM, 0.0f, 0.0f};
}

// Dreht den Roboter um EXAKT angle_deg (positiv = Gegenuhrzeigersinn von oben),
// indem die Zahl der vollen Gangzyklen so gewaehlt wird, dass der Zielwinkel
// nach einer GANZEN Zahl Zyklen erreicht ist (kein Abbruch mitten im Schritt -
// die Fuesse landen exakt wieder in Phase 0). Faehrt vorher optional zur
// 'turn_stance'-Pose (falls in der Bibliothek vorhanden), damit alle Beine mit
// symmetrischer Coxa-Reichweite in die Drehung starten.
void run_turn(Hexapod::ServoDriver& servos, Hexapod::TripodGait& gait, float angle_deg) {
    if (std::abs(angle_deg) < 0.1f) return;

    const Hexapod::PoseDefinition* turn_pose = g_pose_library.find_pose("turn_stance");
    if (turn_pose) {
        auto target = gait.compute_static_pose(*turn_pose, Hexapod::Config::COXA_REST_DEG);
        safe_transition_to_pose(servos, target, 1.0f);
    }

    float abs_angle = std::abs(angle_deg);
    float sign = (angle_deg < 0.0f) ? -1.0f : 1.0f;

    // Die 1x-Grundformel stimmt geometrisch (siehe Bugfix-Historie), aber echtes
    // Drehen auf dem Untergrund ueberschiesst mechanisch (Schwung/Traegheit) -
    // deshalb hier den zu erreichenden Zielwinkel VOR der Zyklenberechnung durch
    // den empirischen TURN_OVERSHOOT_FACTOR teilen. Kein exakter, sondern ein
    // eingemessener Naeherungswert - siehe Kommentar bei der Konstante in config.hpp.
    float corrected_abs_angle = abs_angle / Hexapod::Config::TURN_OVERSHOOT_FACTOR;
    float nominal_per_cycle = Hexapod::Config::TURN_SPEED_DEG_PER_SEC * Hexapod::Config::CYCLE_TIME;
    int cycles = static_cast<int>(std::round(corrected_abs_angle / nominal_per_cycle));
    if (cycles < 1) cycles = 1;
    float rotation_deg_per_sec = sign * (corrected_abs_angle / static_cast<float>(cycles)) / Hexapod::Config::CYCLE_TIME;

    printf("\n[Turn] Drehe %.1f Grad angefordert (korrigiertes Ziel %.1f Grad) in %d Gangzyklus/-zyklen (%.2f Grad/s)...\n",
           angle_deg, sign * corrected_abs_angle, cycles, rotation_deg_per_sec);

    gait.transition_coxa_base(Hexapod::Config::COXA_REST_DEG, Hexapod::Config::COXA_WALK_DEG, 0.4f, 20, g_speed_factor);

    constexpr uint32_t INTERVAL_MS = 20;
    float speed = g_speed_factor < 0.01f ? 0.01f : g_speed_factor;
    for (int c = 0; c < cycles; ++c) {
        float phase = 0.0f;
        while (phase < 1.0f) {
            phase += (0.02f * speed) / Hexapod::Config::CYCLE_TIME;
            gait.compute_and_apply(phase, Hexapod::Vector3D{0.0f, 0.0f, 0.0f}, rotation_deg_per_sec,
                                    Hexapod::Config::COXA_WALK_DEG, g_active_gait);
            servos.update();
            g_current_angles = servos.get_command_angles();
            sleep_ms(INTERVAL_MS);
        }
    }

    gait.transition_coxa_base(Hexapod::Config::COXA_WALK_DEG, Hexapod::Config::COXA_REST_DEG, 0.4f, 20, g_speed_factor);
    safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.0f);
    printf("[Turn] Fertig.\n");
}

// Fuehrt einen GAIT-Schritt einer Bewegungssequenz aus: laesst den Gangzyklus
// fuer step.duration_s Sekunden (ggf. mehrfach bei step.repeat>1) mit fester
// Geschwindigkeit/Rotation laufen - z.B. fuer seitliches Wippen/Schieben
// innerhalb eines Tanz-Moves, statt nur zwischen statischen Posen zu interpolieren.
// gait_name leer = aktuell aktiver Gait (g_active_gait) wird genutzt.
void run_gait_step(Hexapod::ServoDriver& servos, Hexapod::TripodGait& gait, const Hexapod::MotionStep& step) {
    const Hexapod::GaitDefinition* found_gait =
        step.gait_name[0] != '\0' ? g_pose_library.find_gait(step.gait_name) : nullptr;
    const Hexapod::GaitDefinition& use_gait = found_gait ? *found_gait : g_active_gait;

    // Rumpfneigung WAEHREND dieses Schritts (z.B. "in die Kurve legen" bei einer
    // Drehbewegung) - fuer einen fliessenderen Ganzkoerper-Effekt statt nur die
    // Beine zu bewegen. Nach dem Schritt IMMER zuruecksetzen, sonst bliebe der
    // Rumpf fuer alle NACHFOLGENDEN Bewegungen (auch normales Gehen!) geneigt.
    Hexapod::BodyPose lean{};
    lean.roll_deg = step.lean_roll_deg;
    lean.pitch_deg = step.lean_pitch_deg;
    gait.set_body_pose(lean);

    gait.transition_coxa_base(Hexapod::Config::COXA_REST_DEG, Hexapod::Config::COXA_WALK_DEG, 0.3f, 20, g_speed_factor);

    constexpr uint32_t INTERVAL_MS = 20;
    float speed = g_speed_factor < 0.01f ? 0.01f : g_speed_factor;
    Hexapod::Vector3D velocity{step.velocity_x, step.velocity_y, 0.0f};

    for (uint16_t r = 0; r < step.repeat; ++r) {
        float elapsed_s = 0.0f;
        float phase = 0.0f;
        while (elapsed_s < step.duration_s) {
            phase += (0.02f * speed) / Hexapod::Config::CYCLE_TIME;
            if (phase >= 1.0f) phase -= 1.0f;
            gait.compute_and_apply(phase, velocity, step.rotation_deg_per_sec,
                                    Hexapod::Config::COXA_WALK_DEG, use_gait);
            servos.update();
            g_current_angles = servos.get_command_angles();
            sleep_ms(INTERVAL_MS);
            elapsed_s += INTERVAL_MS / 1000.0f;
        }
    }

    gait.transition_coxa_base(Hexapod::Config::COXA_WALK_DEG, Hexapod::Config::COXA_REST_DEG, 0.3f, 20, g_speed_factor);
    gait.set_body_pose(Hexapod::BodyPose{}); // Neigung zuruecksetzen - siehe Kommentar oben
}

// Fuehrt einen ORBIT-Schritt aus: kontinuierliche Kreisbewegung des Rumpfes
// (Roll+Pitch als Sinus/Cosinus ueber die Zeit) - im Unterschied zu einer
// Kette von POSE-Schritten entlang derselben Kreisbahn gibt es hier KEINE
// Zwischenziele/Bremspunkte, die Bewegung ist durchgehend fliessend. Amplitude
// wird an Start (erste ~15%) und Ende (letzte ~15%) weich ein-/ausgeblendet,
// damit der Uebergang aus/in die vorherige/naechste Pose nicht ruckt. Die
// Beine selbst bewegen sich NICHT (keine Schrittbewegung, reine Rumpfneigung -
// wie eine stehende Drehbewegung/"Bauchtanz"-Bewegung).
void run_orbit_step(Hexapod::ServoDriver& servos, Hexapod::TripodGait& gait, const Hexapod::MotionStep& step) {
    if (step.duration_s <= 0.01f) return;

    constexpr uint32_t INTERVAL_MS = 20;
    float speed = g_speed_factor < 0.01f ? 0.01f : g_speed_factor;
    float effective_duration_s = step.duration_s / speed;
    uint32_t total_steps = static_cast<uint32_t>((effective_duration_s * 1000.0f) / INTERVAL_MS);
    if (total_steps < 2) total_steps = 2;

    float dir = step.orbit_clockwise ? -1.0f : 1.0f;
    constexpr float RAMP_FRACTION = 0.15f; // erste/letzte 15% = weiches Ein-/Ausblenden

    for (uint32_t s = 1; s <= total_steps; ++s) {
        float t = static_cast<float>(s) / static_cast<float>(total_steps); // 0..1
        float theta = dir * 2.0f * static_cast<float>(M_PI) * step.orbit_revolutions * t;

        float amp_scale = 1.0f;
        if (t < RAMP_FRACTION) {
            amp_scale = (1.0f - std::cos(t / RAMP_FRACTION * static_cast<float>(M_PI))) * 0.5f;
        } else if (t > 1.0f - RAMP_FRACTION) {
            float t2 = (1.0f - t) / RAMP_FRACTION;
            amp_scale = (1.0f - std::cos(t2 * static_cast<float>(M_PI))) * 0.5f;
        }

        Hexapod::BodyPose pose{};
        pose.roll_deg = step.orbit_amplitude_deg * amp_scale * std::sin(theta);
        pose.pitch_deg = step.orbit_amplitude_deg * amp_scale * std::cos(theta);
        gait.set_body_pose(pose);

        // Reine Standpose (Beine bleiben stehen, keine Schritte) - der
        // Rumpf-Tilt kommt ausschliesslich aus dem gerade gesetzten body_pose_.
        auto target = gait.compute_command_angles(0.0f, Hexapod::Vector3D{0.0f, 0.0f, 0.0f}, 0.0f,
                                                    Hexapod::Config::COXA_REST_DEG);
        for (uint8_t i = 0; i < Hexapod::Config::NUM_SERVOS; ++i) {
            servos.set_command_angle(i, target[i]);
        }
        servos.update();
        g_current_angles = servos.get_command_angles();
        sleep_ms(INTERVAL_MS);
    }

    gait.set_body_pose(Hexapod::BodyPose{}); // Neigung zuruecksetzen
}

void print_help() {
    printf("\n======================================================================\n");
    printf(" HEXAPOD DUAL-CORE ENGINE (GESICHERTE ZUSTANDSMASCHINE)\n");
    printf("======================================================================\n");
    printf(" Button-Steuerung:\n");
    printf("   [User Button GP23] kurz  -> Kontrollierter Toggle: Blume <-> Walk\n");
    printf("   [User Button GP23] 5s halten -> USB-Bootloader-Modus (statt BOOTSEL-Taste)\n");
    printf(" Steuerbefehle:\n");
    printf("   walk             -> Startet Gangzyklus (aktuelle Richtung, Default vorwaerts)\n");
    printf("   walk forward/backward/left/right -> Setzt Richtung UND startet Gangzyklus\n");
    printf("   turn <grad>      -> Dreht exakt um den angegebenen Winkel (z.B. 'turn 45',\n");
    printf("                       'turn -90', 'turn 180'), dreht vorher in 'turn_stance'\n");
    printf("   speed <faktor>   -> Globale Geschwindigkeit fuer ALLE Bewegungen (1.0=normal,\n");
    printf("                       z.B. 2.0=doppelt so schnell, 0.5=halb so schnell)\n");
    printf("   stand            -> Roboter neutral hinstellen\n");
    printf("   blume            -> Pose 'Blume' kollisionsfrei anfahren\n");
    printf("   off              -> Alle 18 Servos stromlos schalten\n");
    printf("   diag             -> Servo-Sollwinkel pro Bein ueber einen Gangzyklus ausgeben\n");
    printf("                       (Roboter auf Staender stellen, Beine frei in der Luft!)\n");
    printf("   calzero          -> Coxa-Nullpunkt-Kalibrierung: faehrt jedes Bein einzeln\n");
    printf("                       in IK-Nullstellung, zum Abgleich mit MOUNTS[i].yaw_deg\n");
    printf("   pose <name>      -> Faehrt sanft zu einer benannten Pose aus der Bibliothek\n");
    printf("   poses            -> Listet alle verfuegbaren Posen\n");
    printf("   gait <name>      -> Waehlt das Gangmuster fuer den naechsten 'walk'\n");
    printf("   gaits            -> Listet alle verfuegbaren Gangmuster\n");
    printf("   motion <name>    -> Spielt eine benannte Bewegungssequenz ab\n");
    printf("   motions          -> Listet alle verfuegbaren Bewegungssequenzen\n");
    printf("   help / ?         -> Dieses Hilfemenue anzeigen\n");
    printf("======================================================================\n\n");
}

// Gibt fuer 5 Phasenpunkte (0.0/0.25/0.5/0.75/1.0) eines Geradeaus-Gangzyklus
// die berechneten Servo-Sollwinkel aller 6 Beine aus. Zum Abgleich gespiegelter
// Beinpaare (VR<->VL, ML<->MR, HL<->HR) bei Kreislauf-Symptomen:
// Roboter auf einen Staender stellen (Beine frei), "diag" eingeben, Ausgabe
// hier posten - Werte je Beinpaar sollten (bis auf's Vorzeichen der Bewegungs-
// richtung) symmetrisch sein. Ein Bein, das deutlich abweicht oder bei 0/270
// haengenbleibt, ist der Uebeltaeter.
void run_leg_diagnostics(Hexapod::ServoDriver& servos, Hexapod::TripodGait& gait) {
    static const char* LEG_NAMES[6] = {"VR", "VL", "ML", "HL", "HR", "MR"};
    printf("\n=== DIAG: Servo-Sollwinkel ueber Gangzyklus (Geradeaus, v=35mm/s) ===\n");
    for (float phase = 0.0f; phase <= 1.0f; phase += 0.25f) {
        gait.compute_and_apply(phase, Hexapod::Vector3D{35.0f, 0.0f, 0.0f}, 0.0f, Hexapod::Config::COXA_WALK_DEG);
        servos.update();
        auto cmd = servos.get_command_angles();
        printf("Phase %.2f:\n", phase);
        for (uint8_t i = 0; i < 6; ++i) {
            printf("  %s (Bein %u): Coxa=%6.1f  Femur=%6.1f  Tibia=%6.1f\n",
                   LEG_NAMES[i], i, cmd[i + 12], cmd[i + 6], cmd[i]);
        }
        sleep_ms(400); // Zeit zum Beobachten/Vergleichen mit der physischen Bewegung
    }
    printf("=== DIAG Ende - zurueck in vorherige Pose ===\n\n");
}

// Coxa-Nullpunkt-Kalibrierung: faehrt jedes Bein EINZELN in die IK-Nullstellung
// (coxa_rad=0, Femur/Tibia=Ruhelage), waehrend die anderen Beine in der aktuellen
// Standpose bleiben. In der Nullstellung MUSS die Coxa exakt in Richtung
// MOUNTS[i].yaw_deg zeigen (0 deg = nach vorne/+X, 90 deg = nach links/+Y usw.,
// gemessen vom Rumpfzentrum aus) - das ist die Grundannahme der Koordinaten-
// transformation. Weicht das sichtbar ab, ist die Differenz (in Servo-Grad, nicht
// IK-Grad!) der Wert, den man in COXA_ZERO_OFFSET_DEG[i] eintraegt (mit Vorzeichen
// passend zu DIR_COXA[i]: wirkt die Korrektur in die "falsche" Richtung, Vorzeichen
// umdrehen). Roboter auf Staender stellen, Beine frei in der Luft!
void run_coxa_zero_calibration(Hexapod::ServoDriver& servos, Hexapod::TripodGait& gait) {
    static const char* LEG_NAMES[6] = {"VR", "VL", "ML", "HL", "HR", "MR"};
    printf("\n=== CALZERO: Coxa-Nullpunkt-Kalibrierung ===\n");
    printf("Fuer jedes Bein: Zeigt die Coxa exakt in Richtung des angegebenen\n");
    printf("MOUNTS[i].yaw_deg (0=vorne/+X, 90=links/+Y, gemessen ab Rumpfmitte)?\n");
    printf("Abweichung in Grad notieren -> Config::COXA_ZERO_OFFSET_DEG[i] eintragen.\n\n");

    // Baseline: alle Beine in Standpose, dann eins nach dem anderen auf Nullstellung.
    gait.apply_stand(Hexapod::Config::COXA_REST_DEG);

    for (uint8_t i = 0; i < 6; ++i) {
        auto leg = static_cast<Hexapod::LegID>(i);
        servos.set_leg_angles(leg, Hexapod::JointAngles{0.0f, 0.0f, 0.0f}, Hexapod::Config::COXA_REST_DEG[i]);
        servos.update();
        printf("Bein %s (Idx %u): erwartete Richtung = %.1f deg (aktuell aktiver Offset: %.1f deg)\n",
               LEG_NAMES[i], i, Hexapod::Config::MOUNTS[i].yaw_deg, Hexapod::Config::COXA_ZERO_OFFSET_DEG[i]);
        sleep_ms(3000);
    }

    gait.apply_stand(Hexapod::Config::COXA_REST_DEG);
    printf("=== CALZERO Ende - zurueck in Standpose ===\n\n");
}

void enter_dormant_mode(Hexapod::ServoDriver& servos, Hexapod::TripodGait& gait) {
    printf("--> [Power] Fahre in Stand und gehe in DORMANT Deep-Sleep...\n");
    safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.0f);
    Hexapod::PoseStorage::save_pose(g_current_angles, Hexapod::RobotMode::STAND);
    sleep_ms(200);
    servos.disable_all();

    stdio_flush();
    multicore_reset_core1();
    gpio_set_dormant_irq_enabled(UART_RX_PIN, GPIO_IRQ_EDGE_FALL, true);
    xosc_dormant();

    xosc_init();
    set_sys_clock_khz(125000, true);
    gpio_set_dormant_irq_enabled(UART_RX_PIN, GPIO_IRQ_EDGE_FALL, false);
    stdio_init_all();

    servos.init();
    safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.0f);
    multicore_launch_core1(core1_entry);

    printf("\n--> [Power] RP2040 aufgewacht! Systeme online.\n");
}

// =============================================================================
// CORE 1: KOMMUNIKATION & BEFEHLE (KEIN DIREKTER HARDWARE-ZUGRIFF)
// =============================================================================
void core1_entry() {
    // Registriert diesen Core als "Lockout Victim" - Voraussetzung dafuer, dass
    // PoseStorage::save_pose() (Core 0) diesen Core waehrend eines Flash-
    // Schreibvorgangs sicher pausieren kann (siehe Kommentar in pose_storage.hpp).
    // MUSS vor jeglicher anderer Arbeit stehen, insbesondere vor dem ersten
    // moeglichen save_pose()-Aufruf von Core 0 aus.
    multicore_lockout_victim_init();

    printf("--> [Core 1] I/O & Terminal Thread aktiv.\n");

    gpio_init(USER_BUTTON_PIN);
    gpio_set_dir(USER_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(USER_BUTTON_PIN);

    Hexapod::CameraUART::init();
    Hexapod::ToFManager tof(i2c0);

    char usb_buf[64];
    size_t usb_idx = 0;

    char uart_buf[64];
    size_t uart_idx = 0;

    bool button_prev_pressed = false;
    uint32_t press_start_ms = 0;
    bool long_press_triggered = false;
    constexpr uint32_t BOOTLOADER_HOLD_MS = 5000;

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Button-Handling mit Flankenerkennung: kurzer Klick = Walk<->Blume-Toggle
        // (wie bisher), 5 Sekunden HALTEN = softwareseitig in den USB-Bootloader
        // wechseln (reset_usb_boot) - ersetzt die physische, schwer zugaengliche
        // BOOTSEL-Taste durch den ohnehin vorhandenen User-Button.
        bool pressed_now = get_user_button();

        if (pressed_now && !button_prev_pressed) {
            // Steigende Flanke: Timer fuer diesen Tastendruck starten
            press_start_ms = now_ms;
            long_press_triggered = false;
        }

        if (pressed_now && !long_press_triggered && (now_ms - press_start_ms >= BOOTLOADER_HOLD_MS)) {
            long_press_triggered = true;
            printf("\n[Button] 5s gehalten -> wechsle in USB-Bootloader-Modus zum Flashen...\n");
            stdio_flush();
            sleep_ms(50);
            reset_usb_boot(0, 0); // kehrt nicht zurueck
        }

        if (!pressed_now && button_prev_pressed) {
            // Fallende Flanke: nur als Kurzklick werten, wenn kein Long-Press ausgeloest wurde
            if (!long_press_triggered && (now_ms - press_start_ms) > 50) {
                Hexapod::RobotMode cur = shared_state.get_requested_mode();
                if (cur == Hexapod::RobotMode::WALK) {
                    printf("\n[Button] Stoppe Lauf sanft -> Sequenz zu 'Blume'...\n");
                    shared_state.request_mode(Hexapod::RobotMode::BLUME);
                } else {
                    printf("\n[Button] Vorbereitung -> Starte 'Walk'...\n");
                    shared_state.request_mode(Hexapod::RobotMode::WALK);
                }
                printf("Befehl: ");
            }
        }
        button_prev_pressed = pressed_now;

        // UART ESP32
        char uc;
        while (Hexapod::CameraUART::read_byte(uc)) {
            if (uc == '\n' || uart_idx >= sizeof(uart_buf) - 1) {
                uart_buf[uart_idx] = '\0';
                if (strcmp(uart_buf, "sleep") == 0 || strstr(uart_buf, "\"cmd\":\"sleep\"")) {
                    shared_state.set_sleep_request(true);
                }
                uart_idx = 0;
            } else if (uc != '\r') {
                uart_buf[uart_idx++] = uc;
            }
        }

        // USB Konsole
        int ch = getchar_timeout_us(1000);
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                if (usb_idx > 0) {
                    usb_buf[usb_idx] = '\0';
                    printf("\n> %s\n", usb_buf);

                    if (strncmp(usb_buf, "walk", 4) == 0) {
                        const char* arg = usb_buf + 4;
                        while (*arg == ' ') ++arg;
                        if (strcmp(arg, "forward") == 0) {
                            shared_state.set_walk_direction(Hexapod::WalkDirection::FORWARD);
                        } else if (strcmp(arg, "backward") == 0) {
                            shared_state.set_walk_direction(Hexapod::WalkDirection::BACKWARD);
                        } else if (strcmp(arg, "left") == 0) {
                            shared_state.set_walk_direction(Hexapod::WalkDirection::LEFT);
                        } else if (strcmp(arg, "right") == 0) {
                            shared_state.set_walk_direction(Hexapod::WalkDirection::RIGHT);
                        }
                        // Leeres arg ("walk" allein) laesst die aktuelle Richtung
                        // unveraendert (z.B. fuer den Button-Toggle, der keine
                        // Richtung mitgibt).
                        shared_state.request_mode(Hexapod::RobotMode::WALK);
                    }
                    else if (strcmp(usb_buf, "stand") == 0) {
                        shared_state.request_mode(Hexapod::RobotMode::STAND);
                    }
                    else if (strcmp(usb_buf, "blume") == 0) {
                        shared_state.request_mode(Hexapod::RobotMode::BLUME);
                    }
                    else if (strcmp(usb_buf, "off") == 0) {
                        shared_state.request_mode(Hexapod::RobotMode::OFF);
                    }
                    else if (strncmp(usb_buf, "turn ", 5) == 0) {
                        char* parse_end = nullptr;
                        float angle = std::strtof(usb_buf + 5, &parse_end);
                        if (parse_end != usb_buf + 5) {
                            shared_state.request_turn(angle);
                        } else {
                            printf("Ungueltiger Winkel - Beispiel: 'turn 45' oder 'turn -90'\n");
                        }
                    }
                    else if (strncmp(usb_buf, "speed ", 6) == 0) {
                        char* parse_end = nullptr;
                        float factor = std::strtof(usb_buf + 6, &parse_end);
                        if (parse_end != usb_buf + 6 && factor > 0.0f) {
                            shared_state.request_speed(factor);
                        } else {
                            printf("Ungueltiger Faktor - Beispiel: 'speed 1.5' oder 'speed 0.5' (muss > 0 sein)\n");
                        }
                    }
                    else if (strcmp(usb_buf, "help") == 0 || strcmp(usb_buf, "?") == 0) {
                        print_help();
                    }
                    else if (strcmp(usb_buf, "diag") == 0) {
                        shared_state.set_diag_request(true);
                    }
                    else if (strcmp(usb_buf, "calzero") == 0) {
                        shared_state.set_calzero_request(true);
                    }
                    else if (strncmp(usb_buf, "pose ", 5) == 0) {
                        shared_state.request_pose(usb_buf + 5);
                    }
                    else if (strcmp(usb_buf, "poses") == 0) {
                        printf("Verfuegbare Posen (%zu):\n", g_pose_library.pose_count());
                        for (size_t i = 0; i < g_pose_library.pose_count(); ++i) {
                            printf("  %s\n", g_pose_library.pose_at(i).name);
                        }
                    }
                    else if (strncmp(usb_buf, "gait ", 5) == 0) {
                        shared_state.request_gait(usb_buf + 5);
                    }
                    else if (strcmp(usb_buf, "gaits") == 0) {
                        printf("Verfuegbare Gangmuster (%zu):\n", g_pose_library.gait_count());
                        for (size_t i = 0; i < g_pose_library.gait_count(); ++i) {
                            printf("  %s\n", g_pose_library.gait_at(i).name);
                        }
                    }
                    else if (strncmp(usb_buf, "motion ", 7) == 0) {
                        shared_state.request_motion(usb_buf + 7);
                    }
                    else if (strcmp(usb_buf, "motions") == 0) {
                        printf("Verfuegbare Bewegungssequenzen (%zu):\n", g_pose_library.motion_count());
                        for (size_t i = 0; i < g_pose_library.motion_count(); ++i) {
                            printf("  %s (%zu Schritte)\n", g_pose_library.motion_at(i).name,
                                   g_pose_library.motion_at(i).step_count);
                        }
                    }

                    usb_idx = 0;
                    printf("Befehl: ");
                }
            } else if (usb_idx < sizeof(usb_buf) - 1 && ch >= 32 && ch <= 126) {
                usb_buf[usb_idx++] = static_cast<char>(ch);
                putchar(ch);
            }
        }

        sleep_us(1000);
    }
}

// =============================================================================
// CORE 0: KINEMATIK-KONTROLLE MIT KONTROLLIERTEM ZYKLUS-AUSLAUF (50 Hz)
// =============================================================================
int main() {
    stdio_init_all();

    for (int i = 0; i < 20; ++i) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }

    print_help();

    // Posen-/Gait-/Bewegungsbibliothek laden - MUSS vor multicore_launch_core1()
    // passieren, da Core 1 (USB-Konsole: 'poses'/'gaits'/'motions') direkt darauf
    // liest, sobald er laeuft.
    size_t lib_entries = g_pose_library.load_from_json(Hexapod::POSE_LIBRARY_JSON);
    printf("--> [Boot] Posen-Bibliothek geladen: %zu Eintraege (%zu Posen, %zu Gaits, %zu Motions).\n",
           lib_entries, g_pose_library.pose_count(), g_pose_library.gait_count(), g_pose_library.motion_count());

    Hexapod::ServoDriver servos;
    servos.init();
    g_servos_ptr = &servos;

    Hexapod::TripodGait gait(servos);
    g_gait_ptr = &gait;

    // Boot-Sequenz: die letzte gespeicherte Pose + den zugehoerigen Modus laden
    // (JSON, letzter Flash-Sektor). Bei STAND wird als Startpunkt fuer die erste
    // Rampe genutzt, statt blind 135 Grad ueberall anzunehmen - die Beine
    // "springen" beim Boot nicht mehr unnoetig in eine andere Stellung (z.B.
    // Tibia nicht mehr aktiv nach aussen gestreckt). Bei BLUME wird NICHT ueber
    // die IK-Standpose gefahren, sondern direkt (sanft) die gespeicherte
    // Blume-Pose wiederhergestellt - der Roboter bleibt also in Blume, statt
    // beim Booten kurz aufzustehen und wieder umzuklappen.
    // Gibt es keine gueltige gespeicherte Pose (allererster Boot, oder Flash
    // korrupt/leer), faellt es sicher auf STAND von Neutral (135 Grad) zurueck.
    // Der Servo-Slew-Limiter (Config::MAX_SLEW_DEG_PER_SEC) bleibt in JEDEM Fall
    // als zusaetzliche harte Sicherheitsgrenze aktiv.
    Hexapod::RobotMode boot_mode = Hexapod::RobotMode::STAND;
    if (!Hexapod::PoseStorage::load_last_pose(g_current_angles, boot_mode)) {
        printf("--> [Boot] Keine gespeicherte Pose gefunden - starte von Neutral (135 Grad).\n");
        g_current_angles.fill(135.0f);
        boot_mode = Hexapod::RobotMode::STAND;
    } else {
        printf("--> [Boot] Letzte gespeicherte Pose geladen (Modus: %s).\n",
               boot_mode == Hexapod::RobotMode::BLUME ? "BLUME" : "STAND");
    }

    // Der ServoDriver selbst startet intern immer bei 135 Grad (Konstruktor-
    // Default) - unabhaengig davon, was wir gerade geladen/angenommen haben.
    // seed_current_angles() gleicht das an: der Treiber "glaubt" ab jetzt exakt
    // die geladene/angenommene Pose, OHNE dass der Slew-Limiter das faelschlich
    // als abrupten Sprung interpretiert (siehe Kommentar dort). Das ist die
    // Voraussetzung dafuer, dass die nachfolgende safe_transition_to_pose()
    // (die jetzt immer vom TREIBER-Zustand ausgeht, nicht von g_current_angles)
    // korrekt von der richtigen Basis aus rampt.
    servos.seed_current_angles(g_current_angles);

    if (boot_mode == Hexapod::RobotMode::BLUME) {
        // g_current_angles enthaelt bereits die gespeicherte Blume-Pose - diese
        // kurze Rampe ist quasi ein No-Op (Start=Ziel), laeuft aber trotzdem
        // ueber den slew-limitierten Pfad statt einem direkten Sprung.
        safe_transition_to_pose(servos, g_current_angles, 0.5f);
    } else {
        safe_transition_to_pose(servos, get_stand_pose(gait, servos), 3.0f);
    }

    // Core 1 (Button/USB/UART) muss den gleichen Modus kennen wie wir gerade
    // hergestellt haben - sonst wuerde der Zustandsautomat im naechsten Tick
    // sofort von BLUME zurueck nach STAND wechseln (SharedState startet mit
    // requested_mode=STAND per Default).
    shared_state.request_mode(boot_mode);

    multicore_launch_core1(core1_entry);

    constexpr uint32_t LOOP_INTERVAL_US = 20000; // 50 Hz
    absolute_time_t next_tick = get_absolute_time();

    Hexapod::RobotMode current_mode = boot_mode;
    float global_phase = 0.0f;
    bool stopping_in_progress = false;

    printf("--> [Core 0] Echtzeit-Kinematik mit Kollisionsschutz aktiv.\n\n");

    while (true) {
        Hexapod::RobotMode req_mode = shared_state.get_requested_mode();

        // Diagnose-Anfrage: nur ausserhalb WALK ausfuehren (Sicherheit - Beine
        // sollen fuer den Test frei in der Luft sein, kein aktiver Gangzyklus).
        if (shared_state.get_diag_request()) {
            shared_state.set_diag_request(false);
            if (current_mode != Hexapod::RobotMode::WALK) {
                run_leg_diagnostics(servos, gait);
                safe_transition_to_pose(servos, get_stand_pose(gait, servos), 0.8f);
            } else {
                printf("\n[Diag] Bitte zuerst 'stand' oder 'off', dann 'diag'.\n");
            }
        }

        if (shared_state.get_calzero_request()) {
            shared_state.set_calzero_request(false);
            if (current_mode != Hexapod::RobotMode::WALK) {
                run_coxa_zero_calibration(servos, gait);
                current_mode = Hexapod::RobotMode::STAND;
            } else {
                printf("\n[Calzero] Bitte zuerst 'stand' oder 'off', dann 'calzero'.\n");
            }
        }

        // 'pose <name>': sanft zu einer benannten Pose aus der Bibliothek fahren.
        // Nur ausserhalb WALK (gleiche Sicherheitslogik wie diag/calzero) - eine
        // Pose ist per Definition ein STATISCHES Ziel, waehrend WALK aktiv den
        // Gangzyklus faehrt.
        {
            char pose_name[24]{};
            if (shared_state.take_pose_request(pose_name, sizeof(pose_name))) {
                if (current_mode == Hexapod::RobotMode::WALK) {
                    printf("\n[Pose] Bitte zuerst 'stand' oder 'off', dann 'pose %s'.\n", pose_name);
                } else {
                    const Hexapod::PoseDefinition* found = g_pose_library.find_pose(pose_name);
                    if (found) {
                        printf("\n[Pose] Fahre zu '%s'...\n", pose_name);
                        auto target = gait.compute_static_pose(*found, Hexapod::Config::COXA_REST_DEG);
                        safe_transition_to_pose(servos, target, 1.5f);
                        current_mode = Hexapod::RobotMode::STAND;
                        Hexapod::PoseStorage::save_pose(g_current_angles, Hexapod::RobotMode::STAND);
                    } else {
                        printf("\n[Pose] Unbekannte Pose '%s' - 'poses' zeigt die verfuegbaren an.\n", pose_name);
                    }
                }
            }
        }

        // 'gait <name>': waehlt nur das Gangmuster fuer den NAECHSTEN 'walk' -
        // bewegt selbst nichts, deshalb ohne WALK-Sperre erlaubt.
        {
            char gait_name[16]{};
            if (shared_state.take_gait_request(gait_name, sizeof(gait_name))) {
                const Hexapod::GaitDefinition* found = g_pose_library.find_gait(gait_name);
                if (found) {
                    g_active_gait = *found;
                    printf("\n[Gait] Aktives Gangmuster: '%s' (duty_factor=%.3f)\n", gait_name, found->duty_factor);
                } else {
                    printf("\n[Gait] Unbekanntes Gangmuster '%s' - 'gaits' zeigt die verfuegbaren an.\n", gait_name);
                }
            }
        }

        // 'motion <name>': spielt eine Sequenz aus POSE- und/oder GAIT-Schritten
        // ab - generalisiert, was safe_transition_to_blume() bisher hartkodiert
        // in main.cpp macht. Gleiche WALK-Sperre wie 'pose'.
        {
            char motion_name[24]{};
            if (shared_state.take_motion_request(motion_name, sizeof(motion_name))) {
                if (current_mode == Hexapod::RobotMode::WALK) {
                    printf("\n[Motion] Bitte zuerst 'stand' oder 'off', dann 'motion %s'.\n", motion_name);
                } else {
                    const Hexapod::MotionDefinition* found = g_pose_library.find_motion(motion_name);
                    if (found) {
                        printf("\n[Motion] Spiele '%s' (%zu Schritte)...\n", motion_name, found->step_count);
                        bool ok = true;
                        bool last_was_gait = false;
                        for (size_t s = 0; s < found->step_count; ++s) {
                            const auto& step = found->steps[s];
                            if (step.type == Hexapod::MotionStepType::POSE) {
                                const Hexapod::PoseDefinition* step_pose = g_pose_library.find_pose(step.pose_name);
                                if (!step_pose) {
                                    printf("[Motion] Schritt %zu: unbekannte Pose '%s' - Sequenz abgebrochen.\n",
                                           s, step.pose_name);
                                    ok = false;
                                    break;
                                }
                                auto target = gait.compute_static_pose(*step_pose, Hexapod::Config::COXA_REST_DEG);
                                for (uint16_t r = 0; r < step.repeat; ++r) {
                                    safe_transition_to_pose(servos, target, step.duration_s);
                                }
                                last_was_gait = false;
                            } else if (step.type == Hexapod::MotionStepType::ORBIT) {
                                run_orbit_step(servos, gait, step);
                                last_was_gait = false; // Beine bewegen sich nicht, kein Nachziehen noetig
                            } else {
                                run_gait_step(servos, gait, step);
                                last_was_gait = true;
                            }
                        }
                        current_mode = Hexapod::RobotMode::STAND;
                        if (ok) {
                            if (last_was_gait) {
                                // Letzter Schritt war ein GAIT-Schritt - Beine koennten
                                // mitten in Schwung/Stand haengengeblieben sein, sauber
                                // in den IK-Stand nachziehen statt so zu belassen.
                                safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.0f);
                            }
                            Hexapod::PoseStorage::save_pose(g_current_angles, Hexapod::RobotMode::STAND);
                        }
                    } else {
                        printf("\n[Motion] Unbekannte Sequenz '%s' - 'motions' zeigt die verfuegbaren an.\n",
                               motion_name);
                    }
                }
            }
        }

        // 'turn <winkel>': dreht exakt um den angegebenen Winkel. Nur ausserhalb
        // WALK (gleiche Sicherheitslogik wie pose/motion).
        {
            float turn_angle = 0.0f;
            if (shared_state.take_turn_request(turn_angle)) {
                if (current_mode == Hexapod::RobotMode::WALK) {
                    printf("\n[Turn] Bitte zuerst 'stand' oder 'off', dann 'turn %.1f'.\n", turn_angle);
                } else {
                    run_turn(servos, gait, turn_angle);
                    current_mode = Hexapod::RobotMode::STAND;
                    Hexapod::PoseStorage::save_pose(g_current_angles, Hexapod::RobotMode::STAND);
                }
            }
        }

        // 'speed <faktor>': globale Geschwindigkeit fuer alle Bewegungen. Ohne
        // WALK-Sperre erlaubt (wirkt erst beim naechsten Tick/Uebergang, bewegt
        // selbst nichts). Grober Sicherheitsbereich [0.2, 3.0] - deutlich
        // extremere Werte koennten den Slew-Limiter (Config::MAX_SLEW_DEG_PER_SEC)
        // so oft zum Kappen bringen, dass Bewegungen unkontrolliert ruckeln.
        {
            float speed_val = 0.0f;
            if (shared_state.take_speed_request(speed_val)) {
                if (speed_val < 0.2f || speed_val > 3.0f) {
                    printf("\n[Speed] %.2f liegt ausserhalb des sinnvollen Bereichs [0.2, 3.0] - ignoriert.\n",
                           speed_val);
                } else {
                    g_speed_factor = speed_val;
                    printf("\n[Speed] Geschwindigkeitsfaktor: %.2f\n", g_speed_factor);
                }
            }
        }

        // 1. ZUSTANDSMASCHINE: Wenn ein Moduswechsel gewünscht ist
        if (req_mode != current_mode) {
            // WENN DER ROBOTER GERADE LÄUFT: Zuerst den Zyklus sauber zu Ende laufen lassen!
            if (current_mode == Hexapod::RobotMode::WALK) {
                stopping_in_progress = true;

                // Weiterlaufen, bis die Phase genau 0.0 erreicht hat (alle 6 Beine am Boden)
                if (global_phase > 0.05f && global_phase < 0.95f) {
                    global_phase += (0.02f * g_speed_factor) / Hexapod::Config::CYCLE_TIME;
                    if (global_phase >= 1.0f) global_phase -= 1.0f;

                    gait.compute_and_apply(global_phase, get_walk_velocity(shared_state.get_walk_direction()),
                                            Hexapod::Config::STEERING_TRIM_DEG_PER_SEC, Hexapod::Config::COXA_WALK_DEG,
                                            g_active_gait);
                    servos.update();
                    g_current_angles = servos.get_command_angles();

                    next_tick = delayed_by_us(next_tick, LOOP_INTERVAL_US);
                    sleep_until(next_tick);
                    continue; // Nächsten Tick abwarten bis Phase abgeschlossen
                }

                // Phase ist am Boden angekommen: Coxa sauber zurückstellen
                gait.transition_coxa_base(Hexapod::Config::COXA_WALK_DEG, Hexapod::Config::COXA_REST_DEG, 0.4f, 20, g_speed_factor);
                // 1.2s statt vorher 0.5s: bei grossen Winkelspruengen (z.B. Tibia)
                // reicht 0.5s bei MAX_SLEW_DEG_PER_SEC=250 rechnerisch nicht immer,
                // um in einem Durchgang wirklich anzukommen (siehe Slew-Limiter-Fix).
                safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.2f);
                global_phase = 0.0f;
                stopping_in_progress = false;
                current_mode = Hexapod::RobotMode::STAND;
                Hexapod::PoseStorage::save_pose(g_current_angles, Hexapod::RobotMode::STAND); // sicherer Checkpoint fuer den naechsten Boot
            }

            // AUS DEM SICHEREN STAND IN DEN ZIELMODUS WECHSELN
            if (req_mode == Hexapod::RobotMode::BLUME) {
                safe_transition_to_blume(servos, gait);
                current_mode = Hexapod::RobotMode::BLUME;
            }
            else if (req_mode == Hexapod::RobotMode::STAND) {
                safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.2f);
                current_mode = Hexapod::RobotMode::STAND;
                Hexapod::PoseStorage::save_pose(g_current_angles, Hexapod::RobotMode::STAND); // sicherer Checkpoint fuer den naechsten Boot
            }
            else if (req_mode == Hexapod::RobotMode::WALK) {
                if (current_mode == Hexapod::RobotMode::BLUME) {
                    safe_transition_to_pose(servos, get_stand_pose(gait, servos), 1.2f);
                }
                gait.transition_coxa_base(Hexapod::Config::COXA_REST_DEG, Hexapod::Config::COXA_WALK_DEG, 0.4f, 20, g_speed_factor);
                global_phase = 0.0f;
                current_mode = Hexapod::RobotMode::WALK;
            }
            else if (req_mode == Hexapod::RobotMode::OFF) {
                servos.disable_all();
                current_mode = Hexapod::RobotMode::OFF;
            }

            next_tick = get_absolute_time();
        }

        // 2. KONTINUIERLICHER BETRIEB
        if (current_mode == Hexapod::RobotMode::WALK && !stopping_in_progress) {
            global_phase += (0.02f * g_speed_factor) / Hexapod::Config::CYCLE_TIME;
            if (global_phase >= 1.0f) global_phase -= 1.0f;

            // Hinweis: STEERING_TRIM_DEG_PER_SEC wurde nur fuer FORWARD kalibriert
            // (siehe Bugfix-Historie) - bei backward/left/right ggf. neu einmessen,
            // falls dort ein aehnlicher Versatz auftritt.
            gait.compute_and_apply(global_phase, get_walk_velocity(shared_state.get_walk_direction()),
                                    Hexapod::Config::STEERING_TRIM_DEG_PER_SEC, Hexapod::Config::COXA_WALK_DEG,
                                    g_active_gait);
            servos.update();
            g_current_angles = servos.get_command_angles();
        }
        else if (current_mode == Hexapod::RobotMode::STAND) {
            servos.update();
        }
        else if (current_mode == Hexapod::RobotMode::BLUME) {
            servos.update();
        }

        next_tick = delayed_by_us(next_tick, LOOP_INTERVAL_US);
        sleep_until(next_tick);
    }

    return 0;
}