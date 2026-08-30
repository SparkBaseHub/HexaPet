#pragma once
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "core/config.hpp"
#include "core/robot_state.hpp"
#include <array>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace Hexapod {

    // Letzter 4KB-Sektor des 2MB-Flash (Adresse 0x101FF000)
    constexpr uint32_t FLASH_TARGET_OFFSET = (2 * 1024 * 1024) - FLASH_SECTOR_SIZE;

    // Menschlich lesbares JSON statt binaerem Struct - einfacher zu inspizieren
    // und robust gegen Struct-Layout-Aenderungen zwischen Firmware-Versionen.
    // Format: {"magic":"HEXP1","mode":"STAND","angles":[a0,...,a17]}
    // "mode" haelt fest, in welchem Zustand die Pose gespeichert wurde (aktuell
    // STAND oder BLUME - WALK/OFF werden nie gespeichert, siehe main.cpp), damit
    // der Boot-Vorgang weiss, ob er in den IK-Stand oder direkt zurueck in die
    // (fixe, nicht IK-berechnete) Blume-Pose fahren soll.
    // Reihenfolge der 18 Winkel = ServoDriver::get_command_angles() Layout
    // (0-5 Tibia, 6-11 Femur, 12-17 Coxa).
    constexpr const char* POSE_JSON_MAGIC = "{\"magic\":\"HEXP1\",";

    class PoseStorage {
    public:
        // Liest die zuletzt gespeicherte Pose + den zugehoerigen Modus. Gibt
        // false zurueck, wenn noch nie gespeichert wurde (Flash nach Erase =
        // 0xFF-Bytes, matcht das Magic-Praefix nie) oder die Daten nicht
        // parsebar sind - der Aufrufer MUSS in diesem Fall einen sicheren
        // Default annehmen (siehe main.cpp Boot-Sequenz).
        static bool load_last_pose(std::array<float, Config::NUM_SERVOS>& out_angles,
                                    RobotMode& out_mode) {
            const char* flash_contents = reinterpret_cast<const char*>(XIP_BASE + FLASH_TARGET_OFFSET);

            if (strncmp(flash_contents, POSE_JSON_MAGIC, strlen(POSE_JSON_MAGIC)) != 0) {
                return false;
            }

            // "mode" ist optional/rueckwaertskompatibel: fehlt es (altes Format
            // oder unbekannter Wert), sicherheitshalber STAND annehmen - niemals
            // ungefragt in eine andere Pose "raten".
            const char* mode_key = strstr(flash_contents, "\"mode\":\"");
            if (mode_key && strncmp(mode_key + strlen("\"mode\":\""), "BLUME", 5) == 0) {
                out_mode = RobotMode::BLUME;
            } else {
                out_mode = RobotMode::STAND;
            }

            const char* angles_key = strstr(flash_contents, "\"angles\":[");
            if (!angles_key) return false;
            const char* cursor = angles_key + strlen("\"angles\":[");

            std::array<float, Config::NUM_SERVOS> parsed{};
            for (uint8_t i = 0; i < Config::NUM_SERVOS; ++i) {
                char* end = nullptr;
                float val = strtof(cursor, &end);
                if (end == cursor) return false; // Parse-Fehler - korrupte/unerwartete Daten
                parsed[i] = val;
                cursor = end;
                while (*cursor == ',' || *cursor == ' ') ++cursor;
            }

            out_angles = parsed;
            return true;
        }

        // Serialisiert Winkel + Modus als JSON und schreibt sie mit harter
        // Interrupt-Sperre in den letzten Flash-Sektor. NICHT kontinuierlich
        // aufrufen (z.B. nicht pro Gangzyklus-Tick) - Flash haelt nur eine
        // begrenzte Zahl Schreib-/Loeschzyklen aus. Aufrufpunkte: immer wenn
        // der Roboter sauber in STAND oder BLUME ankommt, sowie vor Dormant-Sleep.
        //
        // WICHTIG (Dual-Core-Sicherheit): Waehrend flash_range_erase()/program()
        // ist der komplette Flash (XIP) fuer BEIDE Cores kurz nicht lesbar. Core 1
        // fuehrt seinen Loop-Code aber direkt aus dem Flash aus (core1_entry in
        // main.cpp) - ohne Absicherung wuerde Core 1 dabei mitten im Instruktion-
        // Fetch haengen bleiben/abstuerzen. multicore_lockout_start_blocking()
        // pausiert Core 1 sauber fuer die Dauer des Schreibvorgangs (setzt voraus,
        // dass Core 1 zuvor einmal multicore_lockout_victim_init() aufgerufen hat -
        // siehe core1_entry() in main.cpp).
        static void save_pose(const std::array<float, Config::NUM_SERVOS>& angles, RobotMode mode) {
            const char* mode_str = (mode == RobotMode::BLUME) ? "BLUME" : "STAND";

            char json[FLASH_PAGE_SIZE]{};
            int len = snprintf(json, sizeof(json), "%s\"mode\":\"%s\",\"angles\":[", POSE_JSON_MAGIC, mode_str);
            for (uint8_t i = 0; i < Config::NUM_SERVOS; ++i) {
                len += snprintf(json + len, sizeof(json) - static_cast<size_t>(len),
                                 "%s%.2f", (i == 0) ? "" : ",", angles[i]);
            }
            len += snprintf(json + len, sizeof(json) - static_cast<size_t>(len), "]}");

            if (len <= 0 || static_cast<size_t>(len) >= FLASH_PAGE_SIZE) {
                return; // sollte bei 18 Floats nie passieren, aber sicherheitshalber
            }

            multicore_lockout_start_blocking();
            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
            flash_range_program(FLASH_TARGET_OFFSET, reinterpret_cast<const uint8_t*>(json), FLASH_PAGE_SIZE);
            restore_interrupts(ints);
            multicore_lockout_end_blocking();
        }
    };

} // namespace Hexapod