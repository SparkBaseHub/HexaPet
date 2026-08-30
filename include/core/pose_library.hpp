#pragma once
#include "core/pose_types.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace Hexapod {

    // Laedt eine Posen-/Gait-/Bewegungsbibliothek aus einem JSON-Text. Bewusst
    // KEIN generischer JSON-Parser (zu schwer fuer den RP2040) - sondern auf
    // exakt dieses Schema zugeschnitten:
    //
    // {
    //   "poses": [
    //     { "name": "home", "type": "stance",
    //       "stance_radius": 130, "standing_height": -80,
    //       "body": {"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0} },
    //     { "name": "compact", "type": "raw",
    //       "tibia": [135,135,135,135,135,135],
    //       "femur": [160,160,160,160,160,160],
    //       "coxa":  [135,135,135,135,135,135] }
    //   ],
    //   "gaits": [
    //     { "name": "tripod", "duty_factor": 0.5,
    //       "phase_offsets": [0,0.5,0,0.5,0,0.5] }
    //   ],
    //   "motions": [
    //     { "name": "wake_up", "steps": [
    //         {"type":"pose","pose":"sleep","duration_s":0},
    //         {"type":"pose","pose":"crouched","duration_s":1.5},
    //         {"type":"pose","pose":"home","duration_s":1.0} ] },
    //     { "name": "wiggle", "steps": [
    //         {"type":"pose","pose":"sway_left","duration_s":0.25},
    //         {"type":"pose","pose":"sway_right","duration_s":0.25},
    //         {"type":"gait","gait":"wave","vy":30,"duration_s":1.5,"repeat":2},
    //         {"type":"pose","pose":"home","duration_s":0.6} ] }
    //   ]
    // }
    //
    // Zwei Schritt-Typen in "motions.steps": "type":"pose" (Default, wenn
    // weggelassen) faehrt sanft zu einer benannten Pose; "type":"gait" laesst
    // den Gangzyklus fuer duration_s Sekunden mit fester Geschwindigkeit
    // (vx/vy, mm/s) und Rotation (rotation, Grad/s) laufen - fuer Bewegungen,
    // die keine statische Zielpose sind (Wippen, Schieben, Tanz-Moves).
    // "repeat" (optional, Default 1) wiederholt den einzelnen Schritt.
    //
    // "poses"/"gaits"/"motions" sind alle optional; fehlende Felder in einem
    // einzelnen Eintrag behalten die Defaults aus pose_types.hpp. Bein-Reihen-
    // folge ueberall wie im restlichen Code: 0=VR,1=VL,2=ML,3=HL,4=HR,5=MR.
    //
    // ABSICHTLICH QUELLENUNABHAENGIG: load_from_json() nimmt einen rohen
    // C-String, egal ob der aus einem einkompilierten String (main.cpp) oder
    // spaeter aus einem per UART empfangenen Puffer (KI-Kameramodul) kommt -
    // der Parser selbst muss dafuer nie angefasst werden.
    class PoseLibrary {
    public:
        static constexpr size_t MAX_POSES = 32;
        static constexpr size_t MAX_GAITS = 6;
        static constexpr size_t MAX_MOTIONS = 12;

        // Parst json_text und ERSETZT den aktuellen Inhalt komplett (kein Merge).
        // Gibt die Anzahl erfolgreich geparster Eintraege zurueck (poses+gaits+
        // motions) - 0 bedeutet "nichts gefunden", ist aber kein harter Fehler
        // (leeres/kaputtes JSON fuehrt nie zum Absturz, nur zu einer leeren Lib).
        size_t load_from_json(const char* json_text) {
            pose_count_ = 0;
            gait_count_ = 0;
            motion_count_ = 0;

            parse_array(json_text, "\"poses\"", [this](const char* obj_start, const char* obj_end) {
                if (pose_count_ >= MAX_POSES) return;
                PoseDefinition pose{};
                parse_string_field(obj_start, obj_end, "\"name\"", pose.name, sizeof(pose.name));
                char type_buf[8]{};
                parse_string_field(obj_start, obj_end, "\"type\"", type_buf, sizeof(type_buf));
                pose.type = (std::strcmp(type_buf, "raw") == 0) ? PoseType::RAW : PoseType::STANCE;

                if (pose.type == PoseType::STANCE) {
                    parse_float_field(obj_start, obj_end, "\"stance_radius\"", pose.stance_radius);
                    parse_float_field(obj_start, obj_end, "\"standing_height\"", pose.standing_height);
                    const char* body_start = find_key(obj_start, obj_end, "\"body\"");
                    if (body_start) {
                        const char* bobj_start = nullptr;
                        const char* bobj_end = nullptr;
                        if (find_object_bounds(body_start, obj_end, bobj_start, bobj_end)) {
                            parse_float_field(bobj_start, bobj_end, "\"x\"", pose.body.x);
                            parse_float_field(bobj_start, bobj_end, "\"y\"", pose.body.y);
                            parse_float_field(bobj_start, bobj_end, "\"z\"", pose.body.z);
                            parse_float_field(bobj_start, bobj_end, "\"roll\"", pose.body.roll_deg);
                            parse_float_field(bobj_start, bobj_end, "\"pitch\"", pose.body.pitch_deg);
                            parse_float_field(bobj_start, bobj_end, "\"yaw\"", pose.body.yaw_deg);
                        }
                    }
                } else {
                    parse_float_array6(obj_start, obj_end, "\"tibia\"", pose.tibia_deg);
                    parse_float_array6(obj_start, obj_end, "\"femur\"", pose.femur_deg);
                    parse_float_array6(obj_start, obj_end, "\"coxa\"",  pose.coxa_deg);
                }

                if (pose.name[0] != '\0') {
                    poses_[pose_count_++] = pose;
                }
            });

            parse_array(json_text, "\"gaits\"", [this](const char* obj_start, const char* obj_end) {
                if (gait_count_ >= MAX_GAITS) return;
                GaitDefinition gait{};
                parse_string_field(obj_start, obj_end, "\"name\"", gait.name, sizeof(gait.name));
                parse_float_field(obj_start, obj_end, "\"duty_factor\"", gait.duty_factor);
                parse_float_array6(obj_start, obj_end, "\"phase_offsets\"", gait.phase_offsets);

                if (gait.name[0] != '\0') {
                    gaits_[gait_count_++] = gait;
                }
            });

            parse_array(json_text, "\"motions\"", [this](const char* obj_start, const char* obj_end) {
                if (motion_count_ >= MAX_MOTIONS) return;
                MotionDefinition motion{};
                parse_string_field(obj_start, obj_end, "\"name\"", motion.name, sizeof(motion.name));

                const char* steps_start = find_key(obj_start, obj_end, "\"steps\"");
                if (steps_start) {
                    parse_array(steps_start, "\"steps\"", [&motion](const char* s_start, const char* s_end) {
                        if (motion.step_count >= MotionDefinition::MAX_STEPS) return;
                        MotionStep step{};

                        char type_buf[8]{};
                        parse_string_field(s_start, s_end, "\"type\"", type_buf, sizeof(type_buf));
                        if (std::strcmp(type_buf, "gait") == 0) step.type = MotionStepType::GAIT;
                        else if (std::strcmp(type_buf, "orbit") == 0) step.type = MotionStepType::ORBIT;
                        else step.type = MotionStepType::POSE;

                        parse_float_field(s_start, s_end, "\"duration_s\"", step.duration_s);
                        float repeat_f = 1.0f;
                        parse_float_field(s_start, s_end, "\"repeat\"", repeat_f);
                        step.repeat = (repeat_f >= 1.0f) ? static_cast<uint16_t>(repeat_f) : 1;

                        if (step.type == MotionStepType::POSE) {
                            parse_string_field(s_start, s_end, "\"pose\"", step.pose_name, sizeof(step.pose_name));
                            if (step.pose_name[0] == '\0') return; // POSE-Schritt ohne Pose ist ungueltig
                        } else if (step.type == MotionStepType::ORBIT) {
                            parse_float_field(s_start, s_end, "\"amplitude\"", step.orbit_amplitude_deg);
                            parse_float_field(s_start, s_end, "\"revolutions\"", step.orbit_revolutions);
                            float cw_val = 0.0f;
                            parse_float_field(s_start, s_end, "\"clockwise\"", cw_val);
                            step.orbit_clockwise = (cw_val != 0.0f);
                        } else {
                            parse_string_field(s_start, s_end, "\"gait\"", step.gait_name, sizeof(step.gait_name));
                            parse_float_field(s_start, s_end, "\"vx\"", step.velocity_x);
                            parse_float_field(s_start, s_end, "\"vy\"", step.velocity_y);
                            parse_float_field(s_start, s_end, "\"rotation\"", step.rotation_deg_per_sec);
                            parse_float_field(s_start, s_end, "\"lean_roll\"", step.lean_roll_deg);
                            parse_float_field(s_start, s_end, "\"lean_pitch\"", step.lean_pitch_deg);
                        }

                        motion.steps[motion.step_count++] = step;
                    }, steps_start, obj_end);
                }

                if (motion.name[0] != '\0' && motion.step_count > 0) {
                    motions_[motion_count_++] = motion;
                }
            });

            return pose_count_ + gait_count_ + motion_count_;
        }

        const PoseDefinition* find_pose(const char* name) const {
            for (size_t i = 0; i < pose_count_; ++i) {
                if (poses_[i].name_is(name)) return &poses_[i];
            }
            return nullptr;
        }

        const GaitDefinition* find_gait(const char* name) const {
            for (size_t i = 0; i < gait_count_; ++i) {
                if (gaits_[i].name_is(name)) return &gaits_[i];
            }
            return nullptr;
        }

        const MotionDefinition* find_motion(const char* name) const {
            for (size_t i = 0; i < motion_count_; ++i) {
                if (motions_[i].name_is(name)) return &motions_[i];
            }
            return nullptr;
        }

        size_t pose_count() const { return pose_count_; }
        const PoseDefinition& pose_at(size_t i) const { return poses_[i]; }
        size_t gait_count() const { return gait_count_; }
        const GaitDefinition& gait_at(size_t i) const { return gaits_[i]; }
        size_t motion_count() const { return motion_count_; }
        const MotionDefinition& motion_at(size_t i) const { return motions_[i]; }

    private:
        std::array<PoseDefinition, MAX_POSES> poses_{};
        size_t pose_count_{0};
        std::array<GaitDefinition, MAX_GAITS> gaits_{};
        size_t gait_count_{0};
        std::array<MotionDefinition, MAX_MOTIONS> motions_{};
        size_t motion_count_{0};

        // strstr(), aber auf einen [start,end)-Bereich begrenzt (fuer's Suchen
        // NUR innerhalb eines bereits gefundenen Objekts/Arrays, nicht im ganzen
        // restlichen Dokument - sonst wuerde z.B. "body" aus dem falschen Eintrag
        // gematcht werden). Fuer reine Zeichen-/Teilstring-Suche (z.B. ":", ein
        // Anfuehrungszeichen) - NICHT fuer JSON-Schluessel-Suche, dafuer siehe
        // find_key() weiter unten.
        static const char* strstr_bounded(const char* start, const char* end, const char* needle) {
            size_t needle_len = std::strlen(needle);
            if (needle_len == 0 || end <= start) return nullptr;
            for (const char* p = start; p + needle_len <= end; ++p) {
                if (std::strncmp(p, needle, needle_len) == 0) return p;
            }
            return nullptr;
        }

        // Sucht "key" wirklich als JSON-SCHLUESSEL (direkt gefolgt von optionalem
        // Whitespace und einem ':'), NICHT als beliebige Fundstelle. WICHTIG:
        // strstr_bounded() allein reicht nicht - key kann zufaellig auch als
        // WERT eines anderen Feldes vorkommen (z.B. "type":"gait" enthaelt den
        // Text "gait", obwohl es kein "gait"-Schluessel ist). Ohne diese Pruefung
        // wuerde z.B. bei einem GAIT-Schritt ohne eigenes "gait"-Feld der Parser
        // faelschlich den naechsten Schluessel-Namen als gait_name uebernehmen.
        static const char* find_key(const char* start, const char* end, const char* key) {
            size_t key_len = std::strlen(key);
            const char* search_from = start;
            while (search_from < end) {
                const char* found = strstr_bounded(search_from, end, key);
                if (!found) return nullptr;
                const char* after = found + key_len;
                while (after < end && (*after == ' ' || *after == '\t' || *after == '\n' || *after == '\r')) ++after;
                if (after < end && *after == ':') {
                    return found; // echter Schluessel-Treffer
                }
                search_from = found + 1; // Zufallstreffer (z.B. als Wert) - weitersuchen
            }
            return nullptr;
        }

        // Findet den durch { } begrenzten Bereich, der bei/nach 'from' beginnt
        // (Klammertiefe wird mitgezaehlt, damit verschachtelte Objekte korrekt
        // uebersprungen werden). out_start zeigt auf das Zeichen NACH der
        // oeffnenden '{', out_end auf die schliessende '}'.
        static bool find_object_bounds(const char* from, const char* limit,
                                        const char*& out_start, const char*& out_end) {
            const char* brace = nullptr;
            for (const char* p = from; p < limit; ++p) {
                if (*p == '{') { brace = p; break; }
            }
            if (!brace) return false;

            int depth = 0;
            for (const char* p = brace; p < limit; ++p) {
                if (*p == '{') ++depth;
                else if (*p == '}') {
                    --depth;
                    if (depth == 0) {
                        out_start = brace + 1;
                        out_end = p;
                        return true;
                    }
                }
            }
            return false;
        }

        // Findet den durch [ ] begrenzten Bereich analog zu find_object_bounds,
        // aber fuer Arrays (Klammertiefe auf '[' ']' statt '{' '}').
        static bool find_array_bounds(const char* from, const char* limit,
                                       const char*& out_start, const char*& out_end) {
            const char* bracket = nullptr;
            for (const char* p = from; p < limit; ++p) {
                if (*p == '[') { bracket = p; break; }
            }
            if (!bracket) return false;

            int depth = 0;
            for (const char* p = bracket; p < limit; ++p) {
                if (*p == '[') ++depth;
                else if (*p == ']') {
                    --depth;
                    if (depth == 0) {
                        out_start = bracket + 1;
                        out_end = p;
                        return true;
                    }
                }
            }
            return false;
        }

        // Iteriert ueber alle {...}-Objekte innerhalb des Arrays, das nach dem
        // Schluessel 'key' folgt (z.B. "poses"), und ruft fn(obj_start,obj_end)
        // fuer jedes auf. search_start/search_limit begrenzen, WO nach 'key'
        // gesucht wird (Default: das ganze Dokument).
        template <typename Fn>
        static void parse_array(const char* doc, const char* key, Fn fn,
                                 const char* search_start = nullptr, const char* search_limit = nullptr) {
            const char* start = search_start ? search_start : doc;
            const char* limit = search_limit ? search_limit : (doc + std::strlen(doc));

            const char* key_pos = find_key(start, limit, key);
            if (!key_pos) return;

            const char* arr_start = nullptr;
            const char* arr_end = nullptr;
            if (!find_array_bounds(key_pos, limit, arr_start, arr_end)) return;

            const char* cursor = arr_start;
            while (cursor < arr_end) {
                const char* obj_start = nullptr;
                const char* obj_end = nullptr;
                if (!find_object_bounds(cursor, arr_end, obj_start, obj_end)) break;
                fn(obj_start, obj_end);
                cursor = obj_end + 1;
            }
        }

        // Extrahiert einen String-Wert "key": "value" innerhalb [start,end).
        // Kopiert hoechstens out_size-1 Zeichen, terminiert immer NUL. Leer
        // (out[0]='\0'), wenn der Schluessel nicht gefunden wird.
        static void parse_string_field(const char* start, const char* end, const char* key,
                                        char* out, size_t out_size) {
            out[0] = '\0';
            const char* key_pos = find_key(start, end, key);
            if (!key_pos) return;
            const char* colon = strstr_bounded(key_pos, end, ":");
            if (!colon) return;
            const char* quote1 = nullptr;
            for (const char* p = colon; p < end; ++p) {
                if (*p == '"') { quote1 = p + 1; break; }
            }
            if (!quote1) return;
            const char* quote2 = nullptr;
            for (const char* p = quote1; p < end; ++p) {
                if (*p == '"') { quote2 = p; break; }
            }
            if (!quote2) return;

            size_t len = static_cast<size_t>(quote2 - quote1);
            if (len >= out_size) len = out_size - 1;
            std::memcpy(out, quote1, len);
            out[len] = '\0';
        }

        // Extrahiert einen numerischen Wert "key": 12.34 innerhalb [start,end).
        // Laesst 'out' unveraendert (= Default aus der Struct-Initialisierung),
        // wenn der Schluessel fehlt oder nicht parsebar ist.
        static void parse_float_field(const char* start, const char* end, const char* key, float& out) {
            const char* key_pos = find_key(start, end, key);
            if (!key_pos) return;
            const char* colon = strstr_bounded(key_pos, end, ":");
            if (!colon) return;
            char* parse_end = nullptr;
            float val = std::strtof(colon + 1, &parse_end);
            if (parse_end != colon + 1) out = val;
        }

        // Extrahiert ein 6-elementiges Zahlen-Array "key": [a,b,c,d,e,f].
        // Fehlende/kurze Arrays lassen die restlichen Elemente unveraendert.
        static void parse_float_array6(const char* start, const char* end, const char* key,
                                        std::array<float, 6>& out) {
            const char* key_pos = find_key(start, end, key);
            if (!key_pos) return;
            const char* arr_start = nullptr;
            const char* arr_end = nullptr;
            if (!find_array_bounds(key_pos, end, arr_start, arr_end)) return;

            const char* cursor = arr_start;
            for (size_t i = 0; i < 6 && cursor < arr_end; ++i) {
                char* parse_end = nullptr;
                float val = std::strtof(cursor, &parse_end);
                if (parse_end == cursor) break;
                out[i] = val;
                cursor = parse_end;
                while (cursor < arr_end && (*cursor == ',' || *cursor == ' ')) ++cursor;
            }
        }
    };

} // namespace Hexapod