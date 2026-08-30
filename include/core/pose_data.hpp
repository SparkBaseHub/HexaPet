#pragma once

namespace Hexapod {

    // Beispiel-Posen-/Gait-Bibliothek, fest einkompiliert (siehe pose_library.hpp
    // fuer das Schema). Enthaelt nur Posen, die mit der aktuellen Hardware direkt
    // machbar sind (reine IK-Standflaechen + Body-Pose-Orientierung). Werte grob
    // per Reichweiten-Abschaetzung gewaehlt (COXA_LEN=56, FEMUR_LEN=70,
    // TIBIA_LEN=120mm) - bei Bedarf in main.cpp anpassen oder spaeter per KI/
    // UART durch eine neue Bibliothek ersetzen (siehe PoseLibrary::load_from_json,
    // absichtlich quellenunabhaengig gehalten).
    //
    // Spaeter braucht (noch nicht in dieser Bibliothek): IMU (Self-Leveling,
    // Flipping), Stromrueckmeldung (Blind Stepping, Compliant Spring-Leg),
    // ToF-Terrainerkennung (Adaptive Free Gait, Chasm Crossing).
    inline constexpr const char* POSE_LIBRARY_JSON = R"JSON(
{
  "poses": [
    { "name": "home", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0} },

    { "name": "crouched", "type": "stance",
      "stance_radius": 180, "standing_height": -40 },

    { "name": "tall", "type": "stance",
      "stance_radius": 110, "standing_height": -175 },

    { "name": "wide", "type": "stance",
      "stance_radius": 190, "standing_height": -70 },

    { "name": "turn_stance", "type": "stance",
      "stance_radius": 130, "standing_height": -80 },

    { "name": "compact", "type": "raw",
      "tibia": [135,135,135,135,135,135],
      "femur": [170,170,170,170,170,170],
      "coxa":  [135,135,135,135,135,135] },

    { "name": "sleep", "type": "raw",
      "tibia": [90,90,90,90,90,90],
      "femur": [190,190,190,190,190,190],
      "coxa":  [135,135,135,135,135,135] },

    { "name": "surge_forward", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"x":25} },
    { "name": "surge_back", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"x":-25} },

    { "name": "sway_left", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"y":25} },
    { "name": "sway_right", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"y":-25} },

    { "name": "heave_up", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"z":20} },
    { "name": "heave_down", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"z":-20} },

    { "name": "roll_left", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"roll":-15} },
    { "name": "roll_right", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"roll":15} },

    { "name": "pitch_up", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":15} },
    { "name": "pitch_down", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":-15} },

    { "name": "yaw_left", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"yaw":15} },
    { "name": "yaw_right", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"yaw":-15} },

    { "name": "orbit_front", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":12} },
    { "name": "orbit_front_right", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":8.5,"roll":8.5} },
    { "name": "orbit_right", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"roll":12} },
    { "name": "orbit_back_right", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":-8.5,"roll":8.5} },
    { "name": "orbit_back", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":-12} },
    { "name": "orbit_back_left", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":-8.5,"roll":-8.5} },
    { "name": "orbit_left", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"roll":-12} },
    { "name": "orbit_front_left", "type": "stance",
      "stance_radius": 130, "standing_height": -80,
      "body": {"pitch":8.5,"roll":-8.5} }
  ],
  "gaits": [
    { "name": "tripod", "duty_factor": 0.5,
      "phase_offsets": [0, 0.5, 0, 0.5, 0, 0.5] },
    { "name": "wave", "duty_factor": 0.8333,
      "phase_offsets": [0, 0.1667, 0.3333, 0.5, 0.6667, 0.8333] },
    { "name": "ripple", "duty_factor": 0.6667,
      "phase_offsets": [0, 0.3333, 0.6667, 0.3333, 0.6667, 0] }
  ],
  "motions": [
    { "name": "wake_up", "steps": [
        {"pose":"crouched","duration_s":1.5},
        {"pose":"home","duration_s":1.0} ] },
    { "name": "bow", "steps": [
        {"pose":"pitch_down","duration_s":0.8},
        {"pose":"home","duration_s":0.8} ] },

    { "name": "wiggle", "steps": [
        { "type": "pose", "pose": "sway_left",  "duration_s": 0.25 },
        { "type": "pose", "pose": "sway_right", "duration_s": 0.25 },
        { "type": "pose", "pose": "sway_left",  "duration_s": 0.25 },
        { "type": "pose", "pose": "sway_right", "duration_s": 0.25 },
        { "type": "gait", "gait": "wave", "vy": 30, "duration_s": 1.5, "repeat": 2 },
        { "type": "pose", "pose": "home", "duration_s": 0.6 } ] },

    { "name": "moonwalk", "steps": [
        { "type": "gait", "gait": "wave", "vx": -20, "duration_s": 2.0 },
        { "type": "pose", "pose": "home", "duration_s": 0.6 } ] },

    { "name": "waltz", "steps": [
        { "type": "gait", "vx": 15, "vy": 20,  "rotation": 10,  "lean_roll": 8,  "lean_pitch": 5, "duration_s": 0.6 },
        { "type": "gait", "vx": 15, "vy": -20, "rotation": -10, "lean_roll": -8, "lean_pitch": 5, "duration_s": 0.6 },
        { "type": "gait", "vx": 15, "vy": 20,  "rotation": 10,  "lean_roll": 8,  "lean_pitch": 5, "duration_s": 0.6 },
        { "type": "gait", "vx": 15, "vy": -20, "rotation": -10, "lean_roll": -8, "lean_pitch": 5, "duration_s": 0.6 },
        { "type": "pose", "pose": "home", "duration_s": 0.8 } ] },

    { "name": "flow", "steps": [
        { "type": "orbit", "amplitude": 12, "revolutions": 3, "duration_s": 3.6 },
        { "type": "pose", "pose": "home", "duration_s": 0.6 } ] },

    { "name": "hiphop", "steps": [
        { "type": "pose", "pose": "heave_down", "duration_s": 0.15 },
        { "type": "pose", "pose": "heave_up",   "duration_s": 0.15 },
        { "type": "pose", "pose": "heave_down", "duration_s": 0.15 },
        { "type": "pose", "pose": "heave_up",   "duration_s": 0.15 },
        { "type": "pose", "pose": "yaw_left",   "duration_s": 0.2 },
        { "type": "pose", "pose": "yaw_right",  "duration_s": 0.2 },
        { "type": "pose", "pose": "home",       "duration_s": 0.5 } ] },

    { "name": "robot", "steps": [
        { "type": "pose", "pose": "roll_left",  "duration_s": 0.1 },
        { "type": "pose", "pose": "yaw_right",  "duration_s": 0.1 },
        { "type": "pose", "pose": "roll_right", "duration_s": 0.1 },
        { "type": "pose", "pose": "yaw_left",   "duration_s": 0.1 },
        { "type": "pose", "pose": "home",       "duration_s": 0.3 } ] }
  ]
}
)JSON";

} // namespace Hexapod