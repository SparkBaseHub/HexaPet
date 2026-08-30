#pragma once
#include "core/telemetry.hpp"
#include "drivers/camera_uart.hpp"
#include <cstdio>

namespace Hexapod {

    class MotionLogger {
    public:
        static void start_session(const char* filename = "motion_data.jsonl") {
            char cmd[64];
            snprintf(cmd, sizeof(cmd), "{\"cmd\":\"start_rec\",\"file\":\"%s\"}\n", filename);
            CameraUART::send_string(cmd);
            printf("--> [LOGGER] Aufzeichnung gestartet: %s\n", filename);
        }

        static void stop_session() {
            CameraUART::send_string("{\"cmd\":\"stop_rec\"}\n");
            printf("--> [LOGGER] Aufzeichnung beendet.\n");
        }

        static void log_frame(const TelemetryFrame& frame) {
            char buffer[512];
            int len = snprintf(buffer, sizeof(buffer),
                "{\"t\":%lu,\"body\":[%.1f,%.1f,%.1f],"
                "\"knees\":[[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f]],"
                "\"feet\":[[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f]]}\n",
                frame.timestamp_ms,
                frame.body_pos.x, frame.body_pos.y, frame.body_pos.z,
                frame.knees_pos[0].x, frame.knees_pos[0].y, frame.knees_pos[0].z,
                frame.knees_pos[1].x, frame.knees_pos[1].y, frame.knees_pos[1].z,
                frame.knees_pos[2].x, frame.knees_pos[2].y, frame.knees_pos[2].z,
                frame.knees_pos[3].x, frame.knees_pos[3].y, frame.knees_pos[3].z,
                frame.knees_pos[4].x, frame.knees_pos[4].y, frame.knees_pos[4].z,
                frame.knees_pos[5].x, frame.knees_pos[5].y, frame.knees_pos[5].z,
                frame.feet_pos[0].x, frame.feet_pos[0].y, frame.feet_pos[0].z,
                frame.feet_pos[1].x, frame.feet_pos[1].y, frame.feet_pos[1].z,
                frame.feet_pos[2].x, frame.feet_pos[2].y, frame.feet_pos[2].z,
                frame.feet_pos[3].x, frame.feet_pos[3].y, frame.feet_pos[3].z,
                frame.feet_pos[4].x, frame.feet_pos[4].y, frame.feet_pos[4].z,
                frame.feet_pos[5].x, frame.feet_pos[5].y, frame.feet_pos[5].z
            );

            if (len > 0 && static_cast<size_t>(len) < sizeof(buffer)) {
                CameraUART::send_string(buffer);
            }
        }
    };

} // namespace Hexapod