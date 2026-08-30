#pragma once
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <array>
#include <cstdint>
#include <cstdio>

namespace Hexapod {

    struct ToFMatrix {
        std::array<uint16_t, 64> distance_mm{};
        std::array<uint8_t, 64> target_status{};
        bool is_valid{false};
    };

    class ToFManager {
    public:
        // Standard-Pins für Servo 2040 Stemma QT Port & LPn
        static constexpr uint8_t PIN_SDA = 20;
        static constexpr uint8_t PIN_SCL = 21;
        static constexpr uint8_t PIN_LP_SENSOR2 = 26;

        static constexpr uint8_t ADDR_DEFAULT = 0x29; // Werksadresse (Sensor Rechts)
        static constexpr uint8_t ADDR_REASSIGNED = 0x2A; // Neue Adresse (Sensor Links)

        ToFManager(i2c_inst_t* i2c_port = i2c0);

        // Initialisiert I2C-Bus und die beiden Sensoren über den LPn-Pin
        bool init();

        // Prüft, ob ein Sensor auf der Adresse antwortet (I2C-Ping)
        bool ping(uint8_t addr_7bit);

        // Schreibt einen Register-Wert auf den I2C-Bus (16-bit Registeradresse für VL53L5CX)
        bool write_reg16(uint8_t addr_7bit, uint16_t reg, uint8_t val);

        // Adresse eines Sensors von 0x29 auf target_addr_7bit umstellen
        bool change_i2c_address(uint8_t current_addr_7bit, uint8_t target_addr_7bit);

        // Debug-Ausgabe des I2C-Busses in die serielle Konsole
        void scan_and_print_bus();

    private:
        i2c_inst_t* i2c_;
        bool left_sensor_ready_{false};
        bool right_sensor_ready_{false};

        void set_sensor2_power(bool enable);
    };

} // namespace Hexapod