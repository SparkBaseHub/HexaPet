#include "drivers/tof_manager.hpp"

namespace Hexapod {

    ToFManager::ToFManager(i2c_inst_t* i2c_port) : i2c_(i2c_port) {}

    void ToFManager::set_sensor2_power(bool enable) {
        gpio_put(PIN_LP_SENSOR2, enable ? 1 : 0);
    }

    bool ToFManager::ping(uint8_t addr_7bit) {
        uint8_t rx_data = 0;
        // 1-Byte Leseversuch mit kurzem Timeout (10 ms)
        int res = i2c_read_timeout_us(i2c_, addr_7bit, &rx_data, 1, false, 10000);
        return (res >= 0);
    }

    bool ToFManager::write_reg16(uint8_t addr_7bit, uint16_t reg, uint8_t val) {
        uint8_t buffer[3] = {
            static_cast<uint8_t>((reg >> 8) & 0xFF),
            static_cast<uint8_t>(reg & 0xFF),
            val
        };
        int res = i2c_write_timeout_us(i2c_, addr_7bit, buffer, 3, false, 20000);
        return (res == 3);
    }

    bool ToFManager::change_i2c_address(uint8_t current_addr_7bit, uint8_t target_addr_7bit) {
        // Register 0x000E im VL53L5CX hält die 7-bit I2C-Adresse (geschoben als 8-bit Wert)
        uint8_t target_8bit = target_addr_7bit << 1;
        bool ok = write_reg16(current_addr_7bit, 0x000E, target_8bit);
        sleep_ms(50);
        return ok;
    }

    void ToFManager::scan_and_print_bus() {
        printf("[I2C Scan] Aktive Adressen auf dem Bus: ");
        bool found = false;
        for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
            if (ping(addr)) {
                printf("0x%02X ", addr);
                found = true;
            }
        }
        if (!found) printf("Keine Geräte gefunden");
        printf("\n");
    }

    bool ToFManager::init() {
        // 1. I2C-Hardware mit 400 kHz Fast-Mode initialisieren
        i2c_init(i2c_, 400000);
        gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(PIN_SDA);
        gpio_pull_up(PIN_SCL);

        // 2. LPn Pin initialisieren
        gpio_init(PIN_LP_SENSOR2);
        gpio_set_dir(PIN_LP_SENSOR2, GPIO_OUT);
        set_sensor2_power(true);

        sleep_ms(100);
        scan_and_print_bus();

        bool has_29 = ping(ADDR_DEFAULT);
        bool has_2a = ping(ADDR_REASSIGNED);

        // FALL 1: Beide Adressen bereits sauber getrennt (z.B. nach Soft-Reboot)
        if (has_2a && has_29) {
            printf("-> Perfekter Zustand: Adressen 0x2A (Links) und 0x29 (Rechts) bereits aktiv.\n");
            left_sensor_ready_ = true;
            right_sensor_ready_ = true;
            return true;
        }

        // FALL 2: Kaltstart oder beide auf 0x29
        printf("-> Adressen nicht getrennt. Führe LPn-Sequenz durch...\n");

        // Schritt A: Sensor 2 per LPn schlafen legen
        set_sensor2_power(false);
        sleep_ms(300);

        // Schritt B: Sensor 1 (Links) umkonfigurieren
        if (ping(ADDR_DEFAULT)) {
            printf("[Setup] Ändere Sensor 1 Adresse von 0x29 -> 0x2A...\n");
            change_i2c_address(ADDR_DEFAULT, ADDR_REASSIGNED);
        }

        if (ping(ADDR_REASSIGNED)) {
            printf("[OK] Sensor Links (0x2A) ansprechbar.\n");
            left_sensor_ready_ = true;
        } else {
            printf("[FEHLER] Sensor Links (0x2A) reagiert nicht!\n");
        }

        // Schritt C: Sensor 2 (Rechts) aufwecken
        set_sensor2_power(true);
        sleep_ms(400); // Warten bis Bootloader von Sensor 2 hochgefahren ist

        if (ping(ADDR_DEFAULT)) {
            printf("[OK] Sensor Rechts (0x29) ansprechbar.\n");
            right_sensor_ready_ = true;
        } else {
            printf("[FEHLER] Sensor Rechts (0x29) reagiert nicht!\n");
        }

        // Schritt D: Abschluss-Validierung
        scan_and_print_bus();
        return (left_sensor_ready_ && right_sensor_ready_);
    }

} // namespace Hexapod