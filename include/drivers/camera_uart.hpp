#pragma once
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <cstdio>

namespace Hexapod {

    class CameraUART {
    public:
        // 'inline const' statt 'constexpr', da uart1 intern ein reinterpret_cast-Makro ist
        static inline uart_inst_t* const UART_ID = uart1;
        static constexpr uint32_t BAUD_RATE      = 115200;
        static constexpr uint8_t PIN_RX          = 27; // A1 auf Servo 2040 (empfaengt Daten von ESP32 TX)
        static constexpr uint8_t PIN_TX          = 28; // A2 auf Servo 2040 (sendet Befehle an ESP32 RX)

        static void init() {
            // 1. Hardware UART1 mit 115200 Baud starten
            uart_init(UART_ID, BAUD_RATE);

            // 2. Pins A1 (GP27) und A2 (GP28) der Hardware-UART zuweisen
            gpio_set_function(PIN_RX, GPIO_FUNC_UART);
            gpio_set_function(PIN_TX, GPIO_FUNC_UART);

            // 3. Pull-Up auf RX aktivieren (verhindert Geisterbytes bei offenem Pin)
            gpio_pull_up(PIN_RX);

            // 4. Hardware-FIFO aktivieren
            uart_set_fifo_enabled(UART_ID, true);

            printf("--> Camera UART1 initialisiert auf Pins A1 (GP%d/RX) und A2 (GP%d/TX) @ %u Baud.\n",
                   PIN_RX, PIN_TX, BAUD_RATE);
        }

        // Liest verfuegbare Zeichen nicht-blockierend ein
        static bool read_byte(char& out_char) {
            if (uart_is_readable(UART_ID)) {
                out_char = static_cast<char>(uart_getc(UART_ID));
                return true;
            }
            return false;
        }

        // Sendet Zeichenkette zurueck
        static void send_string(const char* str) {
            uart_puts(UART_ID, str);
        }
    };

} // namespace Hexapod