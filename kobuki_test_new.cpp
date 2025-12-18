#include <iostream>
#include <conio.h>   // _kbhit, _getch
#include <windows.h> // Sleep
#include <iomanip>
#include <string>
#include "KobukiDriver.h"

// Pomocnicze funkcje wyświetlania
const char* getChargingState(uint8_t state) {
    switch (state) {
    case 0:  return "DISCHARGING";
    case 2:  return "DOCK (FULL)";
    case 6:  return "DOCK (CHARGING)";
    case 18: return "ADAPTER (FULL)";
    case 22: return "ADAPTER (CHARGING)";
    default: return "UNKNOWN";
    }
}

void clearLine() {
    std::cout << "\r                                                                               \r";
}

int main() {
    KobukiDriver kobuki;

    // 1. Konfiguracja portu
    std::string portName = "COM3";
    std::cout << "--- KOBUKI CONSOLE CONTROLLER ---\n";
    std::cout << "Enter COM port (default COM3): ";

    // Prosta obsługa wejścia z domyślną wartością
    if (_kbhit()) {
        std::string input;
        std::getline(std::cin, input);
        if (!input.empty()) portName = input;
    }
    else {
        std::cout << portName << " (Auto-selected after 1s)\n";
        Sleep(1000);
    }

    if (!kobuki.connect(portName)) {
        std::cerr << "CRITICAL: Failed to connect to " << portName << "!" << std::endl;
        return 1;
    }

    std::cout << "Fetching Hardware Info..." << std::endl;
    auto hw_info = kobuki.getHardwareInfo();

    std::cout << "Hardware Version: " << (int)hw_info.hw_major << "."
        << (int)hw_info.hw_minor << "." << (int)hw_info.hw_patch << "\n";

    std::cout << "Firmware Version: " << (int)hw_info.fw_major << "."
        << (int)hw_info.fw_minor << "." << (int)hw_info.fw_patch << "\n";

    std::cout << "UDID: " << std::hex
        << hw_info.unique_id[0] << "-"
        << hw_info.unique_id[1] << "-"
        << hw_info.unique_id[2] << std::dec << "\n";

    // Sygnał startowy
    kobuki.playSequence(0);

    std::cout << "\nCONTROLS:\n";
    std::cout << " [W] Forward   [S] Backward\n";
    std::cout << " [A] Left (Rot)[D] Right (Rot)\n";
    std::cout << " [SPACE] Stop  [E] Melody\n";
    std::cout << " [C] COMBO TEST (Move + Sound in 1 packet)\n";
    std::cout << " [Q] Quit\n";
    std::cout << "------------------------------------------\n";

    KobukiSensors sensors;
    bool running = true;

    // Zmienne do logiki sterowania
    int base_speed = 0;
    int base_radius = 0;
    bool dirty_control = false; // Czy trzeba wysłać nową komendę ruchu?

    while (running) {
        // --- 1. Odczyt danych (non-blocking) ---
        // Wywołujemy spinOnce tyle razy ile się da, żeby opróżnić bufor
        // ale wypisujemy na ekran rzadziej (np. co 100ms)
        static auto last_print = GetTickCount64();

        if (kobuki.dataOnce(sensors)) {
            // Logika bezpieczeństwa - Zderzak
            if (sensors.bumper > 0) {
                base_speed = 0;
                base_radius = 0;
                dirty_control = true;
                kobuki.playSequence(5); // Error sound
                std::cout << "\n[ALERT] BUMPER HIT! EMERGENCY STOP!\n";
            }
        }

        // Odświeżanie UI co 100ms
        if (GetTickCount64() - last_print > 100) {
            float voltage = sensors.battery / 10.0f;
            int pct = (sensors.battery - 132) * 100 / (167 - 132);
            if (pct < 0) pct = 0; if (pct > 100) pct = 100;

            std::cout << "\rBAT: " << std::setw(3) << pct << "% (" << voltage << "V) | "
                << "CHG: " << std::setw(10) << getChargingState(sensors.charger) << " | "
                << "BTN: " << (int)sensors.button << " | "
                << "ENC: L" << std::setw(5) << sensors.left_encoder
                << " R" << std::setw(5) << sensors.right_encoder
                << " | SPD: " << base_speed << " mm/s"
                << std::flush;
            last_print = GetTickCount64();
        }

        // --- 2. Obsługa klawiatury ---
        if (_kbhit()) {
            char ch = _getch();
            ch = tolower(ch);

            dirty_control = true; // Domyślnie zakładamy, że zmieniamy ruch

            switch (ch) {
            case 'w': base_speed = 150; base_radius = 0; break;
            case 's': base_speed = -150; base_radius = 0; break;
            case 'a': base_speed = 100; base_radius = 1; break; // 1 = obrót w miejscu w lewo
            case 'd': base_speed = 100; base_radius = -1; break; // -1 = obrót w miejscu w prawo
            case ' ': base_speed = 0;   base_radius = 0; break;

            case 'e':
                kobuki.playSequence(3); // Jakaś melodyjka
                dirty_control = false; // To nie zmienia ruchu ciągłego
                break;

            case 'c': {
                // TEST KOMPOZYTOWY: Wysyłamy jedną ramkę zawierającą i ruch i dźwięk
                // Zmniejsza to latency i gwarantuje synchronizację
                std::cout << "\n[TEST] Sending Composite Packet (Drive + Beep)...\n";

                kobuki.createCommandBatch()
                    .addBaseControl(100, 0) // Jedź
                    .addSound(880, 200)     // Trąb (A5)
                    .send();

                base_speed = 100; // Aktualizujemy stan lokalny
                base_radius = 0;
                dirty_control = false; // Już wysłaliśmy ręcznie
                break;
            }

            case 'q':
            case 27: // ESC
                running = false;
                base_speed = 0;
                base_radius = 0;
                break;
            case 'f':
                std::cout << "\n[MUSIC] Playing Imperial March...\n";
                // Uwaga: To zablokuje sterowanie na czas trwania melodii, 
                // ponieważ funkcja używa Sleep() wewnątrz.
                kobuki.playMelody();
                std::cout << "[MUSIC] Done.\n";

                dirty_control = false;
                break;
            default:
                dirty_control = false;
                break;
            }
        }

        // --- 3. Wysyłanie komend ruchu ---
        // Kobuki wymaga ciągłego odświeżania komendy ruchu (watchdog ~1s).
        // Wysyłamy, jeśli zmienił się stan (dirty) LUB co np. 100ms (watchdog keep-alive)
        static auto last_control = GetTickCount64();
        if (dirty_control || (GetTickCount64() - last_control > 50)) {
            kobuki.setBaseControl(base_speed, base_radius);
            dirty_control = false;
            last_control = GetTickCount64();
        }

        // --- 4. Taktowanie pętli ---
        // Sleep(1) jest wystarczający, żeby CPU nie szalał, 
        // a pętla kręciła się szybko dla odczytu bufora.
        Sleep(5);
    }

    // Wyjście
    kobuki.setBaseControl(0, 0);
    Sleep(50);
    kobuki.playSequence(1); // Off sound
    kobuki.disconnect();
    std::cout << "\n\nSession Terminated.\n";

    return 0;
}