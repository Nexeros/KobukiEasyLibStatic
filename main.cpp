#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstdio>
#include "KobukiDriver.h"

// =============================================================
// Helpery: Zarządzanie Terminalem (Non-blocking I/O)
// =============================================================

// Klasa RAII, która włącza tryb RAW na starcie i wyłącza przy wyjściu
class TerminalManager {
    struct termios original_termios;
    bool is_raw = false;

public:
    TerminalManager() {
        // Pobierz aktualne ustawienia
        tcgetattr(STDIN_FILENO, &original_termios);
    }

    ~TerminalManager() {
        disableRawMode();
    }

    void enableRawMode() {
        if (is_raw) return;
        struct termios raw = original_termios;

        // Wyłącz ICANON (buforowanie linii - znak dostępny od razu)
        // Wyłącz ECHO (nie wypisuj wciśniętego klawisza na ekran)
        raw.c_lflag &= ~(ICANON | ECHO);

        // Ustaw zmiany natychmiast
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        is_raw = true;
    }

    void disableRawMode() {
        if (!is_raw) return;
        // Przywróć oryginalne ustawienia (ważne, żeby terminal nie zwariował po wyjściu)
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
        is_raw = false;
    }
};

// Globalny manager (żeby zadziałał destruktor przy return)
TerminalManager termManager;

// Szybki kbhit oparty na ioctl (pyta kernel ile bajtów czeka)
int _kbhit() {
    int bytesWaiting;
    ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

// Prosty getch (czyta 1 bajt)
char _getch() {
    char buf = 0;
    if (read(STDIN_FILENO, &buf, 1) < 0) {
        return 0;
    }
    return buf;
}

// GetTickCount64() na Linuxie
uint64_t GetTickCount64() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

// Sleep() na Linuxie
void Sleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// =============================================================
// Koniec helperów
// =============================================================

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

int main() {
    KobukiDriver kobuki;

    // 1. Konfiguracja portu
    std::string portName = "/dev/ttyUSB0";

    // Wyłączamy tryb RAW na chwilę, żeby pobrać nazwę portu (jeśli user chce wpisać)
    // Ale w tym przykładzie zrobimy auto-wybór dla prostoty, żeby nie mieszać trybów.
    std::cout << "--- KOBUKI CONSOLE CONTROLLER (LINUX) ---\n";
    std::cout << "Target Port: " << portName << "\n";
    std::cout << "Connecting..." << std::endl;

    if (!kobuki.connect(portName)) {
        std::cerr << "CRITICAL: Failed to connect to " << portName << "!" << std::endl;
        std::cerr << "Hint: Check permissions (sudo usermod -a -G dialout $USER)." << std::endl;
        return 1;
    }

    std::cout << "Fetching Hardware Info..." << std::endl;
    auto hw_info = kobuki.getHardwareInfo();

    std::cout << "Hardware Version: " << (int)hw_info.hw_major << "."
        << (int)hw_info.hw_minor << "." << (int)hw_info.hw_patch << "\n";
    std::cout << "UDID: " << std::hex << hw_info.unique_id[0] << std::dec << "...\n";

    // Sygnał startowy
    kobuki.playSequence(0);

    std::cout << "\nCONTROLS:\n";
    std::cout << " [W] Forward   [S] Backward\n";
    std::cout << " [A] Left (Rot)[D] Right (Rot)\n";
    std::cout << " [SPACE] Stop  [E] Melody\n";
    std::cout << " [Q] Quit\n";
    std::cout << "------------------------------------------\n";

    // --- WŁĄCZAMY TRYB RAW KLAWIATURY ---
    // Od teraz printf działa normalnie, ale klawisze nie pojawiają się na ekranie
    // i nie trzeba klikać ENTER.
    termManager.enableRawMode();

    KobukiSensors sensors;
    std::cout << "Rozmiar struktury: " << sizeof(KobukiSensors) << " bajtow" << std::endl;
    bool running = true;

    int base_speed = 0;
    int base_radius = 0;
    bool dirty_control = false;

    while (running) {
        // --- 1. Odczyt danych ---
        static auto last_print = GetTickCount64();

        if (kobuki.dataOnce(sensors)) {
            if (sensors.bumper > 0) {
                base_speed = 0;
                base_radius = 0;
                dirty_control = true;
                // Wyłączamy RAW na chwilę dla ładnego wypisania błędu nową linią
                termManager.disableRawMode();
                std::cout << "\n[ALERT] BUMPER HIT! EMERGENCY STOP!\n";
                termManager.enableRawMode();
                kobuki.playSequence(5);
            }
        }

        // Odświeżanie UI co 100ms
        if (GetTickCount64() - last_print > 100) {
            float voltage = sensors.battery / 10.0f;
            int pct = (sensors.battery - 132) * 100 / (167 - 132);
            if (pct < 0) pct = 0; if (pct > 100) pct = 100;

            // Używamy printf, bo jest bezpieczniejszy w trybie RAW niż cout przy odświeżaniu linii
            printf("\rBAT: %3d%% (%.1fV) | ENC: L %5d R %5d | SPD: %4d mm/s ",
                pct, voltage, sensors.left_encoder, sensors.right_encoder, base_speed);
            fflush(stdout); // Ważne! Wypchnij bufor na ekran

            last_print = GetTickCount64();
        }

        // --- 2. Obsługa klawiatury (TERAZ NIE BLOKUJE) ---
        if (_kbhit()) {
            char ch = _getch();
            ch = tolower(ch);
            dirty_control = true;

            switch (ch) {
            case 'w': base_speed = 150; base_radius = 0; break;
            case 's': base_speed = -150; base_radius = 0; break;
            case 'a': base_speed = 100; base_radius = 1; break;
            case 'd': base_speed = 100; base_radius = -1; break;
            case ' ': base_speed = 0;   base_radius = 0; break;
            case 'e': kobuki.playSequence(3); dirty_control = false; break;
            case 'q': running = false; base_speed = 0; base_radius = 0; break;
            default: dirty_control = false; break;
            }
        }

        // --- 3. Sterowanie ---
        static auto last_control = GetTickCount64();
        if (dirty_control || (GetTickCount64() - last_control > 50)) {
            kobuki.setBaseControl(base_speed, base_radius);
            dirty_control = false;
            last_control = GetTickCount64();
        }

        Sleep(5);
    }

    // Sprzątanie (destruktor termManager przywróci konsolę)
    kobuki.setBaseControl(0, 0);
    Sleep(50);
    kobuki.playSequence(1);
    kobuki.disconnect();

    // Jawne wyłączenie RAW, żeby komunikat końcowy był ładny
    termManager.disableRawMode();
    std::cout << "\n\nSession Terminated.\n";

    return 0;
}