#include "KobukiDriver.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>

// Linuxowe nagłówki
#include <fcntl.h>      // open
#include <unistd.h>     // write, read, close, usleep
#include <termios.h>    // obsługa portu szeregowego
#include <sys/ioctl.h>
#include <cstring>      // strerror
#include <cerrno>
#include <thread>       // std::this_thread::sleep_for
#include <chrono>       // std::chrono::milliseconds
#include <cstdio>
// Pomocnicza funkcja zastępująca Sleep()
void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

KobukiDriver::KobukiDriver()
    : serial_fd_(-1) // -1 oznacza brak połączenia (odpowiednik INVALID_HANDLE_VALUE)
{
    rx_buffer_.reserve(1024);
}

KobukiDriver::~KobukiDriver() {
    disconnect();
}

void KobukiDriver::log(const std::string& msg) const {
    if (log_callback_) {
        log_callback_(msg);
    }
}

bool KobukiDriver::connect(const std::string& port_name) {
    disconnect();

    // Otwarcie portu na Linuxie
    // O_RDWR - czytanie i pisanie
    // O_NOCTTY - port nie staje się terminalem sterującym procesu
    // O_NDELAY - nie blokuj przy otwieraniu (ważne dla niektórych kabli)
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_fd_ == -1) {
        log("Error: Unable to open port: " + port_name + " (" + std::string(strerror(errno)) + ")");
        return false;
    }

    // Usunięcie flagi O_NDELAY, żeby operacje były domyślnie blokujące (chyba że zmienimy w termios)
    fcntl(serial_fd_, F_SETFL, 0);

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        log("Error: tcgetattr failed");
        disconnect();
        return false;
    }

    // Ustawienie prędkości 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Konfiguracja 8N1 (8 data bits, No parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;     // Bez parzystości
    tty.c_cflag &= ~CSTOPB;     // 1 bit stopu
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8 bitów danych

    // Wyłączenie kontroli przepływu sprzętowego
    tty.c_cflag &= ~CRTSCTS;

    // Włączenie odbiornika i trybu lokalnego
    tty.c_cflag |= (CLOCAL | CREAD);

    // Tryb RAW (surowy) - wyłączenie przetwarzania znaków
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Wyłącz flow control programowy
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR); // Nie zamieniaj \r na \n
    tty.c_oflag &= ~OPOST; // Wyłącz przetwarzanie wyjścia

    // Ustawienie timeoutów (Non-blocking read z minimalnym oczekiwaniem)
    // VMIN = 0, VTIME = 0 -> Read wraca natychmiast (jak polling)
    // Aby działało podobnie do Twojego Windowsowego ReadTotalTimeoutConstant,
    // ustawimy VTIME = 1 (0.1 sekundy timeoutu), żeby nie obciążać CPU w pętli.
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0; // Tryb w pełni nieblokujący, zgodny z logiką dataOnce

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        log("Error: tcsetattr failed");
        disconnect();
        return false;
    }

    // Czyszczenie buforów (odpowiednik PurgeComm)
    tcflush(serial_fd_, TCIOFLUSH);

    log("Connected to " + port_name);
    return true;
}

void KobukiDriver::disconnect() {
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    rx_buffer_.clear();
}

bool KobukiDriver::isConnected() const {
    return serial_fd_ != -1;
}

void KobukiDriver::setLogCallback(LogCallback callback) {
    log_callback_ = callback;
}

uint8_t KobukiDriver::calculateChecksum(const std::vector<uint8_t>& data) const {
    uint8_t cs = 0;
    for (uint8_t byte : data) {
        cs ^= byte;
    }
    return cs;
}

bool KobukiDriver::sendPayload(const std::vector<uint8_t>& payload) const {
    if (!isConnected()) return false;

    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 4);

    frame.push_back(HEADER_1);
    frame.push_back(HEADER_2);
    frame.push_back(static_cast<uint8_t>(payload.size()));

    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(calculateChecksum(payload));

    // ZMIANA: write zamiast WriteFile
    ssize_t bytes_written = write(serial_fd_, frame.data(), frame.size());

    if (bytes_written < 0) {
        log("Error: write failed");
        return false;
    }

    return (static_cast<size_t>(bytes_written) == frame.size());
}

bool KobukiDriver::readFromSerial() {
    if (!isConnected()) return false;

    uint8_t tmp_buf[256];

    // ZMIANA: read zamiast ReadFile
    ssize_t bytes_read = read(serial_fd_, tmp_buf, sizeof(tmp_buf));

    if (bytes_read > 0) {
        rx_buffer_.insert(rx_buffer_.end(), tmp_buf, tmp_buf + bytes_read);
        return true;
    }
    else if (bytes_read < 0) {
        // EAGAIN oznacza brak danych w trybie non-blocking, to nie jest krytyczny błąd
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            log("Error: read failed: " + std::string(strerror(errno)));
        }
    }

    return false;
}

//-------------------------------- Command Implementations -------------------------------

bool KobukiDriver::setBaseControl(int16_t speed_mm_s, int16_t radius_mm) {
    std::vector<uint8_t> body;
    body.reserve(6);
    body.push_back(0x01);
    body.push_back(0x04);
    body.push_back(speed_mm_s & 0xFF);
    body.push_back((speed_mm_s >> 8) & 0xFF);
    body.push_back(radius_mm & 0xFF);
    body.push_back((radius_mm >> 8) & 0xFF);
    return sendPayload(body);
}

bool KobukiDriver::playSound(uint16_t freq_hz, uint8_t duration_ms) {
    std::vector<uint8_t> body;
    body.push_back(0x03);
    body.push_back(0x03);
    uint16_t sound_value = 0;
    if (freq_hz > 0) {
        sound_value = static_cast<uint16_t>(1.0 / (static_cast<double>(freq_hz) * 0.00000275));
    }
    body.push_back(sound_value & 0xFF);
    body.push_back((sound_value >> 8) & 0xFF);
    body.push_back(duration_ms);
    return sendPayload(body);
}

bool KobukiDriver::playSequence(int sequence_id) {
    std::vector<uint8_t> body = { 0x04, 0x01, static_cast<uint8_t>(sequence_id) };
    return sendPayload(body);
}

void KobukiDriver::playMelody() {
    constexpr int G4 = 392;
    constexpr int Bb4 = 466;
    constexpr int Eb4 = 311;
    constexpr int D5 = 587;
    constexpr int Eb5 = 622;
    constexpr int Gb4 = 370;

    constexpr int Q = 480;
    constexpr int H = 960;
    constexpr int DE = 360;
    constexpr int S = 120;

    struct Note { int hz; int ms; };

    Note melody[] = {
        {G4, Q}, {G4, Q}, {G4, Q},
        {Eb4, DE}, {Bb4, S}, {G4, Q},
        {Eb4, DE}, {Bb4, S}, {G4, H},

        {D5, Q}, {D5, Q}, {D5, Q},
        {Eb5, DE}, {Bb4, S}, {Gb4, Q},
        {Eb4, DE}, {Bb4, S}, {G4, H}
    };

    auto playLongNote = [&](int hz, int duration) {
        int remaining = duration;
        while (remaining > 0) {
            int chunk = (remaining > 255) ? 255 : remaining;
            playSound(static_cast<uint16_t>(hz), static_cast<uint8_t>(chunk));

            // ZMIANA: sleep_ms zamiast Sleep
            if (chunk > 25) {
                sleep_ms(chunk - 25);
            }
            else {
                sleep_ms(1);
            }
            remaining -= chunk;
        }
        sleep_ms(50);
    };

    for (const auto& note : melody) {
        if (!isConnected()) break;
        playLongNote(note.hz, note.ms);
    }
}

void KobukiDriver::parseSensorPacket(const uint8_t* data, KobukiSensors& s) {
    /*s.timestamp = data[0] | (data[1] << 8);
    s.bumper = data[2];
    s.wheel_drop = data[3];
    s.cliff = data[4];
    s.left_encoder = data[5] | (data[6] << 8);
    s.right_encoder = data[7] | (data[8] << 8);
    s.left_pwm = data[9];
    s.right_pwm = data[10];
    s.button = data[11];
    s.charger = data[12];
    s.battery = data[13];
    s.overcurrent = data[14];*/
    // 1. Najpierw odczytujemy surowe bajty
    uint8_t enc_l_low = data[5];
    uint8_t enc_l_high = data[6];
    uint8_t enc_r_low = data[7];
    uint8_t enc_r_high = data[8];

    // 2. Składamy je w całość (jawne rzutowanie dla pewności)
    s.left_encoder = static_cast<uint16_t>(enc_l_low) | (static_cast<uint16_t>(enc_l_high) << 8);
    s.right_encoder = static_cast<uint16_t>(enc_r_low) | (static_cast<uint16_t>(enc_r_high) << 8);

    // Reszta pól
    s.timestamp = data[0] | (data[1] << 8);
    s.bumper = data[2];
    s.wheel_drop = data[3];
    s.cliff = data[4];

    // PWM i pozostałe
    s.left_pwm = data[9];
    s.right_pwm = data[10];
    s.button = data[11];
    s.charger = data[12];
    s.battery = data[13];
    s.overcurrent = data[14];

    // --- DEBUG: Wypisz surowe bajty co ~50 pakietów żeby nie zalać konsoli ---
    /*static int debug_counter = 0;
    if (debug_counter++ > 20) {
        printf("\n[DEBUG RAW] Bat: %d (%.1fV) | EncL: %02X %02X | EncR: %02X %02X | Drop: %02X\n",
            s.battery, s.battery/10.0f,
            enc_l_high, enc_l_low,
            enc_r_high, enc_r_low,
            s.wheel_drop);
        debug_counter = 0;
    }*/
}

bool KobukiDriver::parseBuffer(KobukiSensors& out_sensors) {
    bool packet_found = false;

    // Logika ta sama co w wersji Windows
    while (rx_buffer_.size() >= 4) {
        if (rx_buffer_[0] != HEADER_1 || rx_buffer_[1] != HEADER_2) {
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
        }
        uint8_t payload_len = rx_buffer_[2];
        size_t full_packet_size = static_cast<size_t>(3) + payload_len + 1;

        if (rx_buffer_.size() < full_packet_size) {
            return packet_found;
        }
        std::vector<uint8_t> chunk_to_check(rx_buffer_.begin() + 2, rx_buffer_.begin() + full_packet_size - 1);
        uint8_t calculated_cs = calculateChecksum(chunk_to_check);
        uint8_t received_cs = rx_buffer_[full_packet_size - 1];

        if (calculated_cs == received_cs) {
            size_t current_idx = 3;
            size_t end_idx = current_idx + payload_len;

            while (current_idx < end_idx) {
                if (current_idx + 2 > end_idx) break;

                uint8_t sub_id = rx_buffer_[current_idx];
                uint8_t sub_len = rx_buffer_[current_idx + 1];

                if (current_idx + 2 + sub_len > end_idx) break;

                const uint8_t* sub_data = &rx_buffer_[current_idx + 2];

                if (sub_id == 0x01 && sub_len == 15) {
                    parseSensorPacket(sub_data, out_sensors);
                    packet_found = true;
                }

                current_idx += (static_cast<unsigned long long>(2) + sub_len);
            }
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + full_packet_size);
        }
        else {
            log("Checksum error detected.");
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 2);
        }
    }
    return packet_found;
}

bool KobukiDriver::dataOnce(KobukiSensors& out_sensors) {
    readFromSerial();
    return parseBuffer(out_sensors);
}

KobukiHardwareInfo KobukiDriver::getHardwareInfo() {
    KobukiHardwareInfo info = { 0 };
    bool got_hw = false;
    bool got_fw = false;
    bool got_uuid = false;

    std::vector<uint8_t> req_body = { 0x09, 0x02, 0x0B, 0x00 };
    rx_buffer_.clear();

    if (!sendPayload(req_body)) {
        log("Error: Failed to send Hardware Request.");
        return info;
    }

    for (int i = 0; i < 50; ++i) {
        readFromSerial();

        if (rx_buffer_.size() < 4) {
            sleep_ms(10); // ZMIANA: sleep_ms
            continue;
        }

        // ... (reszta logiki parsowania bez zmian) ...
        // Ze względu na długość bloku kodu, skróciłem tylko komentarzem,
        // ale w pełnym pliku wklej tutaj całą treść z oryginalnej funkcji getHardwareInfo,
        // pamiętając jedynie o zmianie Sleep(10) na sleep_ms(10).

        // --- SKOPIOWANY KOD PARSOWANIA Z ORYGINAŁU ---
        while (rx_buffer_.size() >= 4) {
            if (rx_buffer_[0] != HEADER_1 || rx_buffer_[1] != HEADER_2) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            uint8_t payload_len = rx_buffer_[2];
            size_t full_packet_size = 3 + payload_len + 1;

            if (rx_buffer_.size() < full_packet_size) {
                break;
            }

            std::vector<uint8_t> chunk_to_check(rx_buffer_.begin() + 2, rx_buffer_.begin() + full_packet_size - 1);
            if (calculateChecksum(chunk_to_check) == rx_buffer_[full_packet_size - 1]) {

                size_t current_idx = 3;
                size_t end_idx = current_idx + payload_len;

                while (current_idx < end_idx) {
                    if (current_idx + 2 > end_idx) break;

                    uint8_t sub_id = rx_buffer_[current_idx];
                    uint8_t sub_len = rx_buffer_[current_idx + 1];

                    if (current_idx + 2 + sub_len > end_idx) break;

                    const uint8_t* data = &rx_buffer_[current_idx + 2];

                    if (sub_id == 0x0A && sub_len >= 3) {
                        info.hw_patch = data[0];
                        info.hw_minor = data[1];
                        info.hw_major = data[2];
                        got_hw = true;
                    }
                    if (sub_id == 0x0B && sub_len >= 3) {
                        info.fw_patch = data[0];
                        info.fw_minor = data[1];
                        info.fw_major = data[2];
                        got_fw = true;
                    }
                    if (sub_id == 0x13 && sub_len == 12) {
                        info.unique_id[0] = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
                        info.unique_id[1] = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
                        info.unique_id[2] = data[8] | (data[9] << 8) | (data[10] << 16) | (data[11] << 24);
                        got_uuid = true;
                    }
                    current_idx += (2 + sub_len);
                }
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + full_packet_size);

                if (got_hw && got_fw && got_uuid) {
                    return info;
                }
            }
            else {
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 2);
            }
        }
        // --- KONIEC KOPIOWANIA ---

        sleep_ms(10);
    }
    return info;
}