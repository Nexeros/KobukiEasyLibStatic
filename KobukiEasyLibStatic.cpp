#include "KobukiDriver.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include "pch.h"

KobukiDriver::KobukiDriver()
    : serial_handle_(INVALID_HANDLE_VALUE)
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
    std::wstring wport(port_name.begin(), port_name.end());
    // Jeśli nazwa to np. "COM10" lub wyżej, musi być prefiks "\\\\.\\"
    if (wport.find(L"\\\\.\\") == std::wstring::npos) {
        wport = L"\\\\.\\" + wport;
    }

    serial_handle_ = CreateFileW(wport.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, 0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0);

    if (serial_handle_ == INVALID_HANDLE_VALUE) {
        log("Error: Unable to open COM port: " + port_name);
        return false;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(serial_handle_, &dcb)) {
        log("Error: Unable to get COMM state");
        disconnect();
        return false;
    }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(serial_handle_, &dcb)) {
        log("Error: Unable to set COMM state");
        disconnect();
        return false;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 3;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 5;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 10;

    if (!SetCommTimeouts(serial_handle_, &timeouts)) {
        log("Warning: Unable to set COMM timeouts");
    }

    PurgeComm(serial_handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);

    log("Connected to " + port_name);
    return true;
}

void KobukiDriver::disconnect() {
    if (serial_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(serial_handle_);
        serial_handle_ = INVALID_HANDLE_VALUE;
    }
    rx_buffer_.clear();
}

bool KobukiDriver::isConnected() const {
    return serial_handle_ != INVALID_HANDLE_VALUE;
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

    DWORD bytes_written;
    if (!WriteFile(serial_handle_, frame.data(), static_cast<DWORD>(frame.size()), &bytes_written, nullptr)) {
        log("Error: WriteFile failed");
        return false;
    }

    return (bytes_written == frame.size());
}

bool KobukiDriver::readFromSerial() {
    if (!isConnected()) return false;

    uint8_t tmp_buf[256];
    DWORD bytes_read = 0;

    if (ReadFile(serial_handle_, tmp_buf, sizeof(tmp_buf), &bytes_read, nullptr)) {
        if (bytes_read > 0) {
            rx_buffer_.insert(rx_buffer_.end(), tmp_buf, tmp_buf + bytes_read);
            return true;
        }
    }
    else {
        log("Error: ReadFile failed");
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

    // Wg dokumentacji Kobuki: value = 1 / (freq * 0.00000275)
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
    // Nuty
    constexpr int G4 = 392;
    constexpr int Bb4 = 466;
    constexpr int Eb4 = 311;
    constexpr int D5 = 587;
    constexpr int Eb5 = 622;
    constexpr int Gb4 = 370; // F#4 / Gb4

    // Czasy trwania
    constexpr int Q = 480;  // Ćwierćnuta
    constexpr int H = 960;  // Półnuta
    constexpr int DE = 360; // Nuta z kropką
    constexpr int S = 120;  // Szesnastka

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
            if (chunk > 25) {
                Sleep(chunk - 25);
            }
            else {
                Sleep(1);
            }

            remaining -= chunk;
        }
        Sleep(50);
        };

    for (const auto& note : melody) {
        if (!isConnected()) break;

        playLongNote(note.hz, note.ms);
    }
}



void KobukiDriver::parseSensorPacket(const uint8_t* data, KobukiSensors& s) {
    s.timestamp = data[0] | (data[1] << 8);
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
    s.overcurrent = data[14];
}

bool KobukiDriver::parseBuffer(KobukiSensors& out_sensors) {
    bool packet_found = false;

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
            Sleep(10);
            continue;
        }

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

        Sleep(10);
    }

    return info;
}