#pragma once

// Usuwamy Windows.h, dodajemy standardowe biblioteki
#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <optional>
#pragma pack(push, 1)
// Struktury danych pozostają bez zmian
struct KobukiSensors {
    uint16_t timestamp = 0;
    uint8_t bumper = 0;
    uint8_t wheel_drop = 0;
    uint8_t cliff = 0;
    uint16_t left_encoder = 0;
    uint16_t right_encoder = 0;
    uint8_t left_pwm = 0;
    uint8_t right_pwm = 0;
    uint8_t button = 0;
    uint8_t charger = 0;
    uint8_t battery = 0;
    uint8_t overcurrent = 0;
};
#pragma pack(pop)
struct KobukiHardwareInfo {
    uint8_t hw_patch = 0;
    uint16_t hw_minor = 0;
    uint16_t hw_major = 0;
    uint8_t fw_patch = 0;
    uint8_t fw_minor = 0;
    uint8_t fw_major = 0;
    uint32_t unique_id[3] = { 0 };
};

class KobukiDriver {
public:
    using LogCallback = std::function<void(const std::string&)>;

    // CommandBuilder bez zmian
    class CommandBuilder {
    public:
        explicit CommandBuilder(const KobukiDriver* driver) : driver_(driver) {
            buffer_.reserve(32);
        }

        CommandBuilder& addBaseControl(int16_t speed_mm_s, int16_t radius_mm) {
            buffer_.push_back(0x01);
            buffer_.push_back(0x04);
            buffer_.push_back(speed_mm_s & 0xFF);
            buffer_.push_back((speed_mm_s >> 8) & 0xFF);
            buffer_.push_back(radius_mm & 0xFF);
            buffer_.push_back((radius_mm >> 8) & 0xFF);
            return *this;
        }

        CommandBuilder& addSound(uint16_t freq_hz, uint8_t duration_ms) {
            buffer_.push_back(0x03);
            buffer_.push_back(0x03);
            uint16_t val = 0;
            if (freq_hz > 0) {
                val = static_cast<uint16_t>(1.0 / (static_cast<double>(freq_hz) * 0.00000275));
            }
            buffer_.push_back(val & 0xFF);
            buffer_.push_back((val >> 8) & 0xFF);
            buffer_.push_back(duration_ms);
            return *this;
        }

        bool send() {
            if (buffer_.empty()) return false;
            return driver_->sendPayload(buffer_);
        }

    private:
        const KobukiDriver* driver_;
        std::vector<uint8_t> buffer_;
    };

    KobukiDriver();
    ~KobukiDriver();

    // port_name na Linux to np. "/dev/ttyUSB0"
    bool connect(const std::string& port_name);
    void disconnect();
    bool isConnected() const;
    void setLogCallback(LogCallback callback);

    bool setBaseControl(int16_t speed_mm_s, int16_t radius_mm);
    bool playSound(uint16_t freq_hz, uint8_t duration_ms);
    bool playSequence(int sequence_id);
    void playMelody();

    CommandBuilder createCommandBatch() const {
        return CommandBuilder(this);
    }

    bool dataOnce(KobukiSensors& out_sensors);
    KobukiHardwareInfo getHardwareInfo();

private:
    // ZMIANA: Zamiast HANDLE mamy int (file descriptor)
    int serial_fd_;
    std::vector<uint8_t> rx_buffer_;
    LogCallback log_callback_;

    static constexpr uint8_t HEADER_1 = 0xAA;
    static constexpr uint8_t HEADER_2 = 0x55;

    void log(const std::string& msg) const;

    // sendPayload musi być publiczny lub przyjaźnić CommandBuilder (tu zostawiłem bez zmian, zakładając friend w Cpp lub public w zależności od Twojej preferencji, w oryginale metoda była private a CommandBuilder miał dostęp)
    // W Twoim kodzie CommandBuilder używa tej metody, więc musi mieć do niej dostęp.
    // Dodałem friend, żeby działało jak w oryginale:
    friend class CommandBuilder;
    bool sendPayload(const std::vector<uint8_t>& payload) const;

    uint8_t calculateChecksum(const std::vector<uint8_t>& data) const;

    bool readFromSerial();
    bool parseBuffer(KobukiSensors& out_sensors);
    void parseSensorPacket(const uint8_t* data, KobukiSensors& out_sensors);
};