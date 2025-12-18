#pragma once
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <optional>

/// <summary>
/// Struktura zawierająca wszystkie dane z sensorów robota Kobuki.
/// Dane są automatycznie wysyłane przez robota co ~20ms.
/// </summary>
struct KobukiSensors {
    /// <summary>Czas w milisekundach od włączenia robota (wraparound co 65535ms)</summary>
    uint16_t timestamp = 0;
    
    /// <summary>
    /// Stan zderzaków (bitowe):
    /// bit 0 (0x01) = prawy zderzak,
    /// bit 1 (0x02) = środkowy zderzak,
    /// bit 2 (0x04) = lewy zderzak
    /// </summary>
    uint8_t bumper = 0;
    
    /// <summary>
    /// Stan czujników odłączenia kół (bitowe):
    /// bit 0 (0x01) = prawe koło,
    /// bit 1 (0x02) = lewe koło
    /// </summary>
    uint8_t wheel_drop = 0;
    
    /// <summary>
    /// Stan czujników krawędzi/przepaści (bitowe):
    /// bit 0 (0x01) = prawy czujnik,
    /// bit 1 (0x02) = środkowy czujnik,
    /// bit 2 (0x04) = lewy czujnik
    /// </summary>
    uint8_t cliff = 0;
    
    /// <summary>Skumulowane tiki enkodera lewego koła (0-65535, wraparound)</summary>
    uint16_t left_encoder = 0;
    
    /// <summary>Skumulowane tiki enkodera prawego koła (0-65535, wraparound)</summary>
    uint16_t right_encoder = 0;
    
    /// <summary>Wartość PWM lewego silnika (0-255)</summary>
    uint8_t left_pwm = 0;
    
    /// <summary>Wartość PWM prawego silnika (0-255)</summary>
    uint8_t right_pwm = 0;
    
    /// <summary>
    /// Stan przycisków na robocie (bitowe):
    /// bit 0 (0x01) = przycisk B0,
    /// bit 1 (0x02) = przycisk B1,
    /// bit 2 (0x04) = przycisk B2
    /// </summary>
    uint8_t button = 0;
    
    /// <summary>Stan ładowania: 0=niepodłączony, 2=docking station, 6=adapter</summary>
    uint8_t charger = 0;
    
    /// <summary>Napięcie baterii pomnożone przez 10 (np. 167 = 16.7V). Minimum ~14.0V</summary>
    uint8_t battery = 0;
    
    /// <summary>Flagi przeciążenia silników (bitowe)</summary>
    uint8_t overcurrent = 0;
};

/// <summary>
/// Struktura zawierająca informacje o wersji hardware i firmware robota Kobuki
/// oraz unikalny identyfikator urządzenia.
/// </summary>
struct KobukiHardwareInfo {
    /// <summary>Numer patch wersji hardware</summary>
    uint8_t hw_patch = 0;
    
    /// <summary>Numer minor wersji hardware</summary>
    uint16_t hw_minor = 0;
    
    /// <summary>Numer major wersji hardware</summary>
    uint16_t hw_major = 0;
    
    /// <summary>Numer patch wersji firmware</summary>
    uint8_t fw_patch = 0;
    
    /// <summary>Numer minor wersji firmware</summary>
    uint8_t fw_minor = 0;
    
    /// <summary>Numer major wersji firmware</summary>
    uint8_t fw_major = 0;
    
    /// <summary>Unikalny identyfikator robota (3 x 32-bit)</summary>
    uint32_t unique_id[3] = { 0 };
};

/// <summary>
/// Główna klasa sterownika robota Kobuki (TurtleBot 2).
/// Zapewnia interfejs do komunikacji przez port szeregowy, sterowania ruchem,
/// odczytu sensorów i odtwarzania dźwięków.
/// </summary>
class KobukiDriver {
public:
    /// <summary>
    /// Typ funkcji callback do logowania wiadomości.
    /// Przykład: [](const std::string& msg) { std::cout << msg << std::endl; }
    /// </summary>
    using LogCallback = std::function<void(const std::string&)>;

    /// <summary>
    /// Builder do tworzenia i wysyłania wielu komend jednocześnie (batch).
    /// Umożliwia efektywne wysłanie kilku rozkazów w jednej ramce.
    /// </summary>
    class CommandBuilder {
    public:
        /// <summary>
        /// Tworzy nowy builder komend powiązany z instancją KobukiDriver.
        /// </summary>
        /// <param name="driver">Wskaźnik do obiektu KobukiDriver</param>
        explicit CommandBuilder(const KobukiDriver* driver) : driver_(driver) {
            buffer_.reserve(32);
        }

        /// <summary>
        /// Dodaje komendę sterowania bazą (ruch).
        /// </summary>
        /// <param name="speed_mm_s">Prędkość w mm/s (-500 do 500). Wartości dodatnie = do przodu</param>
        /// <param name="radius_mm">
        /// Promień skrętu w mm:
        /// 0 = obrót w miejscu w lewo,
        /// 1 do 32767 = jazda po łuku w lewo,
        /// -1 do -32767 = jazda po łuku w prawo
        /// </param>
        /// <returns>Referencja do buildera dla łańcuchowania komend</returns>
        CommandBuilder& addBaseControl(int16_t speed_mm_s, int16_t radius_mm) {
            buffer_.push_back(0x01); // Header ID
            buffer_.push_back(0x04); // Length
            // Little-endian
            buffer_.push_back(speed_mm_s & 0xFF);
            buffer_.push_back((speed_mm_s >> 8) & 0xFF);
            buffer_.push_back(radius_mm & 0xFF);
            buffer_.push_back((radius_mm >> 8) & 0xFF);
            return *this;
        }

        /// <summary>
        /// Dodaje komendę odtworzenia dźwięku.
        /// </summary>
        /// <param name="freq_hz">Częstotliwość tonu w Hz (np. 440 = A4)</param>
        /// <param name="duration_ms">Czas trwania w milisekundach (0-255)</param>
        /// <returns>Referencja do buildera dla łańcuchowania komend</returns>
        CommandBuilder& addSound(uint16_t freq_hz, uint8_t duration_ms) {
            buffer_.push_back(0x03); // Header ID
            buffer_.push_back(0x03); // Length
            uint16_t val = 0;
            if (freq_hz > 0) {
                // Wzór Kobuki: 1 / (freq * 0.00000275)
                val = static_cast<uint16_t>(1.0 / (static_cast<double>(freq_hz) * 0.00000275));
            }
            buffer_.push_back(val & 0xFF);
            buffer_.push_back((val >> 8) & 0xFF);
            buffer_.push_back(duration_ms);
            return *this;
        }

        /// <summary>
        /// Wysyła wszystkie dodane komendy do robota w jednej ramce.
        /// </summary>
        /// <returns>true jeśli wysłanie się powiodło, false w przeciwnym razie</returns>
        bool send() {
            if (buffer_.empty()) return false;
            return driver_->sendPayload(buffer_);
        }

    private:
        const KobukiDriver* driver_;
        std::vector<uint8_t> buffer_;
    };

    /// <summary>
    /// Tworzy nową instancję sterownika Kobuki.
    /// </summary>
    KobukiDriver();
    
    /// <summary>
    /// Destruktor - automatycznie rozłącza połączenie z robotem.
    /// </summary>
    ~KobukiDriver();

    /// <summary>
    /// Nawiązuje połączenie z robotem przez port szeregowy.
    /// Port jest automatycznie konfigurowany na 115200 baud, 8N1.
    /// </summary>
    /// <param name="port_name">Nazwa portu COM (np. "COM3", "COM10")</param>
    /// <returns>true jeśli połączenie się powiodło, false w przeciwnym razie</returns>
    bool connect(const std::string& port_name);
    
    /// <summary>
    /// Rozłącza połączenie z robotem i zamyka port szeregowy.
    /// </summary>
    void disconnect();
    
    /// <summary>
    /// Sprawdza czy połączenie z robotem jest aktywne.
    /// </summary>
    /// <returns>true jeśli połączono, false w przeciwnym razie</returns>
    bool isConnected() const;

    /// <summary>
    /// Ustawia funkcję callback do logowania wiadomości diagnostycznych.
    /// </summary>
    /// <param name="callback">Funkcja przyjmująca string z wiadomością</param>
    void setLogCallback(LogCallback callback);

    /// <summary>
    /// Wysyła komendę sterowania ruchem robota.
    /// </summary>
    /// <param name="speed_mm_s">
    /// Prędkość liniowa w mm/s (-500 do 500).
    /// Wartości dodatnie = jazda do przodu, ujemne = jazda do tyłu
    /// </param>
    /// <param name="radius_mm">
    /// Promień skrętu w mm:
    /// 0 = obrót w miejscu w lewo,
    /// 1 do 32767 = jazda po łuku w lewo (większy = szerszy łuk),
    /// -1 do -32767 = jazda po łuku w prawo
    /// </param>
    /// <returns>true jeśli komenda została wysłana, false w przeciwnym razie</returns>
    bool setBaseControl(int16_t speed_mm_s, int16_t radius_mm);

    /// <summary>
    /// Odtwarza dźwięk o określonej częstotliwości i czasie trwania.
    /// Dla długich dźwięków należy dzielić na fragmenty ≤255ms.
    /// </summary>
    /// <param name="freq_hz">Częstotliwość dźwięku w Hz (np. 440 = nuta A4)</param>
    /// <param name="duration_ms">Czas trwania w milisekundach (0-255)</param>
    /// <returns>true jeśli komenda została wysłana, false w przeciwnym razie</returns>
    bool playSound(uint16_t freq_hz, uint8_t duration_ms);

    /// <summary>
    /// Odtwarza wbudowaną sekwencję dźwiękową.
    /// </summary>
    /// <param name="sequence_id">
    /// ID sekwencji:
    /// 0 = ON (włączenie),
    /// 1 = OFF (wyłączenie),
    /// 2 = RECHARGE (ładowanie),
    /// 3 = BUTTON (przycisk),
    /// 4 = ERROR (błąd),
    /// 5 = CLEANING START (start czyszczenia),
    /// 6 = CLEANING END (koniec czyszczenia)
    /// </param>
    /// <returns>true jeśli komenda została wysłana, false w przeciwnym razie</returns>
    bool playSequence(int sequence_id);
    
    /// <summary>
    /// Odtwarza melodię Imperial March ze Star Wars.
    /// Funkcja jest blokująca przez czas trwania melodii (~15 sekund).
    /// </summary>
    void playMelody();

    /// <summary>
    /// Tworzy builder do wysyłania wielu komend jednocześnie.
    /// Przykład: createCommandBatch().addBaseControl(100, 0).addSound(440, 100).send();
    /// </summary>
    /// <returns>Nowy obiekt CommandBuilder</returns>
    CommandBuilder createCommandBatch() const {
        return CommandBuilder(this);
    }

    /// <summary>
    /// Odczytuje dane z sensorów robota (non-blocking).
    /// Robot automatycznie wysyła dane co ~20ms. Ta funkcja próbuje odczytać
    /// i sparsować dostępne dane z bufora.
    /// </summary>
    /// <param name="out_sensors">Struktura do wypełnienia danymi sensorów</param>
    /// <returns>true jeśli odczytano nowy pakiet danych, false jeśli brak nowych danych</returns>
    bool dataOnce(KobukiSensors& out_sensors);

    /// <summary>
    /// Pobiera informacje o wersji hardware i firmware robota oraz jego unikalny identyfikator.
    /// Funkcja wysyła zapytanie i czeka na odpowiedź (blocking, ~500ms timeout).
    /// </summary>
    /// <returns>Struktura z informacjami o sprzęcie</returns>
    KobukiHardwareInfo getHardwareInfo();

private:
    HANDLE serial_handle_;
    std::vector<uint8_t> rx_buffer_;
    LogCallback log_callback_;

    static constexpr uint8_t HEADER_1 = 0xAA;
    static constexpr uint8_t HEADER_2 = 0x55;

    void log(const std::string& msg) const;
    bool sendPayload(const std::vector<uint8_t>& payload) const;
    uint8_t calculateChecksum(const std::vector<uint8_t>& data) const;

    bool readFromSerial();
    bool parseBuffer(KobukiSensors& out_sensors);

    void parseSensorPacket(const uint8_t* data, KobukiSensors& out_sensors);
};