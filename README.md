# KobukiEasyLibStatic

Biblioteka statyczna C++ do łatwej komunikacji z robotem Kobuki (TurtleBot 2) przez port szeregowy na platformie Windows.

## 📋 Opis

KobukiEasyLibStatic to prosta i łatwa w użyciu biblioteka umożliwiająca kontrolę robota Kobuki. Zapewnia interfejs do sterowania ruchem, odczytywania sensorów, odtwarzania dźwięków oraz pobierania informacji o sprzęcie.

## ✨ Funkcje

- 🎮 **Sterowanie ruchem** - kontrola prędkości i promienia skrętu
- 📊 **Odczyt sensorów** - zderzaki, enkodery kół, bateria, przyciski
- 🔊 **System dźwiękowy** - odtwarzanie tonów i sekwencji melodii
- 🔧 **Informacje o sprzęcie** - wersje firmware i hardware, unikalny identyfikator
- 📦 **Batch commands** - możliwość wysyłania wielu komend naraz
- 📝 **System logowania** - opcjonalny callback do monitorowania komunikacji

## 🚀 Wymagania

- Windows
- Kompilator C++ z obsługą C++17
- Robot Kobuki podłączony przez USB (FTDI serial converter)

## 📖 Podstawowe użycie

### Połączenie z robotem

```cpp
#include "KobukiDriver.h"
#include <iostream>

int main() {
    KobukiDriver kobuki;
    
    // Opcjonalnie: ustaw callback do logowania
    kobuki.setLogCallback([](const std::string& msg) {
        std::cout << "[Kobuki] " << msg << std::endl;
    });
    
    // Połącz z robotem (port COM może się różnić)
    if (!kobuki.connect("COM3")) {
        std::cerr << "Nie udało się połączyć z robotem!" << std::endl;
        return 1;
    }
    
    std::cout << "Połączono z Kobuki!" << std::endl;
    
    // Twój kod sterujący...
    
    kobuki.disconnect();
    return 0;
}
```

### Sterowanie ruchem

```cpp
// Jazda do przodu z prędkością 200 mm/s
kobuki.setBaseControl(200, 0);
Sleep(2000);

// Zatrzymaj
kobuki.setBaseControl(0, 0);

// Obrót w lewo (promień 0 = obrót w miejscu)
kobuki.setBaseControl(100, 1);
Sleep(1000);

// Jazda po łuku (promień 500mm)
kobuki.setBaseControl(150, 500);
Sleep(2000);

// Specjalne wartości promienia:
// 0     - obrót w miejscu w lewo
// 1-32767 - jazda po łuku (dodatnie = lewo)
// -32767 do -1 - jazda po łuku (ujemne = prawo)
```

### Odczyt sensorów

```cpp
KobukiSensors sensors;

while (kobuki.isConnected()) {
    if (kobuki.dataOnce(sensors)) {
        // Sprawdź zderzaki (bitowe)
        if (sensors.bumper & 0x01) std::cout << "Prawy zderzak aktywny!" << std::endl;
        if (sensors.bumper & 0x02) std::cout << "Środkowy zderzak aktywny!" << std::endl;
        if (sensors.bumper & 0x04) std::cout << "Lewy zderzak aktywny!" << std::endl;
        
        // Odczytaj enkodery
        std::cout << "Lewy enkoder: " << sensors.left_encoder << std::endl;
        std::cout << "Prawy enkoder: " << sensors.right_encoder << std::endl;
        
        // Sprawdź stan baterii (wartość * 0.1V)
        float voltage = sensors.battery / 10.0f;
        std::cout << "Bateria: " << voltage << "V" << std::endl;
        
        // Przyciski (B0, B1, B2)
        if (sensors.button & 0x01) std::cout << "Przycisk B0 wciśnięty!" << std::endl;
        if (sensors.button & 0x02) std::cout << "Przycisk B1 wciśnięty!" << std::endl;
        if (sensors.button & 0x04) std::cout << "Przycisk B2 wciśnięty!" << std::endl;
    }
    
    Sleep(50); // 20 Hz odczyt
}
```

### Odtwarzanie dźwięków

```cpp
// Odtwórz ton 440 Hz (A4) przez 200ms
kobuki.playSound(440, 200);
Sleep(250);

// Odtwórz sekwencję wbudowaną
// 0 = ON, 1 = OFF, 2 = RECHARGE, 3 = BUTTON, 4 = ERROR
// 5 = CLEANING START, 6 = CLEANING END
kobuki.playSequence(0);

// Odtwórz melodię (Imperial March z Star Wars)
kobuki.playMelody();
```

### Batch commands (wiele komend naraz)

```cpp
// Wyślij ruch i dźwięk jednocześnie
auto batch = kobuki.createCommandBatch();
batch.addBaseControl(150, 0)
     .addSound(500, 100)
     .send();
```

### Informacje o sprzęcie

```cpp
KobukiHardwareInfo info = kobuki.getHardwareInfo();

std::cout << "Hardware: " << (int)info.hw_major << "." 
          << (int)info.hw_minor << "." << (int)info.hw_patch << std::endl;
          
std::cout << "Firmware: " << (int)info.fw_major << "." 
          << (int)info.fw_minor << "." << (int)info.fw_patch << std::endl;
          
std::cout << "UUID: " << std::hex 
          << info.unique_id[0] << "-"
          << info.unique_id[1] << "-"
          << info.unique_id[2] << std::endl;
```

## 📚 API Reference

### Klasa `KobukiDriver`

#### Połączenie

| Metoda | Opis |
|--------|------|
| `bool connect(const std::string& port_name)` | Nawiązuje połączenie z robotem przez podany port COM |
| `void disconnect()` | Rozłącza połączenie z robotem |
| `bool isConnected() const` | Zwraca `true` jeśli połączenie jest aktywne |

#### Sterowanie

| Metoda | Parametry | Opis |
|--------|-----------|------|
| `bool setBaseControl(int16_t speed_mm_s, int16_t radius_mm)` | speed: -500 do 500 mm/s<br>radius: -32767 do 32767 mm | Ustawia prędkość i promień skrętu |
| `CommandBuilder createCommandBatch()` | - | Tworzy builder do wysyłania wielu komend naraz |

#### Dźwięk

| Metoda | Parametry | Opis |
|--------|-----------|------|
| `bool playSound(uint16_t freq_hz, uint8_t duration_ms)` | freq: częstotliwość w Hz<br>duration: 0-255 ms | Odtwarza ton o podanej częstotliwości |
| `bool playSequence(int sequence_id)` | id: 0-6 | Odtwarza wbudowaną sekwencję dźwiękową |
| `void playMelody()` | - | Odtwarza melodię (Imperial March) |

#### Sensory

| Metoda | Parametry | Opis |
|--------|-----------|------|
| `bool dataOnce(KobukiSensors& out_sensors)` | out_sensors: struktura do wypełnienia | Odczytuje dane z sensorów (non-blocking) |
| `KobukiHardwareInfo getHardwareInfo()` | - | Pobiera informacje o sprzęcie i firmware |

#### Inne

| Metoda | Opis |
|--------|------|
| `void setLogCallback(LogCallback callback)` | Ustawia funkcję callback do logowania wiadomości |

### Struktura `KobukiSensors`

| Pole | Typ | Opis |
|------|-----|------|
| `timestamp` | `uint16_t` | Czas w ms od włączenia robota |
| `bumper` | `uint8_t` | Flagi zderzaków: bit0=prawy, bit1=środek, bit2=lewy |
| `wheel_drop` | `uint8_t` | Flagi odłączenia kół: bit0=prawe, bit1=lewe |
| `cliff` | `uint8_t` | Czujniki krawędzi: bit0=prawy, bit1=środek, bit2=lewy |
| `left_encoder` | `uint16_t` | Wartość enkodera lewego koła (0-65535, wrap-around) |
| `right_encoder` | `uint16_t` | Wartość enkodera prawego koła (0-65535, wrap-around) |
| `left_pwm` | `uint8_t` | PWM lewego silnika |
| `right_pwm` | `uint8_t` | PWM prawego silnika |
| `button` | `uint8_t` | Przyciski: bit0=B0, bit1=B1, bit2=B2 |
| `charger` | `uint8_t` | Stan ładowania (docking/adapter) |
| `battery` | `uint8_t` | Napięcie baterii × 10 (np. 167 = 16.7V) |
| `overcurrent` | `uint8_t` | Flagi przeciążenia silników |

### Struktura `KobukiHardwareInfo`

| Pole | Typ | Opis |
|------|-----|------|
| `hw_patch`, `hw_minor`, `hw_major` | `uint8_t/uint16_t` | Wersja hardware |
| `fw_patch`, `fw_minor`, `fw_major` | `uint8_t` | Wersja firmware |
| `unique_id[3]` | `uint32_t` | Unikalny identyfikator robota |

## 💡 Przykłady zaawansowane

### Reaktywne zachowanie - unikanie przeszkód

```cpp
KobukiSensors sensors;

while (kobuki.isConnected()) {
    if (kobuki.dataOnce(sensors)) {
        if (sensors.bumper) {
            // Zderzak aktywny - cofnij i obróć
            kobuki.setBaseControl(-100, 0);
            Sleep(500);
            kobuki.setBaseControl(100, 1); // Obrót w lewo
            Sleep(800);
        } else {
            // Brak przeszkody - jedź do przodu
            kobuki.setBaseControl(200, 0);
        }
    }
    Sleep(50);
}
```

### Monitorowanie dystansu na podstawie enkoderów

```cpp
KobukiSensors sensors, prev_sensors;
int total_distance = 0;
const double TICKS_PER_MM = 0.085; // Aproksymacja

kobuki.dataOnce(prev_sensors); // Pierwszy odczyt

while (kobuki.isConnected()) {
    if (kobuki.dataOnce(sensors)) {
        // Oblicz różnicę (uwzględnij wrap-around)
        int16_t left_diff = sensors.left_encoder - prev_sensors.left_encoder;
        int16_t right_diff = sensors.right_encoder - prev_sensors.right_encoder;
        
        int avg_ticks = (left_diff + right_diff) / 2;
        total_distance += avg_ticks * TICKS_PER_MM;
        
        std::cout << "Dystans: " << total_distance << " mm" << std::endl;
        
        prev_sensors = sensors;
    }
    Sleep(50);
}
```

## ⚠️ Uwagi

- Port szeregowy musi być skonfigurowany na **115200 baud, 8N1**
- Robot Kobuki automatycznie wysyła dane sensorów co ~20ms
- Enkodery używają 16-bitowych liczników z wrap-around (0-65535)
- Wartość baterii poniżej 14.0V oznacza niski stan naładowania
- Promień skrętu 0 oznacza obrót w miejscu
- Długie tony należy dzielić na fragmenty ≤255ms (patrz `playMelody()`)

## 📝 Licencja

Projekt open-source. Sprawdź plik LICENSE dla szczegółów.

## 🔗 Linki

- [Protokół komunikacji Kobuki](https://yujinrobot.github.io/kobuki/enAppendixProtocolSpecification.html)
- [Dokumentacja robota Kobuki](https://kobuki.readthedocs.io/)

## 🤝 Wkład

Zgłaszaj błędy i propozycje ulepszeń przez GitHub Issues!