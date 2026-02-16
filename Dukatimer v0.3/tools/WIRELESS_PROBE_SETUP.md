# ESP-NOW Wireless Probe - Setup Anleitung

## Überblick

Die Wireless Probe ermöglicht es, einen TSL2591 Lichtsensor kabellos mit dem Dukatimer-Hauptgerät zu verbinden. Dies ist ideal für flexible Messungen ohne Verkabelung.

## Hardware Anforderungen

### Sender (Wireless Probe)
- **ESP32-C3 SuperMini** oder **ESP32-S3 Mini**
- **TSL2591** Lux-Sensor (I2C)
- **LiPo Batterie** 3.7V (optional, z.B. 500-1000mAh)
- **TP4056 Lademodul** (optional, für USB-Ladung)
- Gehäuse (z.B. 3D-Druck)

### Empfänger
- Dukatimer Hauptgerät (bereits integriert)

## Verkabelung ESP32-C3 SuperMini + TSL2591

```
TSL2591         ESP32-C3
=========       =========
VIN     ------> 3V3
GND     ------> GND
SDA     ------> GPIO 8
SCL     ------> GPIO 9
```

**Hinweis:** Bei anderen ESP32-Varianten können die I2C-Pins abweichen!

## Software Setup

### Option 1: Neues PlatformIO-Projekt erstellen

1. **Neues Projekt anlegen:**
   ```
   Name: Dukatimer_Wireless_Probe
   Board: ESP32-C3 SuperMini (oder dein Board)
   Framework: Arduino
   ```

2. **platformio.ini konfigurieren:**
   ```ini
   [env:esp32-c3-supermini]
   platform = espressif32
   board = esp32-c3-devkitm-1
   framework = arduino
   monitor_speed = 115200
   
   lib_deps = 
       adafruit/Adafruit TSL2591 Library @ ^1.4.1
       adafruit/Adafruit Unified Sensor @ ^1.1.6
   ```

3. **Code kopieren:**
   - Kopiere den Inhalt von `wireless_probe_sender.cpp` nach `src/main.cpp`

4. **Kompilieren & Hochladen:**
   ```
   pio run -t upload
   pio device monitor
   ```

### Option 2: Als Standalone Arduino Sketch

Wenn du die Arduino IDE bevorzugst:

1. Öffne Arduino IDE
2. Installiere Board Support: `Tools > Board > Boards Manager > ESP32` (by Espressif)
3. Installiere Libraries:
   - `Adafruit TSL2591 Library`
   - `Adafruit Unified Sensor`
4. Wähle dein Board: `Tools > Board > ESP32 Arduino > ESP32C3 Dev Module`
5. Kopiere Code aus `wireless_probe_sender.cpp` in neuen Sketch
6. Upload!

## Verwendung

### 1. Sender starten
- Verbinde ESP32-C3 mit USB (oder Batterie)
- Öffne Serial Monitor (115200 Baud)
- Du solltest sehen:
  ```
  =================================
  DUKATIMER WIRELESS PROBE
  ESP-NOW TSL2591 Sender
  =================================
  
  Starting I2C (SDA=8, SCL=9)...
  Initializing TSL2591...
  TSL2591 OK
  Gain: MED, Integration: 100ms
  Starting WiFi Station Mode...
  MAC Address: XX:XX:XX:XX:XX:XX
  Initializing ESP-NOW...
  ESP-NOW OK
  
  =================================
  READY TO TRANSMIT
  =================================
  
  [000001] TX OK  | Lux:    45.23 | IR:   123 | Full:   456
  [000002] TX OK  | Lux:    45.18 | IR:   122 | Full:   454
  ```

### 2. Dukatimer konfigurieren
1. Hauptgerät einschalten
2. Drücke **BACK** Button → Setup-Menü öffnet
3. Navigiere zu **"SENSOR SOURCE"** (letzter Eintrag)
4. Toggle auf **"WIRELESS (W)"**
5. Drücke **BACK** zum Speichern

### 3. Testen
- Im Hauptmenü sollte nun die Lux-Anzeige die Werte des Wireless-Sensors zeigen
- Halte eine Taschenlampe vor den Sensor → Wert sollte sofort steigen
- Wenn "NO SIGNAL" erscheint: Sender ist aus oder zu weit entfernt

## Troubleshooting

### Problem: "TSL2591 not found!"
- **Ursache:** I2C-Verdrahtung falsch oder Sensor defekt
- **Lösung:** 
  - Prüfe Verkabelung (besonders SDA/SCL)
  - Teste mit I2C-Scanner: `Tools/i2c_scanner.cpp`
  - TSL2591 Adresse sollte `0x29` sein

### Problem: "TX FAIL"
- **Ursache:** WiFi nicht initialisiert oder Broadcast-Peer fehlerhaft
- **Lösung:**
  - ESP32 neustarten
  - Prüfe ob `WiFi.mode(WIFI_STA)` erfolgreich war

### Problem: Empfänger zeigt "NO SIGNAL"
- **Ursache:** Pakete kommen nicht an (>2 Sekunden Timeout)
- **Lösung:**
  - Sender näher an Empfänger bringen (ESP-NOW: ca. 100m Freifeld)
  - Prüfe ob Sender läuft (Serial Monitor)
  - Stelle sicher, dass `REMOTE_MAGIC` identisch ist (0xD4)

### Problem: Werte "springen" stark
- **Ursache:** Sensor-Gain zu hoch oder Glättung fehlt
- **Lösung:**
  - Im Sender `TSL2591_GAIN_LOW` statt `MED` verwenden
  - Optional: Glättung im Sender implementieren (wie in HW_Sensors.cpp)

## Erweiterte Optionen

### Batteriebetrieb optimieren
Um Batterielebensdauer zu verlängern:

1. **Senderate reduzieren:**
   ```cpp
   #define SEND_INTERVAL_MS 500  // Statt 200ms
   ```

2. **WiFi Power Save aktivieren:**
   ```cpp
   // In setup() nach WiFi.begin():
   esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
   ```

3. **Integration Time verkürzen:**
   ```cpp
   tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // Schneller = weniger Strom
   ```

### Reichweite erhöhen
- **Antenne:** Externe Antenne anbringen (falls Board-Support vorhanden)
- **TX Power:** Erhöhen (Achtung: mehr Stromverbrauch!)
  ```cpp
  esp_wifi_set_max_tx_power(84);  // Max: 84 (=20dBm)
  ```

### Kanal festlegen (für bessere Performance)
Wenn du viele WiFi-Netze in der Umgebung hast, hilft ein fester Kanal:

**Sender:**
```cpp
esp_wifi_set_promiscuous(true);
esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
esp_wifi_set_promiscuous(false);
```

**Empfänger:** (in `HW_Wireless.cpp` bereits vorbereitet, auskommentiert)

## Gehäuse Design

Ein einfaches 3D-Druck Gehäuse sollte enthalten:
- Platz für ESP32-C3 Board
- Aussparung für TSL2591 (transparent!)
- Halterung für LiPo-Akku
- USB-Zugang für Ladung
- Schalter für Ein/Aus (optional)

Beispiel-Maße:
- 60mm x 40mm x 20mm
- TSL2591 Sensor nach "vorne" zeigend
- USB-Port nach "unten"

## Sicherheitshinweise

⚠️ **LiPo Batterie:**
- Niemals über 4.2V laden
- Niemals unter 3.0V entladen
- Verwende immer ein Schutzmodul (TP4056)
- Nicht kurzschließen!

⚠️ **ESP-NOW:**
- Keine Verschlüsselung aktiviert → Daten sind öffentlich
- Für Dunkelkammer-Anwendung unkritisch, aber beachten bei anderen Projekten

## Weitere Informationen

- ESP-NOW Dokumentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
- TSL2591 Datasheet: https://ams.com/tsl25911
- PlatformIO Docs: https://docs.platformio.org/

## Changelog

- **v1.0** (2026-02-12): Initiale Version
  - Basic ESP-NOW Sender/Empfänger
  - Menü-Integration im Dukatimer
  - Fallback auf Kabel-Sensor

---

**Viel Erfolg beim Aufbau! 🚀**
