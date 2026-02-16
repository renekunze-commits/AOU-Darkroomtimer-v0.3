# I2C Konfiguration für ESP32-S3-N16R8 DevKitC-1

## Hardware-Verbindung (2 I2C Busse)

### I2C Bus 0: "Chill-Bus" (100kHz) - Nicht zeitkritisch

#### LCD 20x4 (Adresse: 0x27 oder 0x3E)
| I2C Adapter Pin | ESP32-S3 Pin | Beschreibung |
|---|---|---|
| GND | GND | Gemeinsame Masse |
| VCC | 5V (VIN) | Stromversorgung 5V |
| SDA | GPIO 8 | I2C0 Datenleitung |
| SCL | GPIO 9 | I2C0 Taktleitung |

#### TSL2591 (Adresse: 0x29)
| Sensor Pin | ESP32-S3 Pin | Beschreibung |
|---|---|---|
| GND | GND | Gemeinsame Masse |
| VIN | 3.3V | Stromversorgung |
| SDA | GPIO 8 | I2C0 Datenleitung (geteilt mit LCD) |
| SCL | GPIO 9 | I2C0 Taktleitung (geteilt mit LCD) |

**Hinweis:** Das LCD-Modul benötigt 5V Stromversorgung. Der ESP32-S3 verträgt jedoch nur 3,3V an seinen Eingängen. Ein I2C-Level-Converter wird empfohlen.

---

### I2C Bus 1: "Turbo-Bus" (400kHz) - Zeitkritisch!

#### TSL2561 (Adresse: 0x39)
| Sensor Pin | ESP32-S3 Pin | Beschreibung |
|---|---|---|
| GND | GND | Gemeinsame Masse |
| VIN | 3.3V | Stromversorgung |
| SDA | GPIO 47 | I2C1 Datenleitung (EXKLUSIV) |
| SCL | GPIO 48 | I2C1 Taktleitung (EXKLUSIV) |
| INT | GPIO 3 | Interrupt Pin (optional) |

**Zweck:** Dieser Sensor misst die Lichtmenge **während der Belichtung**. Er benötigt einen eigenen Bus, damit NeoPixel-Updates (Interrupts aus!) die Messung nicht stören.

## Software-Konfiguration

### 1. Pins (include/config.h)
```cpp
// Bus 0: LCD + TSL2591
#define PIN_I2C0_SDA     8
#define PIN_I2C0_SCL     9
#define LCD_I2C_ADDR     0x3E
#define TSL2591_I2C_ADDR 0x29

// Bus 1: TSL2561 (zeitkritisch)
#define PIN_I2C1_SDA     47
#define PIN_I2C1_SCL     48
#define TSL2561_I2C_ADDR 0x39
#define PIN_TSL2561_INT  3
```

### 2. Initialisierung (src/main.cpp)
```cpp
// Zwei separate I2C-Instanzen
TwoWire I2C_SLOW = TwoWire(0);  // Bus 0: LCD + TSL2591
TwoWire I2C_FAST = TwoWire(1);  // Bus 1: TSL2561

// Bus 0: 100kHz für LCD (langsam, aber stabil)
I2C_SLOW.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 100000);
I2C_SLOW.setTimeout(100);

// Bus 1: 400kHz für TSL2561 (Fast Mode)
I2C_FAST.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, 400000);
I2C_FAST.setTimeout(50);

// Sensoren mit ihrem eigenen Bus verbinden
Adafruit_TSL2591 tslPre = Adafruit_TSL2591(2591, &I2C_SLOW);
Adafruit_TSL2561 tslLive = Adafruit_TSL2561(TSL2561_ADDR_FLOAT, &I2C_FAST);

// LCD Initialisierung
lcd.init();
lcd.backlight();
```

### 3. Bibliotheken
- **LiquidCrystal_I2C** @ 1.1.2 (marcoschwartz)
- **Wire** (Built-in ESP32 I2C Library)

## ESP32-S3 I2C Ports

Der ESP32-S3 verfügt über 2 I2C-Controller:

| Port | Standard-Pins | Aktuelle Konfiguration | Taktfrequenz | Geräte |
|---|---|---|---|---|
| **I2C0 (Wire)** | SDA=21, SCL=22 | **SDA=8, SCL=9** | 100kHz | LCD, TSL2591 |
| **I2C1 (Wire1)** | SDA=18, SCL=19 | **SDA=47, SCL=48** | 400kHz | TSL2561 |

---

## Warum zwei Busse?

### Das Problem:
- **NeoPixel (256 LEDs):** Deaktiviert Interrupts für ~8ms während `pixels.show()`
- **TSL2561:** Muss während der Belichtung Lichtmenge messen (Closed-Loop)
- **LCD:** Langsam (100kHz), blockiert den Bus

### Die Lösung:
1. **Bus 0 (langsam):** LCD + TSL2591 (nur Vormessung, nicht während Belichtung)
2. **Bus 1 (schnell):** TSL2561 exklusiv - keine Konflikte während Belichtung

### Timing-Strategie während Belichtung:
```cpp
// Phase A: Messen (TSL2561 auf Bus 1, ~13ms)
lux = tslLive.getLuminosity();

// Phase B: Rechnen
if (lux < target) { brightness++; }

// Phase C: LED Update (Interrupts AUS!)
pixels.setBrightness(brightness);
pixels.show();  // 8ms - KEIN I2C möglich!

// Wiederhole...
```

## Debugging

### I2C Scanner Test (test_minimal.cpp)
Das System enthält einen einfachen I2C-Scanner zur Überprüfung angeschlossener Geräte:

```
Wire.beginTransmission(0x20);  bis  Wire.beginTransmission(0x7F)
Wenn endTransmission() == 0: Gerät gefunden
```

### Serial Monitor Output
```
Init I2C Bus (SDA=8, SCL=9)...
I2C OK
Init LCD...
LCD OK (20x4 at 0x27)
```

## Häufige Probleme

| Problem | Ursache | Lösung |
|---|---|---|
| "LCD FEHLT" | I2C Adresse falsch | Überprüfe mit I2C Scanner auf 0x27 |
| Schwarze Rechtecke auf LCD | Kontrast zu hoch | Poti auf Rückseite des Moduls drehen |
| Keine I2C Kommunikation | Pins nicht richtig verbunden | Überprüfe GPIO 8 und GPIO 9 |
| Instabile Verbindung | Schlechte Kontakte / zu lange Kabel | Kabel überprüfen, max. 50cm empfohlen |
| Level-Mismatch Fehler | 3,3V/5V Problem | Level Converter verwenden |

## Überprüfung

✅ I2C Bus auf GPIO 8/9 initialisiert
✅ Wire.setClock(100000) - Standard I2C Speed
✅ LCD Adresse: 0x27
✅ LCD Größe: 20x4
✅ Doppelte Wire.begin() Aufrufe entfernt
✅ Backlight-Steuerung: backlight()/noBacklight()
