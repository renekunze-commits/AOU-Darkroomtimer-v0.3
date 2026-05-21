#include <Arduino.h>

#if !defined(ARDUINO_TEENSY41)
#error "Dieser Probe-Sketch ist nur fuer Teensy 4.1 gedacht."
#endif

// Teensyduino exportiert die erkannte externe PSRAM-Groesse in Megabyte.
// Fuer einen reinen Groessencheck lesen wir nur dieses Symbol aus und vermeiden
// bewusst grosse EXTMEM-Testarrays, damit ein Board ohne PSRAM nicht schon vor
// setup() an fruehen Speicherzugriffen scheitert.
extern uint8_t external_psram_size;

namespace {

constexpr uint32_t kSerialBaudRate = 115200u;
constexpr uint32_t kSerialWaitTimeoutMs = 4000u;
constexpr uint32_t kReportIntervalMs = 3000u;

uint32_t lastReportAtMs = 0u;

void printPsramReport() {
  const uint32_t detectedMegabytes = static_cast<uint32_t>(external_psram_size);
  const uint32_t detectedBytes = detectedMegabytes * 1024u * 1024u;

  Serial.println();
  Serial.println("=== Teensy 4.1 PSRAM Probe ===");
  Serial.printf("Erwartet laut Build-Flag: %u MB\n",
                static_cast<unsigned>(DUKATIMER_TEENSY_PSRAM_MB));
  Serial.printf("Erkannt vom Teensy-Core:  %u MB\n",
                static_cast<unsigned>(detectedMegabytes));
  Serial.printf("Erkannt in Byte:          %lu\n",
                static_cast<unsigned long>(detectedBytes));

  if (detectedMegabytes == 0u) {
    Serial.println("STATUS: Kein externes PSRAM erkannt.");
    Serial.println("HINWEIS: Bestueckung, Lötstellen und Teensy-Core/Board-Setup pruefen.");
    return;
  }

  if (detectedMegabytes == static_cast<uint32_t>(DUKATIMER_TEENSY_PSRAM_MB)) {
    Serial.println("STATUS: Erkennung passt zur erwarteten PSRAM-Groesse.");
  } else {
    Serial.println("STATUS: PSRAM erkannt, aber Groesse weicht von der Erwartung ab.");
  }
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(kSerialBaudRate);
  const uint32_t waitStartMs = millis();
  while (!Serial && (millis() - waitStartMs) < kSerialWaitTimeoutMs) {
  }

  printPsramReport();
  lastReportAtMs = millis();
}

void loop() {
  const uint32_t nowMs = millis();

  // Blinkt sichtbar, damit der Probe-Sketch ohne Display sofort als aktiv
  // erkennbar ist.
  digitalWrite(LED_BUILTIN, ((nowMs / 250u) % 2u) != 0u ? HIGH : LOW);

  if ((nowMs - lastReportAtMs) >= kReportIntervalMs) {
    printPsramReport();
    lastReportAtMs = nowMs;
  }
}