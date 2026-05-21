/*
 * SensorManager
 *
 * Version:
 * - Implementierungsschritt 4 der Laufzeitspezifikation
 * - eingefuehrt am 2026-04-24
 * - API-Version 1 fuer den lokalen Sensor-Laufzeitdienst
 *
 * Zweck:
 * - besitzt den sichtbaren Zustand der harten Part2-Sensorpfade
 * - trennt Sensorinitialisierung, Gueltigkeit und Thermik explizit von
 *   ExposureEngine, UI und Head-Ausgabe
 *
 * Historische Einordnung:
 * - v0.9 trennt ExposureEngine und SensorManager bereits sauberer als v0.3
 * - Part2 uebernimmt diese Trennung, bindet den harten Dosis-Sensorpfad lokal
 *   am Teensy und den langsamen OneWire-/Thermikpfad bewusst ueber den ESP32-S3
 *   als Servicezulieferer ein
 *
 * Integrationsgrenze dieses Schritts:
 * - lokaler TSL2561-Dosispfad auf dem Teensy ist aktiv
 * - historische Kernparameter bleiben erhalten (101ms, 1x, fester lokaler Bus)
 * - DS18B20-Thermik darf im aktuellen Hardwarestand weiterhin vom ESP32-S3
 *   eingespeist werden, statt lokal am Teensy gelesen zu werden
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "SensorRuntimeStatus.h"

namespace dukatimer {

constexpr uint16_t kSensorManagerApiVersion = 1;

class SensorManager {
public:
	~SensorManager();

	// tslIntPin: Teensy-Pin, an dem TSL_INT anliegt (INPUT_PULLUP, active-low open-drain).
	// kTslIntPinNone deaktiviert den Interrupt-Pfad und schaltet auf Zeit-Polling zurueck.
	void begin(uint8_t tslIntPin = kTslIntPinNone, uint32_t nowMs = 0);
	void tick(uint32_t nowMs);

	// DS18B20-Pfad: extern gespeist ueber EspServiceLink
	void setDs18b20Initialized(bool initialized);
	void publishDs18b20Temperature(float temperatureCelsius);
	void markDs18b20Fault();
	void setThermalThresholds(float deratingThresholdCelsius, float criticalThresholdCelsius);

	const SensorRuntimeStatus& state() const;

private:
	static constexpr float kThermalDeratingThresholdCelsius = 50.0f;
	static constexpr float kThermalCriticalThresholdCelsius = 60.0f;
	static constexpr uint32_t kTslFreshWindowMs = 250;
	static constexpr uint32_t kTslStaleWindowMs = 1000;
	static constexpr uint32_t kTslPollIntervalMs = 110;
	static constexpr uint32_t kTslInitRetryMs = 1000;
	static constexpr uint32_t kTslIntWatchdogMs = 350;
	static constexpr uint8_t kTslReadFailureThreshold = 3;
	static constexpr uint32_t kTslI2cOperationBudgetMs = 20;
	static constexpr uint32_t kTslI2cClockHz = 400000;
	static constexpr uint8_t kTslDefaultAddress = 0x39;
	static constexpr uint8_t kTslIntPinNone = 255;

	bool initialized_ = false;
	bool localTslReady_ = false;
	bool hasTslSample_ = false;
	uint8_t tslReadFailureCount_ = 0;
	uint8_t tslIntPin_ = kTslIntPinNone;
	float thermalDeratingThresholdCelsius_ = kThermalDeratingThresholdCelsius;
	float thermalCriticalThresholdCelsius_ = kThermalCriticalThresholdCelsius;
	uint32_t lastTslSampleMs_ = 0;
	uint32_t nextTslPollDueMs_ = 0;
	uint32_t lastTslInitAttemptMs_ = 0;
	uint32_t lastTslIntMs_ = 0;
	volatile bool tslIntPending_ = false;
	volatile uint32_t tslIntCapturedMs_ = 0;
	SensorRuntimeStatus state_ = makeUnknownSensorRuntimeStatus();
	TwoWire* tslBus_ = &Wire1;
	uint8_t tslAddress_ = kTslDefaultAddress;

	// Statischer ISR-Zeiger: erlaubt dem freistehenden ISR-Callback Zugriff auf
	// die einzige SensorManager-Instanz, ohne globale Variablen ausserhalb der Klasse.
	static SensorManager* s_tsl2561IsrInstance_;
	static void tslIntIsr();

	void resetState(uint32_t nowMs);
	bool initializeLocalTsl2561(uint32_t nowMs);
	void ensureLocalTsl2561Ready(uint32_t nowMs);
	void pollLocalTsl2561(uint32_t nowMs);
	bool writeTslRegister(uint8_t reg, uint8_t value);
	bool readTslChannels(uint16_t* broadbandOut, uint16_t* irOut);
	float calculateLuxFromChannels(uint16_t broadband, uint16_t ir) const;
	void clearTslInterrupt();
	void releaseTslInterruptBinding();

	// TSL2561-Schreibpfad: privat, da der Sensor lokal am Teensy-I2C-Bus haengt.
	// Externer Schreibzugriff wuerde die lokal gemessenen Echtzeit-Werte ueberschreiben.
	void setTsl2561Initialized(bool initialized);
	void publishTsl2561Sample(float lux,
	                         LuxSampleValidity sampleValidity = LuxSampleValidity::Valid,
	                         Tsl2561DiagnosticReason diagnosticReason = Tsl2561DiagnosticReason::None,
	                         uint32_t nowMs = 0);
	void markTsl2561Fault(Tsl2561DiagnosticReason diagnosticReason =
	                     Tsl2561DiagnosticReason::InitializationFailed);

	void updateTslAging(uint32_t nowMs);
	void updateThermalState();
};

}  // namespace dukatimer