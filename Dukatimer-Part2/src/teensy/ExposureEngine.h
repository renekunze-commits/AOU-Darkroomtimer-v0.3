/*
 * ExposureEngine
 *
 * Version:
 * - Implementierungsschritt 4 der Laufzeitspezifikation
 * - eingefuehrt am 2026-04-24
 * - API-Version 2 fuer den lokalen Exposure-Laufzeitdienst
 *
 * Zweck:
 * - besitzt die sichtbare Belichtungs-Zustandsmaschine der Teensy-Seite
 * - fuehrt die lokale Dosisintegration fuer den Dose-Modus aus
 * - kapselt prädiktiven Shutoff, Sensor-Watchdog und thermischen Hard-Stop
 *
 * Integrationsgrenze dieses Schritts:
 * - die Engine regelt den Belichtungsverlauf und Sicherheitsreaktionen
 * - die konkrete Head-Farbabbildung bleibt in einer getrennten Schicht
 */
#pragma once

#include <Arduino.h>

#include "ExposureRuntimeState.h"
#include "HeadTimingConstants.h"
#include "SensorRuntimeStatus.h"

namespace dukatimer {

constexpr uint16_t kExposureEngineApiVersion = 1;

/*
 * ExposureEngine
 *
 * Zweck:
 * - fuehrt die physische Belichtungs-Zustandsmaschine der Teensy-Seite aus
 * - integriert im Dose-Modus reale Messwerte zu Luxsekunden
 * - entscheidet ueber Sicherheitsreaktionen wie Sensor-Watchdog, Plausibilitaet,
 *   thermisches Derating und Hard-Stop
 *
 * Architekturgrenze:
 * - die Engine bleibt strikt einheitenbasiert (Zeit, Lux, Luxsekunden,
 *   Temperatur) und kennt keine UI-, EV- oder modusspezifischen Semantiken
 */
class ExposureEngine {
public:
	// Lebenszyklus- und Beobachtungsschnittstelle: Sensoren und Head-Timing werden
	// von aussen eingespeist; die Engine liest niemals selbst Hardware.
	void begin(uint32_t nowMs = 0);
	void tick(uint32_t nowMs);
	void observeSensorStatus(const SensorRuntimeStatus& sensorStatus, uint32_t nowMs = 0);
	void observeHeadPresentDurationUs(uint32_t durationUs);
	uint32_t currentHeadBusLatencyMs() const;
	uint32_t currentPredictiveShutoffLeadMs() const;
	void setThermalProtectionConfig(float deratingStartCelsius, float hardStopCelsius);

	// Kontrolloberflaeche fuer Workflows oder Glue-Code. Jeder Start wird gegen
	// den aktuellen Laufzeitzustand und die Sicherheitsvorbedingungen geprueft.
	bool startTimeExposure(float requestedSeconds, bool skipPreWait = false);
	bool startDoseExposure(float targetDoseLuxSeconds, bool skipPreWait = false);
	bool stopGracefully();
	bool abortWithFault(ExposureFaultReason faultReason);
	bool pause();
	bool resume();
	bool acknowledgeDone();
	bool clearLatchedFault();

	// Sichtbarer Laufzeitzustand fuer Snapshot, UI und Workflows.
	const ExposureRuntimeState& state() const;

private:
	static constexpr uint32_t kPreWaitMs = 200;
	static constexpr uint32_t kPostWaitMs = 400;
	static constexpr uint32_t kMinimumTimeExposureMs = 50;
	static constexpr uint32_t kMinimumDoseFallbackMs = 50;
	static constexpr uint32_t kDoseSampleWatchdogMs = 420;
	static constexpr uint32_t kZeroLuxGraceMs = 140;
	static constexpr uint32_t kZeroLuxFaultWindowMs = 260;
	static constexpr float kMinimumPlausibleLux = 0.02f;
	static constexpr float kMinimumPredictiveLux = 0.01f;
	static constexpr float kMaximumIntegratedDoseDtSeconds = 0.300f;
	static constexpr float kThermalDeratingStartCelsius = 50.0f;
	static constexpr float kThermalHardStopCelsius = 60.0f;
	static constexpr float kThermalMinimumOutputLimit = 0.35f;
	static constexpr uint32_t kHeadLatencyImmediateRiseToleranceMs = 2;
	static constexpr uint32_t kHeadLatencySpikeBandMs = 3;
	static constexpr uint8_t kHeadLatencySpikeConfirmSamples = 3;

	bool initialized_ = false;
	ExposureRuntimeState state_ = makeIdleExposureRuntimeState();
	// observedSensors_ ist immer der zuletzt publizierte Sensorschnappschuss.
	// Dadurch bleibt die Engine deterministisch und unabhaengig von direktem
	// Sensorzugriff oder ISR-getriebenen Seiteneffekten.
	SensorRuntimeStatus observedSensors_ = makeUnknownSensorRuntimeStatus();
	uint32_t phaseStartedMs_ = 0;
	uint32_t exposureStartedMs_ = 0;
	uint32_t timeBudgetMs_ = 0;
	bool hasUsableDoseSample_ = false;
	float lastPlausibleDoseLux_ = 0.0f;
	uint32_t lastObservedDoseSampleTimestampMs_ = 0;
	uint32_t lastIntegratedDoseSampleTimestampMs_ = 0;
	uint32_t invalidDoseSampleSinceMs_ = 0;
	uint32_t zeroLuxSinceMs_ = 0;
	double integratedDoseLuxSeconds_ = 0.0;
	uint32_t doseFallbackStartedMs_ = 0;
	uint32_t doseFallbackBudgetMs_ = 0;
	uint32_t runtimeHeadBusLatencyMs_ = kHeadBusLatencyMs;
	uint32_t pendingHeadBusLatencyMs_ = 0;
	uint8_t pendingHeadBusLatencyConfirmations_ = 0;
	float thermalDeratingStartCelsius_ = kThermalDeratingStartCelsius;
	float thermalHardStopCelsius_ = kThermalHardStopCelsius;

	bool canAcceptStartRequest() const;
	void beginStartRequest(ExposureControlMode controlMode, float targetDose,
	                       uint32_t requestedTimeBudgetMs, bool skipPreWait, uint32_t nowMs);
	void enterPhase(ExposurePhase phase, uint32_t nowMs);
	void enterIdle(uint32_t nowMs);
	void enterPostWait(uint32_t nowMs);
	void enterDone(uint32_t nowMs);
	void enterFault(ExposureFaultReason faultReason, uint32_t nowMs);
	void resetDoseLoopTracking();
	void clearDoseSensorFallback();
	void updateThermalRuntime(const SensorRuntimeStatus& sensorStatus);
	bool hasThermalHardStop(const SensorRuntimeStatus& sensorStatus) const;
	bool hasFatalDoseSensorFault(const SensorRuntimeStatus& sensorStatus) const;
	bool activateDoseSensorFallback(ExposureFaultReason fallbackReason, uint32_t nowMs);
	bool updateDoseSensorFallback(uint32_t nowMs);
	bool updateDoseClosedLoop(const SensorRuntimeStatus& sensorStatus, uint32_t nowMs);
	bool isDoseSampleUsable(const Tsl2561RuntimeStatus& sensorStatus) const;
	uint32_t estimateDoseSampleTimestampMs(uint32_t nowMs, const Tsl2561RuntimeStatus& sensorStatus) const;
	float thermalOutputLimitForTemperature(float temperatureCelsius) const;
	uint32_t predictiveShutoffLeadMs() const;
	void updateDerivedTelemetry(uint32_t nowMs);
};

}  // namespace dukatimer