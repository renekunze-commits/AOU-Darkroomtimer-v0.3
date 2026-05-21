/*
 * ExposureEngine
 *
 * Laufzeitrolle:
 * - setzt Start-/Stop-Anforderungen in eine robuste physische Belichtungsfolge um
 * - haelt alle sicherheitsrelevanten Reaktionen fuer Dose- und Time-Modus an
 *   einem Ort zusammen
 * - liefert einen reinen Runtime-State, den Workflows und UI nur noch lesen
 */

#include "ExposureEngine.h"

namespace dukatimer {

namespace {

// Historische Aufrufer denken teilweise in Sekunden, die Engine selbst fuehrt
// ihre Zeitgrenzen intern aber ausschliesslich in Millisekunden.
uint32_t secondsToMilliseconds(float seconds) {
	if (seconds <= 0.0f) {
		return 0;
	}

	return static_cast<uint32_t>(seconds * 1000.0f + 0.5f);
}

}  // namespace

void ExposureEngine::begin(uint32_t nowMs) {
	// begin() ist idempotent, damit das Wiring den Dienst gefahrlos erneut
	// initialisieren kann, ohne bestehende Laufzeitobjekte zu duplizieren.
	if (initialized_) {
		return;
	}

	initialized_ = true;
	enterIdle(nowMs);
}

void ExposureEngine::observeSensorStatus(const SensorRuntimeStatus& sensorStatus, uint32_t nowMs) {
	// Die Engine konsumiert nur bereits aufbereitete Sensorzustandsdaten. Dadurch
	// bleiben Sensorzugriff, ISR-Details und I2C-Fehlerbehandlung komplett ausserhalb
	// der Belichtungslogik.
	if (!initialized_) {
		return;
	}

	if (nowMs == 0) {
		nowMs = millis();
	}

	observedSensors_ = sensorStatus;
	updateThermalRuntime(sensorStatus);
	hasUsableDoseSample_ = isDoseSampleUsable(sensorStatus.tsl2561);
	if (hasUsableDoseSample_) {
		lastObservedDoseSampleTimestampMs_ = estimateDoseSampleTimestampMs(nowMs, sensorStatus.tsl2561);
	}
}

void ExposureEngine::observeHeadPresentDurationUs(uint32_t durationUs) {
	if (!initialized_ || durationUs == 0) {
		return;
	}

	// Der gemessene present()-Wert fliesst als konservative Laufzeitbasis in den
	// praediktiven Shutoff ein. Abwaerts wird nur langsam nachgefuehrt; aufwaerts
	// werden groessere Spruenge erst nach bestaetigten Wiederholungen uebernommen,
	// damit einzelne ISR-/Bus-Spikes den Abschaltvorlauf nicht sofort aufblasen.
	const uint32_t measuredMs = (durationUs + 999u) / 1000u;
	const uint32_t clampedMs = measuredMs > kHeadPresentDurationClampMs ? kHeadPresentDurationClampMs : measuredMs;
	if (clampedMs == 0) {
		return;
	}

	if (clampedMs <= runtimeHeadBusLatencyMs_) {
		pendingHeadBusLatencyMs_ = 0;
		pendingHeadBusLatencyConfirmations_ = 0;
		const uint32_t smoothedMs = ((runtimeHeadBusLatencyMs_ * 15u) + clampedMs) / 16u;
		runtimeHeadBusLatencyMs_ = smoothedMs < kHeadBusLatencyMs ? kHeadBusLatencyMs : smoothedMs;
		return;
	}

	const uint32_t riseMs = clampedMs - runtimeHeadBusLatencyMs_;
	if (riseMs <= kHeadLatencyImmediateRiseToleranceMs) {
		runtimeHeadBusLatencyMs_ = clampedMs;
		pendingHeadBusLatencyMs_ = 0;
		pendingHeadBusLatencyConfirmations_ = 0;
		return;
	}

	const uint32_t candidateMinMs = pendingHeadBusLatencyMs_ > kHeadLatencySpikeBandMs
	                              ? (pendingHeadBusLatencyMs_ - kHeadLatencySpikeBandMs)
	                              : 0u;
	const uint32_t candidateMaxMs = pendingHeadBusLatencyMs_ + kHeadLatencySpikeBandMs;
	if (pendingHeadBusLatencyConfirmations_ == 0u || clampedMs < candidateMinMs ||
	    clampedMs > candidateMaxMs) {
		pendingHeadBusLatencyMs_ = clampedMs;
		pendingHeadBusLatencyConfirmations_ = 1u;
		return;
	}

	if (clampedMs > pendingHeadBusLatencyMs_) {
		pendingHeadBusLatencyMs_ = clampedMs;
	}

	++pendingHeadBusLatencyConfirmations_;
	if (pendingHeadBusLatencyConfirmations_ >= kHeadLatencySpikeConfirmSamples) {
		runtimeHeadBusLatencyMs_ = pendingHeadBusLatencyMs_;
		pendingHeadBusLatencyMs_ = 0;
		pendingHeadBusLatencyConfirmations_ = 0;
	}
}

uint32_t ExposureEngine::currentHeadBusLatencyMs() const {
	return runtimeHeadBusLatencyMs_ < kHeadBusLatencyMs ? kHeadBusLatencyMs : runtimeHeadBusLatencyMs_;
}

uint32_t ExposureEngine::currentPredictiveShutoffLeadMs() const {
	return predictiveShutoffLeadMs();
}

void ExposureEngine::tick(uint32_t nowMs) {
	if (!initialized_) {
		return;
	}

	// Vor und nach der eigentlichen Zustandsmaschine wird die abgeleitete Telemetrie
	// aktualisiert. So sehen UI und Workflow auch waehrend PreWait, Pause oder Done
	// einen konsistenten sichtbaren Zustand.
	updateDerivedTelemetry(nowMs);

	switch (state_.phase) {
		case ExposurePhase::PreWait:
			if (hasThermalHardStop(observedSensors_)) {
				enterFault(ExposureFaultReason::ThermalHardStop, nowMs);
				break;
			}

			if (state_.controlMode == ExposureControlMode::Dose && hasFatalDoseSensorFault(observedSensors_)) {
				enterFault(ExposureFaultReason::SensorWatchdog, nowMs);
				break;
			}

			if ((nowMs - phaseStartedMs_) >= kPreWaitMs) {
				if (state_.controlMode == ExposureControlMode::Dose && !hasUsableDoseSample_) {
					enterFault(ExposureFaultReason::SensorWatchdog, nowMs);
					break;
				}
				enterPhase(ExposurePhase::Exposing, nowMs);
			}
			break;

		case ExposurePhase::Exposing:
			if (state_.controlMode == ExposureControlMode::Time) {
				if (hasThermalHardStop(observedSensors_)) {
					enterFault(ExposureFaultReason::ThermalHardStop, nowMs);
					break;
				}
				if (timeBudgetMs_ > 0 && (nowMs - exposureStartedMs_) >= timeBudgetMs_) {
					enterPostWait(nowMs);
				}
			} else if (state_.controlMode == ExposureControlMode::Dose) {
				if (updateDoseClosedLoop(observedSensors_, nowMs)) {
					break;
				}
			}
			break;

		case ExposurePhase::Paused:
			if (hasThermalHardStop(observedSensors_)) {
				enterFault(ExposureFaultReason::ThermalHardStop, nowMs);
				break;
			}

			if (state_.controlMode == ExposureControlMode::Dose && !state_.sensorFallbackActive &&
			    hasFatalDoseSensorFault(observedSensors_)) {
				enterFault(ExposureFaultReason::SensorWatchdog, nowMs);
			}
			break;

		case ExposurePhase::PostWait:
			if (hasThermalHardStop(observedSensors_)) {
				enterFault(ExposureFaultReason::ThermalHardStop, nowMs);
				break;
			}

			if ((nowMs - phaseStartedMs_) >= kPostWaitMs) {
				enterDone(nowMs);
			}
			break;

		case ExposurePhase::Idle:
		case ExposurePhase::Done:
		case ExposurePhase::Fault:
			break;
	}

	updateDerivedTelemetry(nowMs);
}

bool ExposureEngine::startTimeExposure(float requestedSeconds, bool skipPreWait) {
	if (!initialized_) {
		return false;
	}

	const uint32_t requestedTimeBudgetMs = secondsToMilliseconds(requestedSeconds);
	if (requestedTimeBudgetMs < kMinimumTimeExposureMs) {
		return false;
	}

	if (!canAcceptStartRequest()) {
		enterFault(ExposureFaultReason::StartBlocked, millis());
		return false;
	}

	beginStartRequest(ExposureControlMode::Time, 0.0f, requestedTimeBudgetMs, skipPreWait, millis());
	return true;
}

bool ExposureEngine::startDoseExposure(float targetDoseLuxSeconds, bool skipPreWait) {
	if (!initialized_) {
		return false;
	}

	if (targetDoseLuxSeconds <= 0.0f) {
		return false;
	}

	if (!canAcceptStartRequest()) {
		enterFault(ExposureFaultReason::StartBlocked, millis());
		return false;
	}

	if (!hasUsableDoseSample_) {
		enterFault(ExposureFaultReason::StartBlocked, millis());
		return false;
	}

	beginStartRequest(ExposureControlMode::Dose, targetDoseLuxSeconds, 0, skipPreWait, millis());
	return true;
}

bool ExposureEngine::stopGracefully() {
	if (!initialized_) {
		return false;
	}

	if (state_.phase != ExposurePhase::PreWait && state_.phase != ExposurePhase::Exposing &&
	    state_.phase != ExposurePhase::Paused) {
		return false;
	}

	if (state_.phase == ExposurePhase::PreWait) {
		enterIdle(millis());
		return true;
	}

	enterPostWait(millis());
	return true;
}

bool ExposureEngine::abortWithFault(ExposureFaultReason faultReason) {
	if (!initialized_) {
		return false;
	}

	enterFault(faultReason == ExposureFaultReason::None ? ExposureFaultReason::InternalFault : faultReason,
	           millis());
	return true;
}

bool ExposureEngine::pause() {
	if (!initialized_ || state_.phase != ExposurePhase::Exposing) {
		return false;
	}

	if (state_.controlMode == ExposureControlMode::Time && timeBudgetMs_ > 0) {
		const uint32_t elapsedMs = millis() - exposureStartedMs_;
		timeBudgetMs_ = elapsedMs >= timeBudgetMs_ ? 0 : (timeBudgetMs_ - elapsedMs);
	} else if (state_.controlMode == ExposureControlMode::Dose && state_.sensorFallbackActive &&
	           doseFallbackBudgetMs_ > 0 && doseFallbackStartedMs_ > 0) {
		const uint32_t elapsedMs = millis() - doseFallbackStartedMs_;
		doseFallbackBudgetMs_ = elapsedMs >= doseFallbackBudgetMs_ ? 0 : (doseFallbackBudgetMs_ - elapsedMs);
	}

	enterPhase(ExposurePhase::Paused, millis());
	updateDerivedTelemetry(millis());
	return true;
}

bool ExposureEngine::resume() {
	if (!initialized_ || state_.phase != ExposurePhase::Paused) {
		return false;
	}

	const uint32_t nowMs = millis();
	if (hasThermalHardStop(observedSensors_)) {
		enterFault(ExposureFaultReason::ThermalHardStop, nowMs);
		return false;
	}

	if (state_.controlMode == ExposureControlMode::Dose && !state_.sensorFallbackActive && !hasUsableDoseSample_) {
		enterFault(ExposureFaultReason::SensorWatchdog, nowMs);
		return false;
	}

	enterPhase(ExposurePhase::Exposing, nowMs);
	updateDerivedTelemetry(nowMs);
	return true;
}

bool ExposureEngine::acknowledgeDone() {
	if (!initialized_ || state_.phase != ExposurePhase::Done) {
		return false;
	}

	enterIdle(millis());
	return true;
}

bool ExposureEngine::clearLatchedFault() {
	if (!initialized_ || state_.phase != ExposurePhase::Fault) {
		return false;
	}

	enterIdle(millis());
	return true;
}

const ExposureRuntimeState& ExposureEngine::state() const {
	return state_;
}

void ExposureEngine::setThermalProtectionConfig(float deratingStartCelsius, float hardStopCelsius) {
	// Die globalen Safety-Grenzen duerfen zur Laufzeit nur in plausibler Form
	// wirksam werden. So bleibt der physische Hard-Stop fail-safe, auch wenn spaeter
	// weitere Settings-Migrationen oder Altbestaende dazukommen.
	if (!isfinite(deratingStartCelsius) || !isfinite(hardStopCelsius)) {
		return;
	}

	if (deratingStartCelsius < 30.0f) {
		deratingStartCelsius = 30.0f;
	}

	if (hardStopCelsius > 90.0f) {
		hardStopCelsius = 90.0f;
	}

	if ((hardStopCelsius - deratingStartCelsius) < 5.0f) {
		hardStopCelsius = deratingStartCelsius + 5.0f;
	}

	thermalDeratingStartCelsius_ = deratingStartCelsius;
	thermalHardStopCelsius_ = hardStopCelsius;
	updateThermalRuntime(observedSensors_);
}

bool ExposureEngine::canAcceptStartRequest() const {
	return state_.phase == ExposurePhase::Idle || state_.phase == ExposurePhase::Done;
}

void ExposureEngine::beginStartRequest(ExposureControlMode controlMode, float targetDose,
	                                   uint32_t requestedTimeBudgetMs, bool skipPreWait,
	                                   uint32_t nowMs) {
	// Jeder neue Start beginnt aus einem frisch aufgebauten Runtime-State. Dadurch
	// werden Reste frueherer Belichtungen, Faults oder Sensorfenster sicher verworfen,
	// bevor PreWait oder Exposing ueberhaupt beginnen.
	state_ = makeIdleExposureRuntimeState();
	state_.controlMode = controlMode;
	state_.targetDose = targetDose;
	state_.currentDose = 0.0f;
	state_.remainingDose = targetDose;
	state_.remainingTimeSeconds = requestedTimeBudgetMs / 1000.0f;
	state_.faultReason = ExposureFaultReason::None;
	integratedDoseLuxSeconds_ = 0.0;
	timeBudgetMs_ = requestedTimeBudgetMs;
	phaseStartedMs_ = nowMs;
	exposureStartedMs_ = nowMs;
	lastPlausibleDoseLux_ = 0.0f;
	updateThermalRuntime(observedSensors_);
	resetDoseLoopTracking();
	clearDoseSensorFallback();
	state_.runtimeHeadBusLatencyMs = currentHeadBusLatencyMs();

	if (skipPreWait) {
		enterPhase(ExposurePhase::Exposing, nowMs);
		return;
	}

	enterPhase(ExposurePhase::PreWait, nowMs);
}

void ExposureEngine::enterPhase(ExposurePhase phase, uint32_t nowMs) {
	// enterPhase() kapselt die Nebenwirkungen eines Phasenwechsels an einer Stelle:
	// Zeitmarken, Startzeit und das Zuruecksetzen rein exponierungsbezogener Felder.
	state_.phase = phase;
	phaseStartedMs_ = nowMs;

	if (phase == ExposurePhase::Exposing) {
		exposureStartedMs_ = nowMs;
		if (state_.controlMode == ExposureControlMode::Dose) {
			resetDoseLoopTracking();
		}
	} else if (phase != ExposurePhase::Paused) {
		state_.measuredLux = 0.0f;
	}
}

void ExposureEngine::enterIdle(uint32_t nowMs) {
	state_ = makeIdleExposureRuntimeState();
	phaseStartedMs_ = nowMs;
	exposureStartedMs_ = nowMs;
	timeBudgetMs_ = 0;
	integratedDoseLuxSeconds_ = 0.0;
	lastPlausibleDoseLux_ = 0.0f;
	resetDoseLoopTracking();
	clearDoseSensorFallback();
	updateThermalRuntime(observedSensors_);
	state_.runtimeHeadBusLatencyMs = currentHeadBusLatencyMs();
}

void ExposureEngine::enterPostWait(uint32_t nowMs) {
	enterPhase(ExposurePhase::PostWait, nowMs);
	state_.remainingTimeSeconds = 0.0f;
	state_.remainingDose = state_.targetDose > state_.currentDose ? (state_.targetDose - state_.currentDose) : 0.0f;
	state_.measuredLux = 0.0f;
	timeBudgetMs_ = 0;
	resetDoseLoopTracking();
}

void ExposureEngine::enterDone(uint32_t nowMs) {
	enterPhase(ExposurePhase::Done, nowMs);
	state_.remainingTimeSeconds = 0.0f;
	state_.remainingDose = 0.0f;
	state_.measuredLux = 0.0f;
	timeBudgetMs_ = 0;
	resetDoseLoopTracking();
}

void ExposureEngine::enterFault(ExposureFaultReason faultReason, uint32_t nowMs) {
	enterPhase(ExposurePhase::Fault, nowMs);
	state_.faultReason = faultReason;
	state_.faultLatched = true;
	lastPlausibleDoseLux_ = 0.0f;
	state_.remainingTimeSeconds = 0.0f;
	state_.remainingDose = state_.targetDose > state_.currentDose ? (state_.targetDose - state_.currentDose) : 0.0f;
	state_.measuredLux = 0.0f;
	timeBudgetMs_ = 0;
	resetDoseLoopTracking();
	clearDoseSensorFallback();
}

void ExposureEngine::resetDoseLoopTracking() {
	lastIntegratedDoseSampleTimestampMs_ = 0;
	invalidDoseSampleSinceMs_ = 0;
	zeroLuxSinceMs_ = 0;
	integratedDoseLuxSeconds_ = static_cast<double>(state_.currentDose);
	state_.measuredLux = 0.0f;
}

void ExposureEngine::clearDoseSensorFallback() {
	state_.sensorFallbackActive = false;
	state_.sensorFallbackReason = ExposureFaultReason::None;
	doseFallbackStartedMs_ = 0;
	doseFallbackBudgetMs_ = 0;
}

void ExposureEngine::updateThermalRuntime(const SensorRuntimeStatus& sensorStatus) {
	const Ds18b20RuntimeStatus& ds18 = sensorStatus.ds18b20;

	state_.runtimeOutputLimit = 1.0f;
	state_.thermalDeratingActive = false;

	if (!ds18.initialized || ds18.health != SensorHealth::Ok || isnan(ds18.temperatureCelsius)) {
		return;
	}

	if (ds18.temperatureCelsius >= thermalHardStopCelsius_) {
		state_.runtimeOutputLimit = 0.0f;
		state_.thermalDeratingActive = true;
		return;
	}

	state_.runtimeOutputLimit = thermalOutputLimitForTemperature(ds18.temperatureCelsius);
	state_.thermalDeratingActive = state_.runtimeOutputLimit < 0.999f;
}

bool ExposureEngine::hasThermalHardStop(const SensorRuntimeStatus& sensorStatus) const {
	const Ds18b20RuntimeStatus& ds18 = sensorStatus.ds18b20;
	return ds18.thermalState == ThermalState::Critical ||
	       (ds18.initialized && ds18.health == SensorHealth::Ok && !isnan(ds18.temperatureCelsius) &&
	        ds18.temperatureCelsius >= thermalHardStopCelsius_);
}

bool ExposureEngine::hasFatalDoseSensorFault(const SensorRuntimeStatus& sensorStatus) const {
	const Tsl2561RuntimeStatus& tsl = sensorStatus.tsl2561;
	return tsl.initialized &&
	       (tsl.health == SensorHealth::Fault || tsl.sampleValidity == LuxSampleValidity::Fault);
}

bool ExposureEngine::activateDoseSensorFallback(ExposureFaultReason fallbackReason, uint32_t nowMs) {
	if (state_.controlMode != ExposureControlMode::Dose || state_.phase != ExposurePhase::Exposing) {
		enterFault(fallbackReason, nowMs);
		return true;
	}

	// Eine bereits laufende Papierbelichtung darf an dieser Stelle nicht neu
	// gestartet oder verworfen werden. Sobald der lokale Closed-Loop-Sensor im
	// Exposing ausfaellt, wird deshalb mit der letzten noch plausiblen Dosisrate
	// nur noch eine harte Restzeit zu Ende gefahren.

	state_.remainingDose = state_.targetDose > state_.currentDose ? (state_.targetDose - state_.currentDose) : 0.0f;
	if (state_.remainingDose <= 0.0f) {
		state_.currentDose = state_.targetDose;
		enterPostWait(nowMs);
		return true;
	}

	if (!isfinite(lastPlausibleDoseLux_) || lastPlausibleDoseLux_ <= kMinimumPredictiveLux) {
		enterFault(fallbackReason, nowMs);
		return true;
	}

	const float rawRemainingTimeSeconds = state_.remainingDose / lastPlausibleDoseLux_;
	const float leadSeconds = static_cast<float>(predictiveShutoffLeadMs()) / 1000.0f;
	state_.sensorFallbackActive = true;
	state_.sensorFallbackReason = fallbackReason;
	state_.measuredLux = 0.0f;

	if (rawRemainingTimeSeconds <= leadSeconds) {
		state_.currentDose = state_.targetDose;
		state_.remainingDose = 0.0f;
		state_.remainingTimeSeconds = 0.0f;
		enterPostWait(nowMs);
		return true;
	}

	doseFallbackBudgetMs_ = secondsToMilliseconds(rawRemainingTimeSeconds - leadSeconds);
	if (doseFallbackBudgetMs_ < kMinimumDoseFallbackMs) {
		doseFallbackBudgetMs_ = kMinimumDoseFallbackMs;
	}
	doseFallbackStartedMs_ = nowMs;
	state_.remainingTimeSeconds = static_cast<float>(doseFallbackBudgetMs_) / 1000.0f;
	return false;
}

bool ExposureEngine::updateDoseSensorFallback(uint32_t nowMs) {
	if (!state_.sensorFallbackActive) {
		return false;
	}

	state_.measuredLux = 0.0f;
	state_.remainingDose = state_.targetDose > state_.currentDose ? (state_.targetDose - state_.currentDose) : 0.0f;
	if (doseFallbackBudgetMs_ == 0u || doseFallbackStartedMs_ == 0u) {
		state_.currentDose = state_.targetDose;
		state_.remainingDose = 0.0f;
		state_.remainingTimeSeconds = 0.0f;
		enterPostWait(nowMs);
		return true;
	}

	const uint32_t elapsedMs = nowMs - doseFallbackStartedMs_;
	if (elapsedMs >= doseFallbackBudgetMs_) {
		state_.currentDose = state_.targetDose;
		state_.remainingDose = 0.0f;
		state_.remainingTimeSeconds = 0.0f;
		enterPostWait(nowMs);
		return true;
	}

	const uint32_t remainingMs = doseFallbackBudgetMs_ - elapsedMs;
	state_.remainingTimeSeconds = static_cast<float>(remainingMs) / 1000.0f;
	return false;
}

bool ExposureEngine::updateDoseClosedLoop(const SensorRuntimeStatus& sensorStatus, uint32_t nowMs) {
	// Diese Funktion ist der komplette lokale Dose-Regelpfad:
	// - thermische Grenzen pruefen
	// - TSL-Sample auf Gueltigkeit/Freshness/Plausibilitaet pruefen
	// - aus dem Sample-Zeitstempel die integrierte Dosis fortschreiben
	// - verbleibende Zeit fuer den praediktiven Shutoff ableiten
	updateThermalRuntime(sensorStatus);

	if (hasThermalHardStop(sensorStatus)) {
		enterFault(ExposureFaultReason::ThermalHardStop, nowMs);
		return true;
	}

	if (state_.sensorFallbackActive) {
		return updateDoseSensorFallback(nowMs);
	}

	const Tsl2561RuntimeStatus& tsl = sensorStatus.tsl2561;
	if (!isDoseSampleUsable(tsl)) {
		state_.measuredLux = 0.0f;
		if (invalidDoseSampleSinceMs_ == 0) {
			invalidDoseSampleSinceMs_ = nowMs;
		} else if ((nowMs - invalidDoseSampleSinceMs_) >= kDoseSampleWatchdogMs) {
			return activateDoseSensorFallback(ExposureFaultReason::SensorWatchdog, nowMs);
		}
		return false;
	}

	invalidDoseSampleSinceMs_ = 0;
	const uint32_t sampleTimestampMs = estimateDoseSampleTimestampMs(nowMs, tsl);
	lastObservedDoseSampleTimestampMs_ = sampleTimestampMs;

	state_.measuredLux = tsl.lux;
	if (state_.measuredLux <= kMinimumPlausibleLux) {
		if ((nowMs - exposureStartedMs_) > kZeroLuxGraceMs) {
			if (zeroLuxSinceMs_ == 0) {
				zeroLuxSinceMs_ = nowMs;
			} else if ((nowMs - zeroLuxSinceMs_) >= kZeroLuxFaultWindowMs) {
				return activateDoseSensorFallback(ExposureFaultReason::SensorPlausibility, nowMs);
			}
		}
	} else {
		zeroLuxSinceMs_ = 0;
		lastPlausibleDoseLux_ = state_.measuredLux;
	}

	if (lastIntegratedDoseSampleTimestampMs_ == 0) {
		lastIntegratedDoseSampleTimestampMs_ = sampleTimestampMs;
	} else if (sampleTimestampMs != lastIntegratedDoseSampleTimestampMs_) {
		const uint32_t deltaMs = sampleTimestampMs - lastIntegratedDoseSampleTimestampMs_;
		double dtSeconds = static_cast<double>(deltaMs) / 1000.0;
		if (dtSeconds > kMaximumIntegratedDoseDtSeconds) {
			dtSeconds = kMaximumIntegratedDoseDtSeconds;
		}

		if (dtSeconds > 0.0) {
			integratedDoseLuxSeconds_ += static_cast<double>(state_.measuredLux) * dtSeconds;
			state_.currentDose = static_cast<float>(integratedDoseLuxSeconds_);
		}
		lastIntegratedDoseSampleTimestampMs_ = sampleTimestampMs;
	}

	state_.remainingDose = state_.targetDose > state_.currentDose ? (state_.targetDose - state_.currentDose) : 0.0f;
	if (state_.remainingDose <= 0.0f) {
		enterPostWait(nowMs);
		return true;
	}

	if (state_.measuredLux > kMinimumPredictiveLux) {
		const float rawRemainingTimeSeconds = state_.remainingDose / state_.measuredLux;
		const float leadSeconds = static_cast<float>(predictiveShutoffLeadMs()) / 1000.0f;
		state_.remainingTimeSeconds =
			rawRemainingTimeSeconds > leadSeconds ? (rawRemainingTimeSeconds - leadSeconds) : 0.0f;

		if (rawRemainingTimeSeconds <= leadSeconds) {
			enterPostWait(nowMs);
			return true;
		}
	} else {
		state_.remainingTimeSeconds = 0.0f;
	}

	return false;
}

bool ExposureEngine::isDoseSampleUsable(const Tsl2561RuntimeStatus& sensorStatus) const {
	return sensorStatus.initialized && sensorStatus.health == SensorHealth::Ok &&
	       sensorStatus.sampleValidity == LuxSampleValidity::Valid && sensorStatus.sampleFresh &&
	       !isnan(sensorStatus.lux) && sensorStatus.lux >= 0.0f;
}

uint32_t ExposureEngine::estimateDoseSampleTimestampMs(uint32_t nowMs,
	                                                   const Tsl2561RuntimeStatus& sensorStatus) const {
	if (sensorStatus.sampleAgeMs > nowMs) {
		return nowMs;
	}

	return nowMs - sensorStatus.sampleAgeMs;
}

float ExposureEngine::thermalOutputLimitForTemperature(float temperatureCelsius) const {
	if (temperatureCelsius <= thermalDeratingStartCelsius_) {
		return 1.0f;
	}

	if (temperatureCelsius >= thermalHardStopCelsius_) {
		return 0.0f;
	}

	const float normalized =
		(temperatureCelsius - thermalDeratingStartCelsius_) /
		(thermalHardStopCelsius_ - thermalDeratingStartCelsius_);
	const float limited = 1.0f - (normalized * (1.0f - kThermalMinimumOutputLimit));
	if (limited < kThermalMinimumOutputLimit) {
		return kThermalMinimumOutputLimit;
	}
	if (limited > 1.0f) {
		return 1.0f;
	}

	return limited;
}

uint32_t ExposureEngine::predictiveShutoffLeadMs() const {
	// Der Shutoff-Vorlauf basiert nicht auf einem festen Magiewert, sondern auf
	// einer konservativen Schaetzung der tatsaechlichen Head-Latenz. So bleibt die
	// Dosisintegration auch dann stabil, wenn der Head-Bus langsamer wird.
	const uint32_t observedBusLatencyMs = runtimeHeadBusLatencyMs_ > pendingHeadBusLatencyMs_
	                                   ? runtimeHeadBusLatencyMs_
	                                   : pendingHeadBusLatencyMs_;
	const uint32_t busLatencyMs = observedBusLatencyMs < kHeadBusLatencyMs ? kHeadBusLatencyMs : observedBusLatencyMs;
	if (busLatencyMs <= kHeadBusLatencyMs) {
		return kHeadPredictiveShutoffBaseLeadMs;
	}

	return kHeadPredictiveShutoffBaseLeadMs + (busLatencyMs - kHeadBusLatencyMs);
}

void ExposureEngine::updateDerivedTelemetry(uint32_t nowMs) {
	// state_ enthaelt sowohl steuernde Felder als auch reine Sichtdaten.
	// Diese Funktion berechnet nur die sichtbaren Telemetrieanteile wie
	// Restzeit, Restdosis und gemessene Lux aus dem eigentlichen Maschinenzustand.
	state_.phaseAgeMs = nowMs - phaseStartedMs_;
	// Die adaptive Head-Bus-Latenz gehoert zum sichtbaren Runtime-Vertrag, damit
	// UI und Diagnose denselben konservativ geglaetteten Millisekundenwert sehen.
	state_.runtimeHeadBusLatencyMs = currentHeadBusLatencyMs();

	if (state_.controlMode == ExposureControlMode::Time) {
		if (state_.phase == ExposurePhase::Exposing && timeBudgetMs_ > 0) {
			const uint32_t elapsedMs = nowMs - exposureStartedMs_;
			const uint32_t remainingMs = elapsedMs >= timeBudgetMs_ ? 0 : (timeBudgetMs_ - elapsedMs);
			state_.remainingTimeSeconds = remainingMs / 1000.0f;
		} else if (state_.phase == ExposurePhase::Paused || state_.phase == ExposurePhase::PreWait) {
			state_.remainingTimeSeconds = timeBudgetMs_ / 1000.0f;
		} else if (state_.phase == ExposurePhase::Idle || state_.phase == ExposurePhase::Done ||
		           state_.phase == ExposurePhase::Fault || state_.phase == ExposurePhase::PostWait) {
			state_.remainingTimeSeconds = 0.0f;
		}

		state_.measuredLux = 0.0f;
	} else if (state_.controlMode == ExposureControlMode::Dose) {
		state_.remainingDose = state_.targetDose > state_.currentDose ? (state_.targetDose - state_.currentDose) : 0.0f;
		if (state_.sensorFallbackActive) {
			if ((state_.phase == ExposurePhase::Exposing || state_.phase == ExposurePhase::Paused) &&
			    doseFallbackBudgetMs_ > 0) {
				uint32_t remainingMs = doseFallbackBudgetMs_;
				if (state_.phase == ExposurePhase::Exposing && doseFallbackStartedMs_ > 0) {
					const uint32_t elapsedMs = nowMs - doseFallbackStartedMs_;
					remainingMs = elapsedMs >= doseFallbackBudgetMs_ ? 0 : (doseFallbackBudgetMs_ - elapsedMs);
				}
				state_.remainingTimeSeconds = static_cast<float>(remainingMs) / 1000.0f;
			} else if (state_.phase != ExposurePhase::Done && state_.phase != ExposurePhase::PostWait) {
				state_.remainingTimeSeconds = 0.0f;
			}
			state_.measuredLux = 0.0f;
		} else if (state_.phase != ExposurePhase::Exposing) {
			state_.remainingTimeSeconds = 0.0f;
			state_.measuredLux = 0.0f;
		}
	} else {
		state_.remainingTimeSeconds = 0.0f;
		state_.measuredLux = 0.0f;
	}

	if (state_.controlMode == ExposureControlMode::Dose) {
		state_.remainingDose = state_.targetDose > state_.currentDose ? (state_.targetDose - state_.currentDose) : 0.0f;
	} else if (state_.phase == ExposurePhase::Idle || state_.phase == ExposurePhase::Done) {
		state_.remainingDose = 0.0f;
	}
}

}  // namespace dukatimer