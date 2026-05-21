/*
 * SensorManager
 *
 * Laufzeitrolle:
 * - ist der alleinige Besitzer des sichtbaren lokalen Sensorstatus
 * - haertet rohe TSL2561- und DS18B20-Daten in einen stabilen Statusraum
 * - kapselt I2C-, Interrupt- und Diagnosebesonderheiten vollstaendig vor
 *   ExposureEngine, Workflows und UI
 */

#include "SensorManager.h"

namespace dukatimer {

namespace {

constexpr uint8_t kTslCommandBit = 0x80;
constexpr uint8_t kTslControlRegister = 0x00;
constexpr uint8_t kTslTimingRegister = 0x01;
// Block-Modus (Bit 4 = 0x10) aktiviert Auto-Increment des Register-Pointers.
// Damit liest requestFrom(4) sicher die Register 0x0C, 0x0D, 0x0E, 0x0F
// (CH0_LOW, CH0_HIGH, CH1_LOW, CH1_HIGH) in Folge. Befund 3.
constexpr uint8_t kTslData0LowRegister = 0x0C;
constexpr uint8_t kTslBlockBit = 0x10;

constexpr uint8_t kTslControlPowerOn = 0x03;
constexpr uint8_t kTslTiming101ms1x = 0x01;

constexpr uint16_t kTslChannelSaturation = 0xFFFF;

// TSL2561 fixed-point lux constants (T package). This avoids heavy libm usage.
constexpr uint8_t kLuxScale = 14;
constexpr uint8_t kRatioScale = 9;
constexpr uint8_t kChannelScale = 10;

constexpr uint16_t kChannelScaleTint1 = 0x0FE7;  // 101ms integration

constexpr uint16_t kK1T = 0x0040;
constexpr uint16_t kB1T = 0x01F2;
constexpr uint16_t kM1T = 0x01BE;
constexpr uint16_t kK2T = 0x0080;
constexpr uint16_t kB2T = 0x0214;
constexpr uint16_t kM2T = 0x02D1;
constexpr uint16_t kK3T = 0x00C0;
constexpr uint16_t kB3T = 0x023F;
constexpr uint16_t kM3T = 0x037B;
constexpr uint16_t kK4T = 0x0100;
constexpr uint16_t kB4T = 0x0270;
constexpr uint16_t kM4T = 0x03FE;
constexpr uint16_t kK5T = 0x0138;
constexpr uint16_t kB5T = 0x016F;
constexpr uint16_t kM5T = 0x01FC;
constexpr uint16_t kK6T = 0x019A;
constexpr uint16_t kB6T = 0x00D2;
constexpr uint16_t kM6T = 0x00FB;
constexpr uint16_t kK7T = 0x029A;
constexpr uint16_t kB7T = 0x0018;
constexpr uint16_t kM7T = 0x0012;

// Interrupt- und Schwellwert-Register des TSL2561
constexpr uint8_t kTslThreshLowLowRegister = 0x02;
constexpr uint8_t kTslThreshLowHighRegister = 0x03;
constexpr uint8_t kTslThreshHighLowRegister = 0x04;
constexpr uint8_t kTslThreshHighHighRegister = 0x05;
constexpr uint8_t kTslInterruptRegister = 0x06;
// INTR=01 (Level-Interrupt aktiv), PERSIST=0000 (Interrupt nach jedem ADC-Zyklus)
constexpr uint8_t kTslInterruptEveryAdcCycle = 0x10;
// Command-Byte zum Loeschen des TSL2561-Interrupt-Latches: CMD=1, CL=1
constexpr uint8_t kTslCommandClearInterrupt = 0xC0;
// Oberer Schwellwert fuer den Hardware-Not-Abschalt-Pfad.
// Liegt kurz unterhalb der ADC-Saettigung; ein Softwarefehler, der die Matrix
// auf Vollweiss haelt, loest diesen Schwellwert aus.
constexpr uint16_t kTslEmergencyHighThreshold = 0xF000;

// Hinweis: Der Teensy 4.1 (IMXRT1062) bietet in seinem Wire-Core (WireIMXRT)
// keine API zum Setzen eines Hardware-I2C-Timeouts (kein setWireTimeout()).
// Stream::setTimeout() adressiert nur serielle Puffer-Pfade und hat keinerlei
// Wirkung auf blockierende Wire.endTransmission()- oder Wire.requestFrom()-
// Aufrufe. Ein defekter I2C-Bus (SCL-Stretch, kurzgeschlossene Leitungen) kann
// den Main-Loop dauerhaft blockieren. Dagegen laeuft die applikationsseitige
// Loop-Watchdog-Absicherung im Teensy-main.cpp; dieser Sensorpfad besitzt aber
// weiterhin keinen lokalen, feingranularen I2C-Timeout-Mechanismus.

}  // namespace

SensorManager* SensorManager::s_tsl2561IsrInstance_ = nullptr;

SensorManager::~SensorManager() {
	releaseTslInterruptBinding();
}

// Minimale ISR: setzt nur das Flag und speichert den Zeitstempel.
// millis() ist auf dem Teensy 4.1 ISR-sicher (SysTick-Zaehler).
void SensorManager::tslIntIsr() {
	SensorManager* const inst = s_tsl2561IsrInstance_;
	if (inst != nullptr) {
		inst->tslIntPending_ = true;
		inst->tslIntCapturedMs_ = millis();
	}
}

void SensorManager::begin(uint8_t tslIntPin, uint32_t nowMs) {
	// begin() startet ausschliesslich den lokalen Sensordienst. Sichtbare Daten
	// werden erst nach dem ersten gueltigen Sample publiziert; der Rest des Systems
	// muss daher mit Unknown/NotReady sauber umgehen koennen.
	if (initialized_) {
		return;
	}

	tslIntPin_ = tslIntPin;
	initialized_ = true;
	resetState(nowMs);
	// Kein sofortiger Poll: erste Messung kommt per Interrupt (oder Watchdog-Polling-Fallback)
	// nach Ablauf der ersten 101-ms-Integrationsperiode. Befund 1 behoben.
	initializeLocalTsl2561(nowMs);
}

void SensorManager::tick(uint32_t nowMs) {
	// tick() fuehrt alle lokalen Sensorpfade zusammen:
	// - TSL-Recovery / Reinitialisierung
	// - ISR-getriebene oder gepollte Lux-Samples
	// - Sample-Aging fuer Fresh/Stale
	// - thermische Ableitung fuer den DS18B20-Status
	if (!initialized_) {
		return;
	}

	ensureLocalTsl2561Ready(nowMs);

	if (localTslReady_) {
		if (tslIntPin_ != kTslIntPinNone) {
			// Interrupt-gesteuerter Pfad: praeziser Zeitstempel vom ISR
			if (tslIntPending_) {
				tslIntPending_ = false;
				lastTslIntMs_ = nowMs;
				pollLocalTsl2561(tslIntCapturedMs_);
			} else if ((nowMs - lastTslIntMs_) >= kTslIntWatchdogMs) {
				// Sensor hat aufgehoert, Interrupts zu liefern
				localTslReady_ = false;
				markTsl2561Fault(Tsl2561DiagnosticReason::InterruptWatchdog);
			}
		} else if (nowMs >= nextTslPollDueMs_) {
			// Polling-Fallback: aktiv wenn kein Interrupt-Pin konfiguriert
			pollLocalTsl2561(nowMs);
		}
	}

	updateTslAging(nowMs);
	updateThermalState();
}

void SensorManager::setTsl2561Initialized(bool initialized) {
	state_.tsl2561.initialized = initialized;

	if (!initialized) {
		localTslReady_ = false;
		tslReadFailureCount_ = 0;
		hasTslSample_ = false;
		lastTslSampleMs_ = 0;
		state_.tsl2561.health = SensorHealth::Unknown;
		state_.tsl2561.sampleValidity = LuxSampleValidity::Unknown;
		state_.tsl2561.diagnosticReason = Tsl2561DiagnosticReason::None;
		state_.tsl2561.sampleFresh = false;
		state_.tsl2561.sampleAgeMs = 0;
		state_.tsl2561.lux = 0.0f;
	}
}

void SensorManager::publishTsl2561Sample(float lux,
	                                     LuxSampleValidity sampleValidity,
	                                     Tsl2561DiagnosticReason diagnosticReason,
	                                     uint32_t nowMs) {
	if (!initialized_) {
		return;
	}

	if (nowMs == 0) {
		nowMs = millis();
	}

	// Der harte Lux-Pfad haertet weiterhin gegen unplausible Werte, damit
	// ExposureEngine nur auf einen gueltigen und reproduzierbaren Statusraum sieht.
	if (isnan(lux) || lux < 0.0f) {
		state_.tsl2561.initialized = true;
		state_.tsl2561.health = SensorHealth::Fault;
		state_.tsl2561.sampleValidity = LuxSampleValidity::Fault;
		state_.tsl2561.diagnosticReason = Tsl2561DiagnosticReason::InvalidSample;
		state_.tsl2561.sampleFresh = false;
		state_.tsl2561.sampleAgeMs = 0;
		state_.tsl2561.lux = 0.0f;
		hasTslSample_ = false;
		lastTslSampleMs_ = 0;
		return;
	}

	state_.tsl2561.initialized = true;
	state_.tsl2561.health = SensorHealth::Ok;
	state_.tsl2561.sampleValidity = sampleValidity;
	state_.tsl2561.diagnosticReason = diagnosticReason;
	state_.tsl2561.sampleFresh = true;
	state_.tsl2561.sampleAgeMs = 0;
	state_.tsl2561.lux = lux;
	hasTslSample_ = true;
	lastTslSampleMs_ = nowMs;

	if (sampleValidity == LuxSampleValidity::Fault) {
		state_.tsl2561.health = SensorHealth::Fault;
		state_.tsl2561.sampleFresh = false;
	}
}

void SensorManager::markTsl2561Fault(Tsl2561DiagnosticReason diagnosticReason) {
	state_.tsl2561.initialized = true;
	state_.tsl2561.health = SensorHealth::Fault;
	state_.tsl2561.sampleValidity = LuxSampleValidity::Fault;
	state_.tsl2561.diagnosticReason = diagnosticReason;
	state_.tsl2561.sampleFresh = false;
	state_.tsl2561.sampleAgeMs = 0;
	state_.tsl2561.lux = 0.0f;
	hasTslSample_ = false;
	lastTslSampleMs_ = 0;
}

void SensorManager::setDs18b20Initialized(bool initialized) {
	state_.ds18b20.initialized = initialized;

	if (!initialized) {
		state_.ds18b20.health = SensorHealth::Unknown;
		state_.ds18b20.thermalState = ThermalState::Unknown;
		state_.ds18b20.diagnosticReason = Ds18b20DiagnosticReason::None;
		state_.ds18b20.temperatureCelsius = 0.0f;
	}
}

void SensorManager::publishDs18b20Temperature(float temperatureCelsius) {
	if (!initialized_) {
		return;
	}

	if (isnan(temperatureCelsius)) {
		state_.ds18b20.diagnosticReason = Ds18b20DiagnosticReason::InvalidSample;
		markDs18b20Fault();
		return;
	}

	state_.ds18b20.initialized = true;
	state_.ds18b20.health = SensorHealth::Ok;
	state_.ds18b20.diagnosticReason = Ds18b20DiagnosticReason::None;
	state_.ds18b20.temperatureCelsius = temperatureCelsius;
	updateThermalState();
}

void SensorManager::markDs18b20Fault() {
	state_.ds18b20.initialized = true;
	state_.ds18b20.health = SensorHealth::Fault;
	state_.ds18b20.thermalState = ThermalState::Fault;
	if (state_.ds18b20.diagnosticReason == Ds18b20DiagnosticReason::None) {
		state_.ds18b20.diagnosticReason = Ds18b20DiagnosticReason::UpstreamFault;
	}
}

void SensorManager::setThermalThresholds(float deratingThresholdCelsius, float criticalThresholdCelsius) {
	// Die grobe ThermalState-Klassifikation folgt denselben Grenzwerten wie der
	// physische Exposure-Pfad, damit UI und Hard-Stop nie unterschiedliche
	// Schwellen sichtbar machen.
	thermalDeratingThresholdCelsius_ = deratingThresholdCelsius;
	thermalCriticalThresholdCelsius_ = criticalThresholdCelsius;
	updateThermalState();
}

const SensorRuntimeStatus& SensorManager::state() const {
	return state_;
}

void SensorManager::resetState(uint32_t nowMs) {
	(void)nowMs;
	state_ = makeUnknownSensorRuntimeStatus();
	localTslReady_ = false;
	tslReadFailureCount_ = 0;
	hasTslSample_ = false;
	lastTslSampleMs_ = 0;
	nextTslPollDueMs_ = 0;
	lastTslInitAttemptMs_ = 0;
	tslIntPending_ = false;
	tslIntCapturedMs_ = 0;
	lastTslIntMs_ = 0;
}

bool SensorManager::initializeLocalTsl2561(uint32_t nowMs) {
	// Reinitialisierung baut den Sensor komplett neu auf und verwirft dabei alte
	// Pending-Flags, Read-Failure-Zaehler und Sampleannahmen. Das ist wichtig,
	// damit Recovery nach einem Bus- oder Interruptfehler reproduzierbar bleibt.
	lastTslInitAttemptMs_ = nowMs;
	tslReadFailureCount_ = 0;
	tslIntPending_ = false;  // altes Pending aus vorherigem Zyklus verwerfen

	if (tslBus_ == nullptr) {
		localTslReady_ = false;
		markTsl2561Fault(Tsl2561DiagnosticReason::InitializationFailed);
		return false;
	}

	tslBus_->begin();
	tslBus_->setClock(kTslI2cClockHz);

	// Reihenfolge: CONTROL -> TIMING -> Schwellwerte -> INTERRUPT
	// Schwellwerte muessen vor INTERRUPT gesetzt sein, damit der erste
	// Hardware-Edge nach Ablauf der Integrationsperiode sauber ausgeloest wird.
	if (!writeTslRegister(kTslControlRegister, kTslControlPowerOn) ||
	    !writeTslRegister(kTslTimingRegister, kTslTiming101ms1x) ||
	    !writeTslRegister(kTslThreshLowLowRegister, 0x00) ||
	    !writeTslRegister(kTslThreshLowHighRegister, 0x00) ||
	    !writeTslRegister(kTslThreshHighLowRegister,
	                      static_cast<uint8_t>(kTslEmergencyHighThreshold & 0xFF)) ||
	    !writeTslRegister(kTslThreshHighHighRegister,
	                      static_cast<uint8_t>((kTslEmergencyHighThreshold >> 8) & 0xFF)) ||
	    !writeTslRegister(kTslInterruptRegister, kTslInterruptEveryAdcCycle)) {
		localTslReady_ = false;
		markTsl2561Fault(Tsl2561DiagnosticReason::InitializationFailed);
		return false;
	}

	if (tslIntPin_ != kTslIntPinNone) {
		releaseTslInterruptBinding();
		s_tsl2561IsrInstance_ = this;
		pinMode(tslIntPin_, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(tslIntPin_), tslIntIsr, FALLING);
	}

	setTsl2561Initialized(true);
	state_.tsl2561.health = SensorHealth::Unknown;
	state_.tsl2561.sampleValidity = LuxSampleValidity::Unknown;
	state_.tsl2561.diagnosticReason = Tsl2561DiagnosticReason::None;
	state_.tsl2561.sampleFresh = false;
	state_.tsl2561.sampleAgeMs = 0;
	state_.tsl2561.lux = 0.0f;
	hasTslSample_ = false;
	lastTslSampleMs_ = 0;
	// Fallback-Polling erst nach einer vollen Integrationsperiode (101 ms). Befund 1.
	nextTslPollDueMs_ = nowMs + kTslPollIntervalMs;
	lastTslIntMs_ = nowMs;  // Watchdog-Startpunkt
	localTslReady_ = true;
	return true;
}

void SensorManager::ensureLocalTsl2561Ready(uint32_t nowMs) {
	if (localTslReady_) {
		return;
	}

	if (lastTslInitAttemptMs_ != 0 && (nowMs - lastTslInitAttemptMs_) < kTslInitRetryMs) {
		return;
	}

	initializeLocalTsl2561(nowMs);
}

void SensorManager::pollLocalTsl2561(uint32_t nowMs) {
	// pollLocalTsl2561() ist die einzige Stelle, an der echte Rohkanalwerte des
	// TSL2561 gelesen und in publizierte Luxdaten ueberfuehrt werden.
	nextTslPollDueMs_ = nowMs + kTslPollIntervalMs;

	uint16_t broadband = 0;
	uint16_t ir = 0;
	const uint32_t readStartedMs = millis();
	if (!readTslChannels(&broadband, &ir)) {
		if ((millis() - readStartedMs) > kTslI2cOperationBudgetMs) {
			localTslReady_ = false;
			markTsl2561Fault(Tsl2561DiagnosticReason::ReadFailure);
			return;
		}

		if (tslReadFailureCount_ < 0xFF) {
			++tslReadFailureCount_;
		}

		if (tslReadFailureCount_ >= kTslReadFailureThreshold) {
			localTslReady_ = false;
			markTsl2561Fault(Tsl2561DiagnosticReason::ReadFailure);
		}
		return;
	}

	tslReadFailureCount_ = 0;
	// Interrupt-Latch im TSL2561 freigeben, damit der INT-Pin wieder auf HIGH geht
	clearTslInterrupt();

	if (broadband == kTslChannelSaturation || ir == kTslChannelSaturation) {
		publishTsl2561Sample(0.0f, LuxSampleValidity::Invalid,
		                    Tsl2561DiagnosticReason::SaturatedSample, nowMs);
		return;
	}

	const float lux = calculateLuxFromChannels(broadband, ir);
	if (isnan(lux) || !isfinite(lux) || lux < 0.0f) {
		publishTsl2561Sample(0.0f, LuxSampleValidity::Invalid,
		                    Tsl2561DiagnosticReason::InvalidSample, nowMs);
		return;
	}

	publishTsl2561Sample(lux, LuxSampleValidity::Valid, Tsl2561DiagnosticReason::None, nowMs);
}

bool SensorManager::writeTslRegister(uint8_t reg, uint8_t value) {
	if (tslBus_ == nullptr) {
		return false;
	}

	tslBus_->beginTransmission(tslAddress_);
	tslBus_->write(static_cast<uint8_t>(kTslCommandBit | reg));
	tslBus_->write(value);
	return tslBus_->endTransmission() == 0;
}

bool SensorManager::readTslChannels(uint16_t* broadbandOut, uint16_t* irOut) {
	if (tslBus_ == nullptr || broadbandOut == nullptr || irOut == nullptr) {
		return false;
	}

	tslBus_->beginTransmission(tslAddress_);
	tslBus_->write(static_cast<uint8_t>(kTslCommandBit | kTslBlockBit | kTslData0LowRegister));
	if (tslBus_->endTransmission(false) != 0) {
		return false;
	}

	const uint8_t expectedBytes = 4;
	if (tslBus_->requestFrom(tslAddress_, expectedBytes) != expectedBytes) {
		return false;
	}

	const uint8_t ch0Low = tslBus_->read();
	const uint8_t ch0High = tslBus_->read();
	const uint8_t ch1Low = tslBus_->read();
	const uint8_t ch1High = tslBus_->read();

	*broadbandOut = static_cast<uint16_t>((static_cast<uint16_t>(ch0High) << 8) | ch0Low);
	*irOut = static_cast<uint16_t>((static_cast<uint16_t>(ch1High) << 8) | ch1Low);
	return true;
}

void SensorManager::clearTslInterrupt() {
	if (tslBus_ == nullptr || tslIntPin_ == kTslIntPinNone) {
		return;
	}
	// Das TSL2561-Interrupt-Latch wird durch einen Write auf das Command-Register
	// mit gesetztem CL-Bit (Bit 6) geloescht: CMD=1 (0x80), CL=1 (0x40) = 0xC0.
	tslBus_->beginTransmission(tslAddress_);
	tslBus_->write(kTslCommandClearInterrupt);
	tslBus_->endTransmission();
}

void SensorManager::releaseTslInterruptBinding() {
	if (s_tsl2561IsrInstance_ != this) {
		return;
	}

	if (tslIntPin_ != kTslIntPinNone) {
		detachInterrupt(digitalPinToInterrupt(tslIntPin_));
	}

	s_tsl2561IsrInstance_ = nullptr;
	tslIntPending_ = false;
	tslIntCapturedMs_ = 0;
}

float SensorManager::calculateLuxFromChannels(uint16_t broadband, uint16_t ir) const {
	// Die Umrechnung bleibt als fester, libm-freier Fixed-Point-Pfad im
	// Sensordienst. Damit wird dieselbe Lux-Semantik fuer alle spaeteren
	// Verbraucher zentral garantiert.
	if (broadband == 0u) {
		return 0.0f;
	}

	uint32_t chScale = kChannelScaleTint1;
	chScale <<= 4;  // fixed 1x gain -> normalize to 16x base

	uint32_t channel0 = (static_cast<uint32_t>(broadband) * chScale) >> kChannelScale;
	uint32_t channel1 = (static_cast<uint32_t>(ir) * chScale) >> kChannelScale;

	if (channel0 == 0u) {
		return 0.0f;
	}

	uint32_t ratio1 = (channel1 << (kRatioScale + 1)) / channel0;
	uint32_t ratio = (ratio1 + 1u) >> 1;

	uint16_t b = 0;
	uint16_t m = 0;
	if (ratio <= kK1T) {
		b = kB1T;
		m = kM1T;
	} else if (ratio <= kK2T) {
		b = kB2T;
		m = kM2T;
	} else if (ratio <= kK3T) {
		b = kB3T;
		m = kM3T;
	} else if (ratio <= kK4T) {
		b = kB4T;
		m = kM4T;
	} else if (ratio <= kK5T) {
		b = kB5T;
		m = kM5T;
	} else if (ratio <= kK6T) {
		b = kB6T;
		m = kM6T;
	} else if (ratio <= kK7T) {
		b = kB7T;
		m = kM7T;
	} else {
		return 0.0f;
	}

	int32_t luxRaw = static_cast<int32_t>((channel0 * b) - (channel1 * m));
	if (luxRaw < 0) {
		luxRaw = 0;
	}

	luxRaw += static_cast<int32_t>(1u << (kLuxScale - 1));
	const uint32_t lux = static_cast<uint32_t>(luxRaw) >> kLuxScale;
	return static_cast<float>(lux);
}

void SensorManager::updateTslAging(uint32_t nowMs) {
	// Ein gueltiges Sample verliert mit der Zeit zuerst seine Freshness und spaeter
	// seine Health. So koennen Engine und UI zwischen "alt, aber noch bekannt" und
	// "wirklich nicht mehr vertrauenswuerdig" unterscheiden.
	if (!state_.tsl2561.initialized || !hasTslSample_ || state_.tsl2561.health == SensorHealth::Fault) {
		state_.tsl2561.sampleFresh = false;
		state_.tsl2561.sampleAgeMs = 0;
		return;
	}

	state_.tsl2561.sampleAgeMs = nowMs - lastTslSampleMs_;
	state_.tsl2561.sampleFresh = state_.tsl2561.sampleAgeMs <= kTslFreshWindowMs;

	if (state_.tsl2561.sampleAgeMs > kTslStaleWindowMs) {
		state_.tsl2561.health = SensorHealth::Stale;
		state_.tsl2561.diagnosticReason = Tsl2561DiagnosticReason::StaleSample;
	} else {
		state_.tsl2561.health = SensorHealth::Ok;
		if (state_.tsl2561.sampleValidity == LuxSampleValidity::Valid) {
			state_.tsl2561.diagnosticReason = Tsl2561DiagnosticReason::None;
		}
	}
}

void SensorManager::updateThermalState() {
	// ThermalState ist eine grobe, aber stabile fachliche Verdichtung des rohen
	// DS18B20-Zustands. Die feinere Leistungsdrosselung berechnet spaeter erst die
	// ExposureEngine aus dem Temperaturwert selbst.
	if (!state_.ds18b20.initialized) {
		state_.ds18b20.thermalState = ThermalState::Unknown;
		return;
	}

	if (state_.ds18b20.health == SensorHealth::Fault) {
		state_.ds18b20.thermalState = ThermalState::Fault;
		return;
	}

	if (state_.ds18b20.temperatureCelsius >= thermalCriticalThresholdCelsius_) {
		state_.ds18b20.thermalState = ThermalState::Critical;
		return;
	}

	if (state_.ds18b20.temperatureCelsius >= thermalDeratingThresholdCelsius_) {
		state_.ds18b20.thermalState = ThermalState::Derating;
		return;
	}

	state_.ds18b20.thermalState = ThermalState::Normal;
}

}  // namespace dukatimer