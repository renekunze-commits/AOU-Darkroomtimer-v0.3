/*
 * Dukatimer-Part2 - Teensy bootstrap and local runtime wiring
 *
 * Zweck:
 * - ist der einzige Ort, an dem die konkreten Teensy-Hardwareobjekte erzeugt
 *   und miteinander verdrahtet werden
 * - uebersetzt rohe Hardwareaenderungen (Encoder, Taster, Touch, ESP-Frames)
 *   in normierte Eingabeereignisse und Snapshot-Daten
 * - koppelt die lokalen Laufzeitdienste in fester Reihenfolge: Input,
 *   ModeCoordinator, SensorManager, ExposureEngine, Head-Arbiter und LVGL
 *
 * Architekturrolle:
 * - main.cpp darf takten, verdrahten und weiterreichen, aber keine
 *   modusspezifische Mathematik, keine UI-Formatierung und keine verstreute
 *   Sensorfachlogik aufnehmen
 * - diese Datei bildet damit bewusst den schmalen Integrationsrand zwischen
 *   Hardware, Laufzeitdiensten und UI
 *
 * Status:
 * - die Datei ist weiterhin Bring-up-nah, enthaelt aber bereits die produktive
 *   Hauptroute fuer Event-Dispatch, Snapshot-Aufbau und Belichtungsverdrahtung
 */
#include <Arduino.h>
#include <cstring>
#include <Encoder.h>

#include <DukatimerProtocol.h>
#include <IntervalTimer.h>

#include "DukaTeenBoardPins.h"
#include "ExposureEngine.h"
#include "EspServiceLink.h"
constexpr bool kEnableMainLoopWatchdog = true;
constexpr uint32_t kMainLoopWatchdogTimeoutMs = 2500;
constexpr uint32_t kMainLoopWatchdogCheckIntervalUs = 100000;
constexpr uint32_t kMainLoopWatchdogResetMagic = 0x44554757u;
#include "FirmwareVersion.h"
#include "HeadCalibrationProfile.h"
#include "HeadLightArbiter.h"
#include "HeadSpectrumCommand.h"
#include "HeadSpectrumMapper.h"
#include "IModeWorkflow.h"
#include "InputNormalizer.h"
#include "InputRouterPolicy.h"
#include "LightController.h"
#include "LvglUi.h"
#include "MeasurementDomainService.h"
#include "RemoteCommandTracker.h"
#include "RotaryEncoderDriver.h"
#include "UiModalAction.h"
IntervalTimer mainLoopWatchdogTimer;
volatile uint32_t mainLoopWatchdogHeartbeatMs = 0;
volatile bool mainLoopWatchdogArmed = false;

void mainLoopWatchdogIsr() {
	if (!mainLoopWatchdogArmed) {
		return;
	}

	const uint32_t nowMs = millis();
	const uint32_t lastHeartbeatMs = mainLoopWatchdogHeartbeatMs;
	if ((nowMs - lastHeartbeatMs) < kMainLoopWatchdogTimeoutMs) {
		return;
	}

	SRC_GPR5 = kMainLoopWatchdogResetMagic;
	__disable_irq();
	SCB_AIRCR = 0x05FA0004;
	asm volatile("dsb" ::: "memory");
	while (true) {
		asm volatile("nop");
	}
}

void beginMainLoopWatchdog(uint32_t nowMs) {
	if (!kEnableMainLoopWatchdog) {
		return;
	}

	mainLoopWatchdogHeartbeatMs = nowMs;
	mainLoopWatchdogArmed = true;
	mainLoopWatchdogTimer.begin(mainLoopWatchdogIsr, kMainLoopWatchdogCheckIntervalUs);
}

void feedMainLoopWatchdog(uint32_t nowMs) {
	if (!kEnableMainLoopWatchdog) {
		return;
	}

	mainLoopWatchdogHeartbeatMs = nowMs;
}
#include "ModeCoordinator.h"
#include "ModeWorkflowServices.h"
#include "BlackWhiteWorkflow.h"
#include "NeoPixelHead.h"
#include "NeoPixelHeadPatterns.h"
#include "NeoPixelHeadPresets.h"
#include "NormalizedInputEvent.h"
#include "PaperWorkflow.h"
#include "PaperSlotBankCommandAdapter.h"
#include "PaperSlotBankQueryAdapter.h"
#include "PaperSlotPersistenceCodec.h"
#include "PaperSlotStorage.h"
#include "SensorManager.h"
#include "SetupWorkflow.h"
#include "SplitgradeWorkflow.h"
#include "SystemSettingsAdapters.h"
#include "SystemSettingsPersistenceCodec.h"
#include "SystemSettingsStorage.h"
#include "SystemSnapshot.h"
#include "TeensyStoragePolicy.h"
#include "TouchSampler.h"

extern uint8_t external_psram_size;

namespace {

using namespace dukatimer::board;

static_assert(dukatimer::protocol::kProtocolVersion == 1, "Unexpected SharedProtocol version");

constexpr uint8_t TFT_ROTATION = 3;
constexpr uint32_t SNAPSHOT_REFRESH_MS = 120;
constexpr uint16_t TOUCH_MIN_Z = 200;
constexpr uint32_t LOCAL_LONG_PRESS_MS = 650;
constexpr uint32_t LOCAL_REPEAT_START_MS = 1000;
constexpr uint32_t LOCAL_REPEAT_INTERVAL_MS = 250;
constexpr uint8_t EXPOSURE_SOFT_GREEN = 255;
constexpr uint8_t EXPOSURE_HARD_BLUE = 255;
// Uncalibriertes Standardprofil. Wird aktiv, sobald Kopfkalibrierung eingelesen wird.
// profile.calibrated == false -> HeadSpectrumMapper nutzt den Identity-Fallback.
dukatimer::HeadCalibrationProfile headCalibrationProfile;
constexpr uint32_t REMOTE_RENDER_KEEPALIVE_MS = 1000;
constexpr uint8_t kRemoteDispatchBudgetPerLoop = 4;
constexpr uint8_t REMOTE_GATEWAY_TARGET_ANY = 0;
constexpr dukatimer::ModeId kInitialMode = dukatimer::ModeId::Splitgrade;
constexpr dukatimer::NeoPixelHeadTopologyPreset kActiveHeadTopologyPreset =
	dukatimer::NeoPixelHeadTopologyPreset::SingleMatrix16x16;
constexpr dukatimer::NeoPixelHeadTestPattern kSetupHeadTimingTestPattern =
	dukatimer::NeoPixelHeadTestPattern::SegmentCornerMarkers;
constexpr uint32_t kHeadTimingSerialReportIntervalMs = 1000;
constexpr uint32_t kExposurePreWaitDeadlockRecoveryMs = 1500;
constexpr uint32_t kExposurePostWaitDeadlockRecoveryMs = 2000;
constexpr uint32_t kExposurePausedDeadlockRecoveryMs = 15000;
constexpr uint32_t kExposureTimeZeroRemainingDeadlockRecoveryMs = 1200;
constexpr long kEncoder1CountsPerDetent = 4;
constexpr long kEncoder2CountsPerDetent = 4;
constexpr long kEncoder3CountsPerDetent = 4;

[[noreturn]] void haltStartupFailure(const char* reason) {
	// Der aktuelle Produktstand setzt eine funktionsfaehige lokale UI als
	// sichere Bedienoberflaeche voraus. Wenn der Bring-up diese Basisschicht
	// nicht herstellen kann, bleibt der Start bewusst fail-closed, statt spaeter
	// mit unvollstaendigem UI-/Belichtungspfad weiterzulaufen.
	Serial.printf("[FATAL] startup halted: %s\n", reason);
	while (true) {
		yield();
	}
}

bool startupPsramReady() {
	// Der Teensy-Core meldet fehlendes oder defektes externes RAM ueber
	// `external_psram_size == 0`. Diese Vorbedingung wird direkt am Systemstart
	// geprueft, bevor weitere Dienste von LVGL-/EXTMEM-Nutzung ausgehen.
	if (external_psram_size != 0u) {
		return true;
	}

	Serial.println("[RAM] external PSRAM missing - startup rejected before subsystem init");
	return false;
}

dukatimer::NeoPixelHeadConfig buildActiveNeoPixelHeadConfig() {
	// Das physische Pin-Mapping des aktiven Lichtkopfs wird einmalig aus dem
	// gewaehlten Topologie-Preset und den konkreten Board-Pins aufgebaut. Alle
	// spaeteren Head-Kommandos arbeiten nur noch gegen dieses neutrale Config-Objekt.
	dukatimer::NeoPixelHeadPinout pinout;
	pinout.neo1Pin = PIN_NEO_1;
	pinout.neo2Pin = PIN_NEO_2;
	pinout.neo3Pin = PIN_NEO_3;
	pinout.neo4Pin = PIN_NEO_4;
	return dukatimer::buildNeoPixelHeadConfig(kActiveHeadTopologyPreset, pinout);
}

void copyPaperSlotName(char* destination, size_t capacity, const char* source) {
	if (destination == nullptr || capacity == 0u) {
		return;
	}

	snprintf(destination, capacity, "%s", source != nullptr ? source : "");
}

dukatimer::PaperGradeMode sanitizePaperGradeMode(dukatimer::PaperGradeMode gradeMode) {
	return dukatimer::isValidPaperGradeMode(gradeMode)
		? gradeMode
		: dukatimer::PaperGradeMode::Multigrade;
}

void copyPaperSlotSummary(dukatimer::PaperSlotUiSummary& destination,
	                      const dukatimer::PaperExposureProfile& source) {
	destination = {};
	destination.available = true;
	destination.calibrated = source.calibrated;
	destination.gradeMode = sanitizePaperGradeMode(source.gradeMode);
	destination.useIsoMath = source.useIsoMath;
	destination.fixedGradeValue = source.fixedGradeValue;
	copyPaperSlotName(destination.name, sizeof(destination.name), source.name);
}

void copyPaperProfileDetail(dukatimer::PaperProfileUiDetail& destination,
	                       const dukatimer::PaperExposureProfile& source) {
	destination = {};
	destination.available = true;
	destination.calibrated = source.calibrated;
	destination.gradeMode = sanitizePaperGradeMode(source.gradeMode);
	destination.useIsoMath = source.useIsoMath;
	destination.fixedGradeValue = source.fixedGradeValue;
	destination.isoP = source.isoP;
	destination.isoR = source.isoR;
	copyPaperSlotName(destination.name, sizeof(destination.name), source.name);
}

bool normalizeActivePaperSlotSelection(dukatimer::PaperSlotBank& bank) {
	bool changed = false;
	if (bank.slotCount == 0u || bank.slotCount > dukatimer::kPaperSlotCount) {
		bank.slotCount = dukatimer::kPaperSlotCount;
		changed = true;
	}

	if (bank.activeSlot >= bank.slotCount) {
		bank.activeSlot = 0u;
		changed = true;
	}

	if (!bank.slots[bank.activeSlot].calibrated) {
		for (uint8_t slotIndex = 0u; slotIndex < bank.slotCount; ++slotIndex) {
			if (bank.slots[slotIndex].calibrated) {
				if (bank.activeSlot != slotIndex) {
					bank.activeSlot = slotIndex;
					changed = true;
				}
				break;
			}
		}
	}

	return changed;
}

bool shouldAutoPersistDefaultPaperSlotBank(dukatimer::PaperSlotStorageError error) {
	// Sicherheitsgrenze: Nur ein wirklich fehlender Persistenz-Blob wird beim
	// Boot automatisch mit Defaults initialisiert. Parse-, Lese- oder
	// Bankfehler koennen auf korrupte oder nur transient unlesbare Nutzdaten
	// hindeuten und duerfen deshalb nicht still durch Defaultdaten ersetzt werden.
	return error == dukatimer::PaperSlotStorageError::FileNotFound;
}

bool shouldAutoPersistDefaultSystemSettings(dukatimer::SystemSettingsStorageError error) {
	return error == dukatimer::SystemSettingsStorageError::FileNotFound;
}

struct DebouncedInput {
	// DebouncedInput entprellt diskrete Eingaben ueber ein einfaches
	// "Ruhezeit nach Flankenwechsel"-Modell. Diese Schicht ist bewusst nur fuer
	// Taster und Schalter gedacht, nicht fuer die A/B-Spuren der Encoder.
	uint8_t pin;
	bool activeLow;
	bool stableState;
	bool lastRawState;
	uint32_t lastChangeMs;

	void begin(uint8_t inputPin, bool useActiveLow = true) {
		pin = inputPin;
		activeLow = useActiveLow;
		pinMode(pin, activeLow ? INPUT_PULLUP : INPUT);
		const bool raw = readRaw();
		stableState = raw;
		lastRawState = raw;
		lastChangeMs = millis();
	}

	bool readRaw() const {
		const int level = digitalRead(pin);
		return activeLow ? (level == LOW) : (level == HIGH);
	}

	bool update(uint32_t nowMs, uint32_t debounceMs = 20) {
		const bool raw = readRaw();
		if (raw != lastRawState) {
			lastRawState = raw;
			lastChangeMs = nowMs;
		}

		if ((nowMs - lastChangeMs) >= debounceMs && stableState != raw) {
			stableState = raw;
			return true;
		}

		return false;
	}

	bool isActive() const {
		return stableState;
	}
};

struct ButtonGestureState {
	bool held = false;
	bool longPressEmitted = false;
	uint32_t pressedAtMs = 0;
	uint32_t nextRepeatMs = 0;

	void reset() {
		held = false;
		longPressEmitted = false;
		pressedAtMs = 0;
		nextRepeatMs = 0;
	}
};

// SdFat ist bereits eingebunden, wird in dieser Basis aber nur reserviert.
// Die eigentliche Karteninitialisierung folgt erst mit dem Storage- und
// Papierprofilmodell.
dukatimer::TeensyStorageVolume sd;
EXTMEM dukatimer::PaperSlotBank paperSlotBank;
dukatimer::PaperSlotStorage paperSlotStorage;
dukatimer::PaperSlotBankCommandAdapter paperProfileCommands(paperSlotBank, paperSlotStorage);
dukatimer::PaperSlotBankQueryAdapter paperProfileQuery(paperSlotBank, paperSlotStorage);
dukatimer::SystemSettings systemSettings = dukatimer::makeDefaultSystemSettings();
dukatimer::SystemSettingsStorage systemSettingsStorage;
dukatimer::SystemSettingsCommandAdapter systemSettingsCommands(systemSettings, systemSettingsStorage);
dukatimer::SystemSettingsQueryAdapter systemSettingsQuery(systemSettings, systemSettingsStorage);

Encoder enc1(PIN_ENC1_A, PIN_ENC1_B);
Encoder enc2(PIN_ENC2_A, PIN_ENC2_B);
Encoder enc3(PIN_ENC3_A, PIN_ENC3_B);

dukatimer::TouchSampler touchSampler(PIN_TOUCH_CS, PIN_TOUCH_IRQ, TFT_ROTATION, TOUCH_MIN_Z);
dukatimer::LightController lightController(PIN_SSR_ROOM);
dukatimer::ExposureEngine exposureEngine;
dukatimer::SensorManager sensorManager;
dukatimer::MeasurementDomainService measurementDomain;
EXTMEM dukatimer::EspServiceLink espServiceLink(Serial3, PIN_S3_RTS, PIN_S3_CTS);
const dukatimer::NeoPixelHeadConfig headConfig = buildActiveNeoPixelHeadConfig();
dukatimer::NeoPixelHead neoPixelHead(headConfig);
dukatimer::HeadLightArbiter headLightArbiter;
dukatimer::SplitgradeWorkflow sgModeWorkflow;
dukatimer::PaperWorkflow paperModeWorkflow;
dukatimer::BlackWhiteWorkflow bwModeWorkflow;
dukatimer::SetupWorkflow setupModeWorkflow;
dukatimer::IModeWorkflow* modeWorkflows[] = {
	&sgModeWorkflow,
	&paperModeWorkflow,
	&bwModeWorkflow,
	&setupModeWorkflow,
};
dukatimer::ModeCoordinator modeCoordinator(modeWorkflows,
	                                       sizeof(modeWorkflows) / sizeof(modeWorkflows[0]));
dukatimer::ModeWorkflowServices workflowServices;

struct UiRuntime {
	UiRuntime(dukatimer::TouchSampler& touchSampler)
		: presenter(),
		  ui({PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST, PIN_TFT_BL, TFT_ROTATION, ILI9488_SPICLOCK, false},
		     touchSampler,
		     presenter) {}

	dukatimer::UiPresenter presenter;
	dukatimer::LvglUi ui;
};

// Presenter und UI werden bewusst unter einem gemeinsamen Besitzer erzeugt.
// Dadurch ist die Lebensdauerbeziehung explizit: `presenter` wird vor `ui`
// konstruiert und erst nach `ui` wieder zerstoert.
UiRuntime uiRuntime(touchSampler);
dukatimer::UiPresenter& uiPresenter = uiRuntime.presenter;
dukatimer::LvglUi& ui = uiRuntime.ui;

dukatimer::RotaryEncoderDriver encoder1;
dukatimer::RotaryEncoderDriver encoder2;
dukatimer::RotaryEncoderDriver encoder3;

DebouncedInput startButton;
DebouncedInput focusSwitch;
DebouncedInput saveSwitch;
DebouncedInput roomSwitch;
ButtonGestureState encoder1ButtonGesture;
ButtonGestureState encoder2ButtonGesture;
ButtonGestureState encoder3ButtonGesture;
ButtonGestureState startButtonGesture;
dukatimer::InputNormalizer inputNormalizer;
dukatimer::InputRouterPolicy inputRouter;
dukatimer::RemoteCommandTracker remoteCommandTracker;
uint32_t localInputSequence = 1;
uint32_t remoteCommandSequence = 1;
uint32_t remoteDisplaySequence = 1;
dukatimer::RemoteCommandTrackerStatus lastTelemeteredRemoteCommandTrackerStatus = {};
uint32_t lastRemoteRenderTxMs = 0;
bool remoteRenderInitialized = false;
bool deferredRemoteInputEventPending = false;
dukatimer::protocol::RemoteDisplayPayload lastRemoteDisplayPayload = {};
dukatimer::SplitgradeExecutionState lastRemoteExecutionState = dukatimer::SplitgradeExecutionState::Inactive;
dukatimer::BwExecutionState lastRemoteBwExecutionState = dukatimer::BwExecutionState::Inactive;
dukatimer::NormalizedInputEvent deferredRemoteInputEvent = {};
dukatimer::ModeId setupReturnMode = kInitialMode;
dukatimer::ModeId printReturnMode = kInitialMode;
dukatimer::SystemSettings lastAppliedSystemSettings = {};
bool systemSettingsApplied = false;
bool suppressNextSetupToggleMeasureEvent = false;
bool localDoseControlForcedTime = false;
bool localDoseControlWatchdogResetLatched = false;
uint8_t localDoseControlDiagnosticReasonCode =
	static_cast<uint8_t>(dukatimer::Tsl2561DiagnosticReason::None);

elapsedMillis uiSnapshotElapsed;
elapsedMillis lvglTickElapsed;
elapsedMillis headTimingReportElapsed;
uint32_t lastObservedHeadAppliedFrames = 0;

struct HeadTimingDiagnosticsRuntimeState {
	dukatimer::NeoPixelHeadTestPattern activeTestPattern = dukatimer::NeoPixelHeadTestPattern::Disabled;
	bool serialReportEnabled = false;
};

// Dieser Diagnosezustand bleibt absichtlich runtime-only. Setup darf das feste
// Bring-up-Muster fuer den Kopf und den zugehoerigen Serialreport schalten,
// ohne daraus persistente Produktkonfiguration zu machen.
HeadTimingDiagnosticsRuntimeState headTimingDiagnosticsState;

class HeadTimingDiagnosticsAdapter final
	: public dukatimer::HeadTimingDiagnosticsCommandPort,
	  public dukatimer::HeadTimingDiagnosticsQueryPort {
public:
	HeadTimingDiagnosticsAdapter(HeadTimingDiagnosticsRuntimeState& state,
	                           dukatimer::NeoPixelHead& head,
	                           elapsedMillis& reportElapsed,
	                           uint32_t& lastObservedAppliedFrames)
		: state_(state),
		  head_(head),
		  reportElapsed_(reportElapsed),
		  lastObservedAppliedFrames_(lastObservedAppliedFrames) {}

	bool headTimingDiagnosticsEnabled() const override {
		return state_.serialReportEnabled &&
		       state_.activeTestPattern != dukatimer::NeoPixelHeadTestPattern::Disabled;
	}

	void setHeadTimingDiagnosticsEnabled(bool enabled) override {
		state_.activeTestPattern = enabled ? kSetupHeadTimingTestPattern
		                                  : dukatimer::NeoPixelHeadTestPattern::Disabled;
		state_.serialReportEnabled = enabled;
		head_.resetPresentStats();
		reportElapsed_ = 0u;
		lastObservedAppliedFrames_ = 0u;
		Serial.printf("[HEAD] diagnostics %s (%s, report=1s)\n",
		             enabled ? "enabled" : "disabled",
		             enabled ? "SegmentCornerMarkers" : "normal-output");
	}

private:
	HeadTimingDiagnosticsRuntimeState& state_;
	dukatimer::NeoPixelHead& head_;
	elapsedMillis& reportElapsed_;
	uint32_t& lastObservedAppliedFrames_;
};

HeadTimingDiagnosticsAdapter headTimingDiagnostics(headTimingDiagnosticsState,
	                                              neoPixelHead,
	                                              headTimingReportElapsed,
	                                              lastObservedHeadAppliedFrames);

bool isLocalDoseControlServiceFault(const dukatimer::Tsl2561RuntimeStatus& tslStatus) {
	if (!tslStatus.initialized) {
		return false;
	}

	// Nur Bus-/Dienstfehler zwingen auf Zeitmodus. Normale Messinhalte wie
	// Saettigung oder ein einzelnes invalides Sample duerfen den Dose-Modus nicht
	// global fuer die ganze Session abschalten.
	switch (tslStatus.diagnosticReason) {
		case dukatimer::Tsl2561DiagnosticReason::InitializationFailed:
		case dukatimer::Tsl2561DiagnosticReason::ReadFailure:
		case dukatimer::Tsl2561DiagnosticReason::InterruptWatchdog:
		case dukatimer::Tsl2561DiagnosticReason::StaleSample:
			return true;

		case dukatimer::Tsl2561DiagnosticReason::None:
		case dukatimer::Tsl2561DiagnosticReason::SaturatedSample:
		case dukatimer::Tsl2561DiagnosticReason::InvalidSample:
			return false;
	}

	return false;
}

void updateLocalDoseControlPolicy() {
	const dukatimer::Tsl2561RuntimeStatus& tslStatus = sensorManager.state().tsl2561;
	const bool sensorForcedTime = isLocalDoseControlServiceFault(tslStatus);

	localDoseControlForcedTime = localDoseControlWatchdogResetLatched || sensorForcedTime;
	localDoseControlDiagnosticReasonCode = sensorForcedTime
		? static_cast<uint8_t>(tslStatus.diagnosticReason)
		: static_cast<uint8_t>(dukatimer::Tsl2561DiagnosticReason::None);

	// Die Degradationspolitik bleibt bewusst im Wiring-Layer: Sensordienst und
	// Watchdog liefern nur Fakten, der Workflow bekommt daraus eine harte
	// Laufzeitvorgabe "Dose gesperrt, nur Time".
	sgModeWorkflow.setDoseControlForcedTime(localDoseControlForcedTime);
	bwModeWorkflow.setDoseControlForcedTime(localDoseControlForcedTime);
}

template <size_t N>
void copyRemoteRenderText(char (&destination)[N], const char* source) {
	snprintf(destination, N, "%.*s", static_cast<int>(N - 1u), source != nullptr ? source : "");
}

uint16_t packRemoteCommandDiagnosticDetail(uint8_t inFlightCount, uint8_t maxInFlight) {
	return static_cast<uint16_t>((static_cast<uint16_t>(maxInFlight) << 8u) | inFlightCount);
}

bool sendRemoteCommandDiagnostic(dukatimer::protocol::DiagnosticCode code,
	                            uint16_t detail,
	                            uint32_t counter,
	                            uint32_t nowMs) {
	dukatimer::protocol::DiagnosticPayload payload = {};
	payload.code = static_cast<uint16_t>(code);
	payload.detail = detail;
	payload.counter = counter;
	payload.timestampMs = nowMs;
	return espServiceLink.sendDiagnostic(payload, nowMs);
}

bool isExposureStartBlockedByFileTransaction() {
	const dukatimer::EspLinkRuntimeStatus& linkStatus = espServiceLink.state();
	return linkStatus.vfs.transferActive ||
	       (linkStatus.health != dukatimer::EspLinkHealth::Lost &&
	        linkStatus.remoteFileTransactionActive);
}

uint8_t resolveRemoteDisplayViewType(const dukatimer::SystemSnapshot& snapshot) {
	const dukatimer::ExposurePhase phase = snapshot.exposureState.phase;
	if (phase == dukatimer::ExposurePhase::PreWait || phase == dukatimer::ExposurePhase::Exposing ||
	    phase == dukatimer::ExposurePhase::Paused || phase == dukatimer::ExposurePhase::PostWait ||
	    phase == dukatimer::ExposurePhase::Done || phase == dukatimer::ExposurePhase::Fault) {
		return 2u;
	}

	if (snapshot.modeState.activeMode == dukatimer::ModeId::Splitgrade ||
	    snapshot.modeState.activeMode == dukatimer::ModeId::Paper ||
	    snapshot.modeState.activeMode == dukatimer::ModeId::BlackWhite ||
	    snapshot.modeState.activeMode == dukatimer::ModeId::Setup) {
		return 0u;
	}

	return 1u;
}

uint8_t resolveRemoteDisplayProgressPercent(const dukatimer::SystemSnapshot& snapshot) {
	const dukatimer::ExposureRuntimeState& exposureState = snapshot.exposureState;
	if (exposureState.controlMode == dukatimer::ExposureControlMode::Dose && exposureState.targetDose > 0.0f) {
		const float fraction = exposureState.currentDose / exposureState.targetDose;
		const int percent = static_cast<int>(fraction * 100.0f + 0.5f);
		return static_cast<uint8_t>(percent < 0 ? 0 : (percent > 100 ? 100 : percent));
	}

	if (exposureState.controlMode == dukatimer::ExposureControlMode::Time &&
	    exposureState.phase == dukatimer::ExposurePhase::Exposing && exposureState.remainingTimeSeconds >= 0.0f) {
		const float elapsedMs = static_cast<float>(exposureState.phaseAgeMs);
		const float totalMs = elapsedMs + (exposureState.remainingTimeSeconds * 1000.0f);
		if (totalMs > 1.0f) {
			const int percent = static_cast<int>((elapsedMs / totalMs) * 100.0f + 0.5f);
			return static_cast<uint8_t>(percent < 0 ? 0 : (percent > 100 ? 100 : percent));
		}
	}

	return 255u;
}

dukatimer::protocol::RemoteDisplayPayload buildRemoteDisplayPayload(
	const dukatimer::SystemSnapshot& snapshot) {
	dukatimer::protocol::RemoteDisplayPayload payload = {};
	payload.viewType = resolveRemoteDisplayViewType(snapshot);
	payload.progressPercent = resolveRemoteDisplayProgressPercent(snapshot);

	if (snapshot.modeState.activeMode == dukatimer::ModeId::BlackWhite) {
		copyRemoteRenderText(payload.line1, uiPresenter.getBwHeader(snapshot));
		copyRemoteRenderText(payload.line2, uiPresenter.getBwTargets(snapshot));
		copyRemoteRenderText(payload.line3, uiPresenter.getBwExposureMain(snapshot));
		if (payload.line3[0] == '\0') {
			copyRemoteRenderText(payload.line3, uiPresenter.getOverlayText(snapshot));
		}
	} else if (snapshot.modeState.activeMode == dukatimer::ModeId::Splitgrade ||
	    snapshot.modeState.activeMode == dukatimer::ModeId::Paper ||
	    snapshot.modeState.activeMode == dukatimer::ModeId::Setup) {
		copyRemoteRenderText(payload.line1, uiPresenter.getSgHeader(snapshot));
		copyRemoteRenderText(payload.line2, uiPresenter.getSgTargets(snapshot));
		copyRemoteRenderText(payload.line3, uiPresenter.getSgExposureMain(snapshot));
		if (payload.line3[0] == '\0') {
			copyRemoteRenderText(payload.line3, uiPresenter.getOverlayText(snapshot));
		}
	} else {
		copyRemoteRenderText(payload.line1, uiPresenter.getOverlayText(snapshot));
		copyRemoteRenderText(payload.line2, uiPresenter.getModeInfo(snapshot));
		copyRemoteRenderText(payload.line3, uiPresenter.getGatewayInfo(snapshot));
	}

	return payload;
}

bool isPrintFamilyMode(dukatimer::ModeId modeId) {
	return modeId == dukatimer::ModeId::Splitgrade || modeId == dukatimer::ModeId::BlackWhite;
}

dukatimer::ModeId resolvePrintReturnMode() {
	return isPrintFamilyMode(printReturnMode) ? printReturnMode : kInitialMode;
}

// Sendet den aktuellen ExposureEngine-Zustand an beide Print-Workflows
// (Splitgrade und BlackWhite). Alle Paare von observeExposureState-Aufrufen
// ausserhalb der Execution-Command-Prozessoren gehen ueber diesen Helfer.
void broadcastExposureStateToWorkflows(uint32_t nowMs) {
	sgModeWorkflow.observeExposureState(exposureEngine.state(), nowMs);
	bwModeWorkflow.observeExposureState(exposureEngine.state(), nowMs);
}

bool sendRemoteCommand(dukatimer::protocol::TeensyCommandKind commandKind,
	                 uint16_t argument0,
	                 uint32_t argument1,
	                 uint32_t argument2,
	                 uint32_t nowMs) {
	dukatimer::protocol::TeensyCommandPayload payload = {};
	payload.commandKind = static_cast<uint8_t>(commandKind);
	payload.target = REMOTE_GATEWAY_TARGET_ANY;
	payload.argument0 = argument0;
	payload.argument1 = argument1;
	payload.argument2 = argument2;
	payload.commandSequence = remoteCommandSequence++;
	const bool sent = espServiceLink.sendTeensyCommand(payload, nowMs);
	if (!sent) {
		return false;
	}

	// Retry/Timeout bleibt bewusst im Wiring-Layer zwischen Workflow und
	// Transport. Welche Command-Klassen ueberhaupt verfolgt werden, entscheidet
	// allein der Tracker; main.cpp reagiert nur fail-closed auf Trackingverlust.
	if (!remoteCommandTracker.pushCommand(payload, espServiceLink.state().wireless.commandAckSequence, nowMs)) {
		Serial.println("[REMOTE] command tracker queue full - forcing exposure fault");
		if (exposureEngine.abortWithFault(dukatimer::ExposureFaultReason::InternalFault)) {
			broadcastExposureStateToWorkflows(nowMs);
		}
		return false;
	}

	return true;
}

void sendRemoteMeasurementCommand(bool hardPhase, uint32_t nowMs) {
	(void)sendRemoteCommand(dukatimer::protocol::TeensyCommandKind::RemoteMeasurementStart,
	                     hardPhase ? 1u : 0u, 0u, 0u, nowMs);
}

void cancelRemoteMeasurement(uint32_t nowMs) {
	(void)sendRemoteCommand(dukatimer::protocol::TeensyCommandKind::RemoteMeasurementCancel, 0u, 0u, 0u,
	                     nowMs);
}

void sendRemoteHaptic(dukatimer::protocol::RemoteHapticFeedback feedback, uint32_t nowMs) {
	if (systemSettings.vibrationEnabled == 0u) {
		return;
	}

	(void)sendRemoteCommand(dukatimer::protocol::TeensyCommandKind::RemoteHaptic,
	                     static_cast<uint16_t>(feedback), 0u, 0u, nowMs);
}

void publishRemoteRenderIfNeeded(const dukatimer::SystemSnapshot& snapshot, uint32_t nowMs, bool force = false) {
	const dukatimer::protocol::RemoteDisplayPayload payload = buildRemoteDisplayPayload(snapshot);
	const bool changed = !remoteRenderInitialized ||
	                   memcmp(&payload, &lastRemoteDisplayPayload, sizeof(payload)) != 0;
	const bool keepaliveDue = !remoteRenderInitialized || (nowMs - lastRemoteRenderTxMs) >= REMOTE_RENDER_KEEPALIVE_MS;
	if (!force && !changed && !keepaliveDue) {
		return;
	}

	dukatimer::protocol::RemoteDisplayPayload txPayload = payload;
	txPayload.sequenceNumber = remoteDisplaySequence++;
	if (espServiceLink.sendRemoteRender(txPayload, nowMs)) {
		lastRemoteDisplayPayload = payload;
		lastRemoteRenderTxMs = nowMs;
		remoteRenderInitialized = true;
	}
}

void updateRemoteExecutionFeedback(uint32_t nowMs) {
	const dukatimer::ModeRuntimeState& modeState = modeCoordinator.state();
	if (modeState.activeMode == dukatimer::ModeId::BlackWhite) {
		const dukatimer::BwExecutionState bwState = modeState.bw.executionState;
		if (bwState == lastRemoteBwExecutionState) {
			return;
		}
		lastRemoteBwExecutionState = bwState;
		switch (bwState) {
			case dukatimer::BwExecutionState::Completed:
				cancelRemoteMeasurement(nowMs);
				sendRemoteHaptic(dukatimer::protocol::RemoteHapticFeedback::Done, nowMs);
				break;
			case dukatimer::BwExecutionState::Fault:
				cancelRemoteMeasurement(nowMs);
				sendRemoteHaptic(dukatimer::protocol::RemoteHapticFeedback::Error, nowMs);
				break;
			case dukatimer::BwExecutionState::Aborted:
				cancelRemoteMeasurement(nowMs);
				break;
			case dukatimer::BwExecutionState::Inactive:
			case dukatimer::BwExecutionState::IdleConfig:
			case dukatimer::BwExecutionState::Arming:
			case dukatimer::BwExecutionState::Exposing:
				break;
		}
	} else {
		const dukatimer::SplitgradeExecutionState executionState = modeState.splitgrade.executionState;
		if (executionState == lastRemoteExecutionState) {
			return;
		}
		lastRemoteExecutionState = executionState;
		switch (executionState) {
			case dukatimer::SplitgradeExecutionState::Completed:
				cancelRemoteMeasurement(nowMs);
				sendRemoteHaptic(dukatimer::protocol::RemoteHapticFeedback::Done, nowMs);
				break;
			case dukatimer::SplitgradeExecutionState::Fault:
				cancelRemoteMeasurement(nowMs);
				sendRemoteHaptic(dukatimer::protocol::RemoteHapticFeedback::Error, nowMs);
				break;
			case dukatimer::SplitgradeExecutionState::Aborted:
				cancelRemoteMeasurement(nowMs);
				break;
			case dukatimer::SplitgradeExecutionState::Inactive:
			case dukatimer::SplitgradeExecutionState::IdleConfig:
			case dukatimer::SplitgradeExecutionState::ArmingSoft:
			case dukatimer::SplitgradeExecutionState::ExposingSoft:
			case dukatimer::SplitgradeExecutionState::WaitForFilter:
			case dukatimer::SplitgradeExecutionState::ArmingHard:
			case dukatimer::SplitgradeExecutionState::ExposingHard:
				break;
		}
	}
}

void processRemoteCommandTracking(uint32_t nowMs) {
	const uint32_t ackSequence = espServiceLink.state().wireless.commandAckSequence;
	const dukatimer::RemoteCommandTrackerTickResult trackingResult =
		remoteCommandTracker.tick(ackSequence, nowMs);

	if (trackingResult.hasRetry) {
		(void)espServiceLink.sendTeensyCommand(trackingResult.retryPayload, nowMs);
	}

	if (!trackingResult.fatalTimeout) {
		return;
	}

	Serial.println("[REMOTE] command ack timeout - forcing exposure fault");
	if (exposureEngine.abortWithFault(dukatimer::ExposureFaultReason::InternalFault)) {
		broadcastExposureStateToWorkflows(nowMs);
	}
	// Haptik bleibt unkritisch und wird nicht selbst retried. Das Ereignis dient
	// nur als optionales Nutzerfeedback, falls der Remote-Pfad gerade wieder lebt.
	sendRemoteHaptic(dukatimer::protocol::RemoteHapticFeedback::Error, nowMs);
}

void telemeterRemoteCommandDiagnostics(uint32_t nowMs) {
	const dukatimer::RemoteCommandTrackerStatus& status = remoteCommandTracker.status();
	const uint16_t detail = packRemoteCommandDiagnosticDetail(
		status.inFlightCount, dukatimer::RemoteCommandTracker::kMaxInFlightCommands);

	if (status.retryCount != lastTelemeteredRemoteCommandTrackerStatus.retryCount) {
		if (!sendRemoteCommandDiagnostic(dukatimer::protocol::DiagnosticCode::RemoteCommandRetry,
		                            detail, status.retryCount, nowMs)) {
			return;
		}
		lastTelemeteredRemoteCommandTrackerStatus.retryCount = status.retryCount;
	}

	if (status.saturationCount != lastTelemeteredRemoteCommandTrackerStatus.saturationCount) {
		if (!sendRemoteCommandDiagnostic(dukatimer::protocol::DiagnosticCode::RemoteCommandTrackerSaturated,
		                            detail, status.saturationCount, nowMs)) {
			return;
		}
		lastTelemeteredRemoteCommandTrackerStatus.saturationCount = status.saturationCount;
	}

	if (status.timeoutCount != lastTelemeteredRemoteCommandTrackerStatus.timeoutCount) {
		if (!sendRemoteCommandDiagnostic(dukatimer::protocol::DiagnosticCode::RemoteCommandTimeout,
		                            detail, status.timeoutCount, nowMs)) {
			return;
		}
		lastTelemeteredRemoteCommandTrackerStatus.timeoutCount = status.timeoutCount;
	}
}

FLASHMEM void setupInputs() {
	// Die Eingabeschicht wird frueh initialisiert, damit bereits waehrend des
	// restlichen Bring-up stabile Schalter- und Encoderzustaende vorliegen.
	startButton.begin(PIN_BTN_START, true);
	focusSwitch.begin(PIN_BTN_WHITE, true);
	saveSwitch.begin(PIN_BTN_RED, true);
	roomSwitch.begin(PIN_BTN_ROOM, true);

	encoder1.begin(enc1, PIN_ENC1_A, PIN_ENC1_B, PIN_ENC1_SW, kEncoder1CountsPerDetent);
	encoder2.begin(enc2, PIN_ENC2_A, PIN_ENC2_B, PIN_ENC2_SW, kEncoder2CountsPerDetent);
	encoder3.begin(enc3, PIN_ENC3_A, PIN_ENC3_B, PIN_ENC3_SW, kEncoder3CountsPerDetent);
}

FLASHMEM void setupStorage() {
	// Die SD-Karte bleibt exklusiv Teensy-Besitz. Die eigentliche Initialisierung
	// erfolgt lazy beim ersten VFS-Zugriff, damit Bring-up und UI auch ohne Karte
	// starten koennen.
	espServiceLink.attachStorage(&sd);
	paperSlotStorage.attachStorage(&sd);
	systemSettingsStorage.attachStorage(&sd);
}

FLASHMEM void setupSystemSettingsPersistence() {
	dukatimer::SystemSettingsPersistenceCodec::initializeDefaults(systemSettings);

	const bool loadSucceeded = systemSettingsStorage.load(systemSettings);
	bool allowWriteBack = loadSucceeded;

	if (!loadSucceeded) {
		const dukatimer::SystemSettingsStorageError error = systemSettingsStorage.status().lastError;
		if (shouldAutoPersistDefaultSystemSettings(error)) {
			allowWriteBack = systemSettingsStorage.save(systemSettings);
		}
	}

	if (!allowWriteBack && !loadSucceeded) {
		Serial.printf("[SETUP] system settings load failed e:%u d:%lu\n",
			          static_cast<unsigned>(systemSettingsStorage.status().lastError),
			          static_cast<unsigned long>(systemSettingsStorage.status().detail));
	}
}

FLASHMEM void setupPaperSlotPersistence() {
	// Schritt 3 bleibt nicht-invasiv: immer mit gueltigen Defaults starten.
	dukatimer::PaperSlotPersistenceCodec::initializeDefaultBank(paperSlotBank);

	const bool loadSucceeded = paperSlotStorage.load(paperSlotBank);
	bool allowWriteBack = loadSucceeded;

	if (!loadSucceeded) {
		// Recovery bleibt bewusst konservativ: Defaults werden nur bei fehlender
		// Datei rueckgeschrieben. Lesefehler oder ungueltige Blob-Daten bleiben
		// sichtbar, damit kalibrierte Nutzdaten nicht durch einen transienten
		// Fehler still mit Defaults ueberschrieben werden.
		const dukatimer::PaperSlotStorageError error = paperSlotStorage.status().lastError;
		if (shouldAutoPersistDefaultPaperSlotBank(error)) {
			allowWriteBack = paperSlotStorage.save(paperSlotBank);
		}
	}

	if (normalizeActivePaperSlotSelection(paperSlotBank) && allowWriteBack) {
		(void)paperSlotStorage.save(paperSlotBank);
	}
}

dukatimer::SystemSnapshot buildSystemSnapshot() {
	// Die UI-Schicht bekommt einen reinen Snapshot und greift selbst nicht auf
	// Hardware oder globale Rohzustaende zu. Diese Grenze ist absichtlich hart:
	// Presenter und UI sollen nur sichtbare Laufzeitdaten lesen, niemals selbst
	// in Dienste, Hardware oder Workflows zurueckgreifen.
	dukatimer::SystemSnapshot snapshot;
	snapshot.encoder1Position = encoder1.logicalPosition();
	snapshot.encoder2Position = encoder2.logicalPosition();
	snapshot.encoder3Position = encoder3.logicalPosition();
	snapshot.encoder1Status = encoder1.status();
	snapshot.encoder2Status = encoder2.status();
	snapshot.encoder3Status = encoder3.status();
	snapshot.startButtonActive = startButton.isActive();
	const dukatimer::PaperSlotStorageStatus paperSlotStatus = paperSlotStorage.status();
	snapshot.paperSlotStorageErrorCode = static_cast<uint8_t>(paperSlotStatus.lastError);
	snapshot.paperSlotStorageErrorDetail = paperSlotStatus.detail;
	const uint8_t reportedPaperSlotCount = paperProfileQuery.slotCount();
	snapshot.paperSlotCount = reportedPaperSlotCount > dukatimer::kPaperSlotCount
		? dukatimer::kPaperSlotCount
		: reportedPaperSlotCount;
	snapshot.paperActiveSlot = paperProfileQuery.activeSlotIndex();
	if (snapshot.paperSlotCount == 0u || snapshot.paperActiveSlot >= snapshot.paperSlotCount) {
		snapshot.paperActiveSlot = 0u;
	}
	for (uint8_t slotIndex = 0u; slotIndex < snapshot.paperSlotCount; ++slotIndex) {
		const dukatimer::PaperExposureProfile* profile = paperProfileQuery.profileAt(slotIndex);
		if (profile == nullptr) {
			continue;
		}

		copyPaperSlotSummary(snapshot.paperSlotSummaries[slotIndex], *profile);
	}
	if (const dukatimer::PaperExposureProfile* activePaperProfile = paperProfileQuery.activeProfile();
	    activePaperProfile != nullptr) {
		copyPaperProfileDetail(snapshot.paperActiveProfile, *activePaperProfile);
	}
	snapshot.paperActiveSlotCalibrated =
		snapshot.paperActiveProfile.available && snapshot.paperActiveProfile.calibrated;
	snapshot.paperActiveGradeMode = snapshot.paperActiveProfile.available
		? snapshot.paperActiveProfile.gradeMode
		: dukatimer::PaperGradeMode::Multigrade;
	copyPaperSlotName(snapshot.paperActiveSlotName, sizeof(snapshot.paperActiveSlotName),
	                 snapshot.paperActiveProfile.available ? snapshot.paperActiveProfile.name : "");
	const dukatimer::InputRouterRuntimeState& inputRouterState = inputRouter.state();
	snapshot.inputFocusDomainCode = static_cast<uint8_t>(inputRouterState.focusDomain);
	snapshot.inputModalStateCode = static_cast<uint8_t>(inputRouterState.modalState);
	snapshot.inputEventGuardStateCode = static_cast<uint8_t>(inputRouterState.eventGuardState);
	snapshot.inputControlOwnerCode = static_cast<uint8_t>(inputRouterState.controlOwner);
	snapshot.inputFocusAgeMs = inputRouterState.focusAgeMs;
	snapshot.inputModalAgeMs = inputRouterState.modalAgeMs;
	snapshot.inputEventGuardAgeMs = inputRouterState.eventGuardAgeMs;
	snapshot.inputControlOwnerAgeMs = inputRouterState.controlOwnerAgeMs;
	snapshot.localDoseControlForcedTime = localDoseControlForcedTime;
	snapshot.localDoseControlWatchdogResetLatched = localDoseControlWatchdogResetLatched;
	snapshot.localDoseControlDiagnosticReasonCode = localDoseControlDiagnosticReasonCode;
	snapshot.modeState = modeCoordinator.state();
	snapshot.exposureState = exposureEngine.state();
	snapshot.measurementStatus = measurementDomain.state();
	snapshot.espLinkStatus = espServiceLink.state();
	snapshot.remoteCommandTrackerStatus = remoteCommandTracker.status();
	snapshot.sensorStatus = sensorManager.state();
	snapshot.lightState = lightController.state();
	snapshot.touchState = touchSampler.state();
	return snapshot;
}

dukatimer::NormalizedInputSource mapRemoteInputSource(dukatimer::protocol::RemoteInputSource source) {
	switch (source) {
		case dukatimer::protocol::RemoteInputSource::None:
			return dukatimer::NormalizedInputSource::None;
		case dukatimer::protocol::RemoteInputSource::Encoder4:
			return dukatimer::NormalizedInputSource::RemoteEncoder4;
		case dukatimer::protocol::RemoteInputSource::WirelessEncoder:
			return dukatimer::NormalizedInputSource::WirelessEncoder;
		case dukatimer::protocol::RemoteInputSource::WirelessMeasureButton:
			return dukatimer::NormalizedInputSource::WirelessMeasureButton;
		case dukatimer::protocol::RemoteInputSource::WirelessBackButton:
			return dukatimer::NormalizedInputSource::WirelessBackButton;
		case dukatimer::protocol::RemoteInputSource::WirelessEncoderButton:
			return dukatimer::NormalizedInputSource::WirelessEncoderButton;
	}

	return dukatimer::NormalizedInputSource::None;
}

dukatimer::NormalizedInputEventKind mapRemoteInputEventKind(dukatimer::protocol::InputEventKind eventKind) {
	switch (eventKind) {
		case dukatimer::protocol::InputEventKind::None:
			return dukatimer::NormalizedInputEventKind::None;
		case dukatimer::protocol::InputEventKind::RotateLeft:
			return dukatimer::NormalizedInputEventKind::RotateLeft;
		case dukatimer::protocol::InputEventKind::RotateRight:
			return dukatimer::NormalizedInputEventKind::RotateRight;
		case dukatimer::protocol::InputEventKind::Press:
			return dukatimer::NormalizedInputEventKind::Press;
		case dukatimer::protocol::InputEventKind::LongPress:
			return dukatimer::NormalizedInputEventKind::LongPress;
		case dukatimer::protocol::InputEventKind::Measure:
			return dukatimer::NormalizedInputEventKind::Measure;
		case dukatimer::protocol::InputEventKind::Undo:
			return dukatimer::NormalizedInputEventKind::Undo;
	}

	return dukatimer::NormalizedInputEventKind::None;
}

// Gemeinsame Startlogik fuer alle Print-Modi (SG und BW): fault-clear,
// done-acknowledge, target-clamp, forced-time-override und Engine-Start.
// VFS-Pruefung und modusspezifische Vorbereitungen (z.B. Remote-Measurement
// fuer SG) muss der Aufrufer selbst durchfuehren, bevor er diesen Helfer ruft.
FLASHMEM void executePrintExposureStart(float rawTargetValue,
                                        dukatimer::ExposureControlMode controlMode,
                                        const char* modePrefix,
                                        uint32_t nowMs) {
	(void)nowMs;  // Reserviert fuer zukuenftige zeitabhaengige Erweiterungen.

	if (exposureEngine.state().phase == dukatimer::ExposurePhase::Fault) {
		exposureEngine.clearLatchedFault();
	}
	if (exposureEngine.state().phase == dukatimer::ExposurePhase::Done) {
		exposureEngine.acknowledgeDone();
	}

	const float targetValue = (rawTargetValue < 0.1f) ? 0.1f : rawTargetValue;
	const dukatimer::ExposureControlMode effectiveControlMode =
		(controlMode == dukatimer::ExposureControlMode::Dose && localDoseControlForcedTime)
			? dukatimer::ExposureControlMode::Time
			: controlMode;

	if (controlMode == dukatimer::ExposureControlMode::Dose &&
	    effectiveControlMode == dukatimer::ExposureControlMode::Time) {
		if (localDoseControlWatchdogResetLatched) {
			Serial.printf("[TSL] %s: forcing start to TIME after loop watchdog reset\n", modePrefix);
		} else {
			Serial.printf("[TSL] %s: forcing start to TIME due to local diagnostic %u\n",
			             modePrefix,
			             static_cast<unsigned>(localDoseControlDiagnosticReasonCode));
		}
	}

	if (effectiveControlMode == dukatimer::ExposureControlMode::Dose) {
		exposureEngine.startDoseExposure(targetValue, false);
	} else {
		exposureEngine.startTimeExposure(targetValue, false);
	}
}

void processSplitgradeExecutionCommands(uint32_t nowMs) {
	// SplitgradeWorkflow bleibt oberhalb der physischen Engine und darf nur
	// fachliche Kommandos aeussern. Die konkrete Umsetzung in Time-/Dose-Starts,
	// Fault-Reset oder Done-Acknowledge passiert zentral hier im Wiring-Layer.
	dukatimer::SplitgradeExecutionCommand command;
	while (sgModeWorkflow.consumeExecutionCommand(command)) {
		if (!command.isMeaningful()) {
			continue;
		}

		switch (command.kind) {
			case dukatimer::SplitgradeExecutionCommandKind::StartSoft:
			case dukatimer::SplitgradeExecutionCommandKind::StartHard: {
				if (isExposureStartBlockedByFileTransaction()) {
					Serial.printf("[VFS] blocking exposure start while file transaction active (serial:%u remote:%u)\n",
					             espServiceLink.state().vfs.transferActive ? 1u : 0u,
					             espServiceLink.state().remoteFileTransactionActive ? 1u : 0u);
					exposureEngine.abortWithFault(dukatimer::ExposureFaultReason::StartBlocked);
					break;
				}

				sendRemoteMeasurementCommand(command.kind == dukatimer::SplitgradeExecutionCommandKind::StartHard,
				                         nowMs);
				executePrintExposureStart(command.targetValue, command.controlMode, "SG", nowMs);
				break;
			}

				case dukatimer::SplitgradeExecutionCommandKind::PauseExposure:
					exposureEngine.pause();
					break;

				case dukatimer::SplitgradeExecutionCommandKind::ResumeExposure:
					exposureEngine.resume();
					break;

			case dukatimer::SplitgradeExecutionCommandKind::AbortExposure:
				cancelRemoteMeasurement(nowMs);
				exposureEngine.stopGracefully();
				break;

			case dukatimer::SplitgradeExecutionCommandKind::AcknowledgeDone:
				cancelRemoteMeasurement(nowMs);
				exposureEngine.acknowledgeDone();
				break;

			case dukatimer::SplitgradeExecutionCommandKind::ClearFault:
				cancelRemoteMeasurement(nowMs);
				exposureEngine.clearLatchedFault();
				break;

			case dukatimer::SplitgradeExecutionCommandKind::None:
				break;
		}
	}

	sgModeWorkflow.observeExposureState(exposureEngine.state(), nowMs);
}

void processBlackWhiteExecutionCommands(uint32_t nowMs) {
	// Analog zu processSplitgradeExecutionCommands: BlackWhiteWorkflow bleibt
	// oberhalb der physischen Engine und gibt nur fachliche Kommandos ab.
	dukatimer::BlackWhiteExecutionCommand command;
	while (bwModeWorkflow.consumeExecutionCommand(command)) {
		if (!command.isMeaningful()) {
			continue;
		}

		switch (command.kind) {
			case dukatimer::BlackWhiteExecutionCommandKind::Start: {
				if (isExposureStartBlockedByFileTransaction()) {
					Serial.printf("[VFS] BW: blocking exposure start while file transaction active (serial:%u remote:%u)\n",
					             espServiceLink.state().vfs.transferActive ? 1u : 0u,
					             espServiceLink.state().remoteFileTransactionActive ? 1u : 0u);
					exposureEngine.abortWithFault(dukatimer::ExposureFaultReason::StartBlocked);
					break;
				}

				executePrintExposureStart(command.targetValue, command.controlMode, "BW", nowMs);
				break;
			}

			case dukatimer::BlackWhiteExecutionCommandKind::PauseExposure:
				exposureEngine.pause();
				break;

			case dukatimer::BlackWhiteExecutionCommandKind::ResumeExposure:
				exposureEngine.resume();
				break;

			case dukatimer::BlackWhiteExecutionCommandKind::AbortExposure:
				exposureEngine.stopGracefully();
				break;

			case dukatimer::BlackWhiteExecutionCommandKind::AcknowledgeDone:
				exposureEngine.acknowledgeDone();
				break;

			case dukatimer::BlackWhiteExecutionCommandKind::ClearFault:
				exposureEngine.clearLatchedFault();
				break;

			case dukatimer::BlackWhiteExecutionCommandKind::None:
				break;
		}
	}

	bwModeWorkflow.observeExposureState(exposureEngine.state(), nowMs);
}

bool shouldRecoverStaleExposureWait(const dukatimer::ExposureRuntimeState& exposureState) {
	if (exposureState.phase == dukatimer::ExposurePhase::PreWait) {
		return exposureState.phaseAgeMs > kExposurePreWaitDeadlockRecoveryMs;
	}

	if (exposureState.phase == dukatimer::ExposurePhase::PostWait) {
		return exposureState.phaseAgeMs > kExposurePostWaitDeadlockRecoveryMs;
	}

	if (exposureState.phase == dukatimer::ExposurePhase::Paused) {
		// Pause ist in der aktuellen AP-06/AP-07-Integration kein produktiver
		// Bedienpfad. Bleibt die Engine dort haengen, wird der globale Wait-Lock
		// sonst unbegrenzt. Deshalb fuehren wir konservativ in einen sicheren Fault.
		return exposureState.phaseAgeMs > kExposurePausedDeadlockRecoveryMs;
	}

	if (exposureState.phase == dukatimer::ExposurePhase::Exposing &&
	    exposureState.controlMode == dukatimer::ExposureControlMode::Time &&
	    exposureState.remainingTimeSeconds <= 0.0f) {
		return exposureState.phaseAgeMs > kExposureTimeZeroRemainingDeadlockRecoveryMs;
	}

	return false;
}

void enforceExposureDeadlockRecovery(uint32_t nowMs) {
	const dukatimer::ExposureRuntimeState& exposureState = exposureEngine.state();
	if (!shouldRecoverStaleExposureWait(exposureState)) {
		return;
	}

	// Sicherheitsentscheidung: Bei unmoeglich lange festhaengenden Wait-Phasen
	// wird nicht in `Ready` entsperrt, sondern in einen sichtbaren Fault
	// gewechselt. So bleibt der Guard nicht endlos geschlossen, ohne waehrend
	// einer unklaren Belichtungslage freie Parametereingaben zuzulassen.
	if (exposureEngine.abortWithFault(dukatimer::ExposureFaultReason::InternalFault)) {
		broadcastExposureStateToWorkflows(nowMs);
	}
}

enum class InputDispatchOutcome : uint8_t {
	Dropped,
	Dispatched,
	DeferredRemoteOwnership,
};

InputDispatchOutcome dispatchNormalizedInputEvent(const dukatimer::NormalizedInputEvent& event, uint32_t nowMs) {
	if (suppressNextSetupToggleMeasureEvent &&
	    event.source == dukatimer::NormalizedInputSource::LocalEncoder3) {
		if (event.eventKind == dukatimer::NormalizedInputEventKind::Measure) {
			suppressNextSetupToggleMeasureEvent = false;
			return InputDispatchOutcome::Dropped;
		}

		if (event.eventKind != dukatimer::NormalizedInputEventKind::LongPress) {
			suppressNextSetupToggleMeasureEvent = false;
		}
	}

	// Diese Funktion ist die einzige lokale Hauptroute fuer bereits normierte
	// Eingaben. Reihenfolge ist wichtig:
	// 1. semantische Aktion ableiten
	// 2. modalen Laufzeitzustand beobachten
	// 3. Event+Aktion ggf. blocken
	// 4. an den aktiven Workflow dispatchen
	// 5. daraus resultierende Belichtungskommandos sofort nachziehen
	inputRouter.observeRuntimeState(modeCoordinator.state(), exposureEngine.state(), nowMs);
	if (event.isMeaningful() && inputRouter.state().modalState == dukatimer::InputModalState::None &&
	    exposureEngine.state().phase == dukatimer::ExposurePhase::Idle &&
	    event.eventKind == dukatimer::NormalizedInputEventKind::Press) {
		const dukatimer::ModeRuntimeState& modeState = modeCoordinator.state();
		// Bis produktive EEZ-ModeTabs existieren, nutzt der lokale Placeholder die
		// bisher ungenutzten Enc1-/Enc2-Taps als provisorischen Familienwechsel.
		// Dadurch bleibt Enc3-LongPress exklusiv fuer Setup reserviert und die
		// bestehende Kontextsemantik der Workflows wird nicht aufgebrochen.
		if (event.source == dukatimer::NormalizedInputSource::LocalEncoder1 &&
		    modeState.activeMode != dukatimer::ModeId::Setup &&
		    modeState.activeMode != dukatimer::ModeId::Paper) {
			if (isPrintFamilyMode(modeState.activeMode)) {
				printReturnMode = modeState.activeMode;
			}
			if (modeCoordinator.requestMode(dukatimer::ModeId::Paper, nowMs)) {
				inputRouter.observeDispatchedEvent(event, nowMs);
				return InputDispatchOutcome::Dispatched;
			}
		}

		if (event.source == dukatimer::NormalizedInputSource::LocalEncoder2 &&
		    modeState.activeMode == dukatimer::ModeId::Paper) {
			if (modeCoordinator.requestMode(resolvePrintReturnMode(), nowMs)) {
				inputRouter.observeDispatchedEvent(event, nowMs);
				return InputDispatchOutcome::Dispatched;
			}
		}
	}

	if (event.isMeaningful() && event.source == dukatimer::NormalizedInputSource::LocalEncoder3 &&
	    event.eventKind == dukatimer::NormalizedInputEventKind::LongPress &&
	    inputRouter.state().modalState == dukatimer::InputModalState::None &&
	    exposureEngine.state().phase == dukatimer::ExposurePhase::Idle) {
		const dukatimer::ModeRuntimeState& modeState = modeCoordinator.state();
		if (modeState.activeMode == dukatimer::ModeId::Setup) {
			if (!modeState.setup.editingActive && !modeState.setup.parametersDirty) {
				const dukatimer::ModeId targetMode =
					setupReturnMode == dukatimer::ModeId::None ? kInitialMode : setupReturnMode;
				if (modeCoordinator.requestMode(targetMode, nowMs)) {
					encoder3ButtonGesture.reset();
					suppressNextSetupToggleMeasureEvent = true;
					inputRouter.observeDispatchedEvent(event, nowMs);
					return InputDispatchOutcome::Dispatched;
				}
			}
		} else if (modeState.activeMode != dukatimer::ModeId::None) {
			setupReturnMode = modeState.activeMode;
			if (modeCoordinator.requestMode(dukatimer::ModeId::Setup, nowMs)) {
				encoder3ButtonGesture.reset();
				suppressNextSetupToggleMeasureEvent = true;
				inputRouter.observeDispatchedEvent(event, nowMs);
				return InputDispatchOutcome::Dispatched;
			}
		}
	}

	const dukatimer::InputSemanticAction action = inputNormalizer.normalize(event);
	if (!inputRouter.shouldDispatch(event, action)) {
		return inputRouter.shouldDeferRemoteEvent(event)
			       ? InputDispatchOutcome::DeferredRemoteOwnership
			       : InputDispatchOutcome::Dropped;
	}

	modeCoordinator.handleInputEvent(event, action, nowMs);
	inputRouter.observeDispatchedEvent(event, nowMs);
	broadcastExposureStateToWorkflows(nowMs);
	processSplitgradeExecutionCommands(nowMs);
	processBlackWhiteExecutionCommands(nowMs);
	return InputDispatchOutcome::Dispatched;
}

void emitLocalEncoderRotationEvents(dukatimer::NormalizedInputSource source, long oldPosition, long newPosition,
	                                uint32_t nowMs) {
	if (oldPosition == newPosition) {
		return;
	}

	const long stepDirection = newPosition > oldPosition ? 1 : -1;
	const dukatimer::NormalizedInputEventKind eventKind =
		stepDirection > 0 ? dukatimer::NormalizedInputEventKind::RotateRight
		                  : dukatimer::NormalizedInputEventKind::RotateLeft;
	for (long position = oldPosition; position != newPosition; position += stepDirection) {
		(void)dispatchNormalizedInputEvent(
			dukatimer::makeNormalizedInputEvent(source, eventKind, static_cast<int16_t>(stepDirection), nowMs,
			                                   localInputSequence++),
			nowMs);
	}
}

void emitLocalButtonEvent(dukatimer::NormalizedInputSource source,
	                      dukatimer::NormalizedInputEventKind eventKind,
	                      int16_t value,
	                      uint32_t nowMs) {
	(void)dispatchNormalizedInputEvent(
		dukatimer::makeNormalizedInputEvent(source, eventKind, value, nowMs, localInputSequence++), nowMs);
}

void emitLocalModalButtonAction(dukatimer::UiModalAction action, uint32_t nowMs) {
	if (action == dukatimer::UiModalAction::None) {
		return;
	}

	// Alle globalen Modal-Buttons laufen ueber dieselbe Rohquelle; die konkrete
	// Fachabsicht steckt im Value-Payload und wird erst im Normalizer semantisch.
	emitLocalButtonEvent(dukatimer::NormalizedInputSource::LocalModalButton,
	                 dukatimer::NormalizedInputEventKind::Press,
	                 dukatimer::encodeUiModalAction(action),
	                 nowMs);
}

bool dispatchDeferredRemoteInputEventWithinBudget(uint32_t nowMs, uint8_t& remainingBudget) {
	if (!deferredRemoteInputEventPending) {
		return true;
	}

	if (remainingBudget == 0u) {
		return false;
	}

	const InputDispatchOutcome outcome = dispatchNormalizedInputEvent(deferredRemoteInputEvent, nowMs);
	if (outcome == InputDispatchOutcome::DeferredRemoteOwnership) {
		return false;
	}

	deferredRemoteInputEvent = {};
	deferredRemoteInputEventPending = false;
	--remainingBudget;
	return true;
}

void emitLocalButtonGestures(dukatimer::NormalizedInputSource source,
	                        bool changed,
	                        bool active,
	                        uint32_t nowMs,
	                        ButtonGestureState& gestureState,
	                        bool emitMeasureOnLongPress,
	                        bool emitUndoOnLongPress) {
	// Die lokale Tastergestik wird bewusst vor dem Workflow-Layer verdichtet.
	// Dadurch muessen spaetere Modi nicht selbst rekonstruieren, wann aus einem
	// stabilen Tasterzustand Press, LongPress, Repeat, Measure oder Undo wird.
	if (changed && active) {
		gestureState.held = true;
		gestureState.longPressEmitted = false;
		gestureState.pressedAtMs = nowMs;
		gestureState.nextRepeatMs = nowMs + LOCAL_REPEAT_START_MS;
		emitLocalButtonEvent(source, dukatimer::NormalizedInputEventKind::Press, 1, nowMs);
		return;
	}

	if (changed && !active) {
		gestureState.reset();
		return;
	}

	if (!gestureState.held || !active) {
		return;
	}

	if (!gestureState.longPressEmitted && (nowMs - gestureState.pressedAtMs) >= LOCAL_LONG_PRESS_MS) {
		emitLocalButtonEvent(source, dukatimer::NormalizedInputEventKind::LongPress, 1, nowMs);
		if (emitMeasureOnLongPress) {
			emitLocalButtonEvent(source, dukatimer::NormalizedInputEventKind::Measure, 1, nowMs);
		}
		if (emitUndoOnLongPress) {
			emitLocalButtonEvent(source, dukatimer::NormalizedInputEventKind::Undo, 1, nowMs);
		}
		gestureState.longPressEmitted = true;
	}

	if (gestureState.longPressEmitted && nowMs >= gestureState.nextRepeatMs) {
		emitLocalButtonEvent(source, dukatimer::NormalizedInputEventKind::RepeatPress, 1, nowMs);
		gestureState.nextRepeatMs = nowMs + LOCAL_REPEAT_INTERVAL_MS;
	}
}

void applyEspServiceDataToSensorManager() {
	// Der ESP bleibt fuer DS18B20 und weitere Servicepfade nur Zulieferer.
	// SensorManager bleibt der alleinige Besitzer des sichtbaren Sensorstatus;
	// main.cpp uebersetzt nur das SharedProtocol-Payload in dessen API.
	dukatimer::protocol::EspServiceSnapshotPayload serviceSnapshot;
	if (!espServiceLink.consumeServiceSnapshot(&serviceSnapshot)) {
		return;
	}

	const bool ds18Present = (serviceSnapshot.sensorFlags & dukatimer::protocol::kServiceSensorDs18Present) != 0u;
	const bool ds18Fault = (serviceSnapshot.sensorFlags & dukatimer::protocol::kServiceSensorDs18Fault) != 0u;
	sensorManager.setDs18b20Initialized(ds18Present);
	if (!ds18Present) {
		return;
	}

	if (ds18Fault) {
		sensorManager.markDs18b20Fault();
		return;
	}

	sensorManager.publishDs18b20Temperature(static_cast<float>(serviceSnapshot.ds18TemperatureCentiC) / 100.0f);
}

void consumeEspInputEvents(uint32_t nowMs) {
	uint8_t remainingBudget = kRemoteDispatchBudgetPerLoop;
	if (!dispatchDeferredRemoteInputEventWithinBudget(nowMs, remainingBudget)) {
		return;
	}

	dukatimer::protocol::InputEventPayload inputEvent;
	while (remainingBudget > 0u && espServiceLink.consumeInputEvent(&inputEvent)) {
		const dukatimer::NormalizedInputEvent event = dukatimer::makeNormalizedInputEvent(
			mapRemoteInputSource(static_cast<dukatimer::protocol::RemoteInputSource>(inputEvent.source)),
			mapRemoteInputEventKind(static_cast<dukatimer::protocol::InputEventKind>(inputEvent.eventKind)),
			inputEvent.value, inputEvent.sourceTimestampMs, inputEvent.eventSequence);
		const InputDispatchOutcome outcome = dispatchNormalizedInputEvent(event, nowMs);
		if (outcome == InputDispatchOutcome::DeferredRemoteOwnership) {
			deferredRemoteInputEvent = event;
			deferredRemoteInputEventPending = true;
			return;
		}
		--remainingBudget;
	}
}

void updateInputs(uint32_t nowMs) {
	// Reihenfolge: erst diskrete Inputs und Encoder aktualisieren, danach den
	// zentralen Lichtzustand neu aus den stabilen Eingangswerten ableiten.
	// Erst danach werden normierte Events emittiert. So vermeiden wir, dass
	// halbaktualisierte Input-Zustaende gleichzeitig in Licht- und Workflowpfade
	// einsickern.
	const long oldEncoder1Position = encoder1.logicalPosition();
	const long oldEncoder2Position = encoder2.logicalPosition();
	const long oldEncoder3Position = encoder3.logicalPosition();

	const bool startButtonChanged = startButton.update(nowMs);
	focusSwitch.update(nowMs);
	saveSwitch.update(nowMs);
	roomSwitch.update(nowMs);

	encoder1.update(nowMs);
	encoder2.update(nowMs);
	encoder3.update(nowMs);

	lightController.setLocalSwitches(focusSwitch.isActive(), saveSwitch.isActive(), roomSwitch.isActive());

	emitLocalEncoderRotationEvents(dukatimer::NormalizedInputSource::LocalEncoder1,
	                             oldEncoder1Position,
	                             encoder1.logicalPosition(),
	                             nowMs);
	emitLocalEncoderRotationEvents(dukatimer::NormalizedInputSource::LocalEncoder2,
	                             oldEncoder2Position,
	                             encoder2.logicalPosition(),
	                             nowMs);
	emitLocalEncoderRotationEvents(dukatimer::NormalizedInputSource::LocalEncoder3,
	                             oldEncoder3Position,
	                             encoder3.logicalPosition(),
	                             nowMs);
	emitLocalButtonGestures(dukatimer::NormalizedInputSource::LocalEncoder1,
	                     encoder1.buttonChanged(),
	                     encoder1.buttonActive(),
	                     nowMs,
	                     encoder1ButtonGesture,
	                     true,
	                     false);
	emitLocalButtonGestures(dukatimer::NormalizedInputSource::LocalEncoder2,
	                     encoder2.buttonChanged(),
	                     encoder2.buttonActive(),
	                     nowMs,
	                     encoder2ButtonGesture,
	                     false,
	                     true);
	emitLocalButtonGestures(dukatimer::NormalizedInputSource::LocalEncoder3,
	                     encoder3.buttonChanged(),
	                     encoder3.buttonActive(),
	                     nowMs,
	                     encoder3ButtonGesture,
	                     true,
	                     false);
	emitLocalButtonGestures(dukatimer::NormalizedInputSource::LocalStartButton, startButtonChanged,
	                     startButton.isActive(), nowMs, startButtonGesture, false, false);
}

bool isExposureHeadOverridePhase(dukatimer::ExposurePhase phase) {
	return phase == dukatimer::ExposurePhase::PreWait || phase == dukatimer::ExposurePhase::Exposing ||
	       phase == dukatimer::ExposurePhase::Paused || phase == dukatimer::ExposurePhase::PostWait ||
	       phase == dukatimer::ExposurePhase::Fault;
}

bool isSplitgradeHardExecutionState(dukatimer::SplitgradeExecutionState executionState) {
	return executionState == dukatimer::SplitgradeExecutionState::ArmingHard ||
	       executionState == dukatimer::SplitgradeExecutionState::ExposingHard;
}

uint8_t applyRuntimeOutputLimitToChannel(uint8_t channel, float runtimeOutputLimit) {
	if (runtimeOutputLimit <= 0.0f) {
		return 0;
	}

	if (runtimeOutputLimit >= 1.0f) {
		return channel;
	}

	return static_cast<uint8_t>((static_cast<float>(channel) * runtimeOutputLimit) + 0.5f);
}

float currentHeadBrightnessLimit() {
	const uint8_t brightnessPercent = systemSettings.maxHeadBrightnessPercent;
	if (brightnessPercent >= 100u) {
		return 1.0f;
	}

	if (brightnessPercent == 0u) {
		return 0.0f;
	}

	return static_cast<float>(brightnessPercent) / 100.0f;
}

dukatimer::LightRgb applyRuntimeOutputLimit(const dukatimer::LightRgb& color, float runtimeOutputLimit) {
	return dukatimer::makeLightRgb(applyRuntimeOutputLimitToChannel(color.red, runtimeOutputLimit),
	                              applyRuntimeOutputLimitToChannel(color.green, runtimeOutputLimit),
	                              applyRuntimeOutputLimitToChannel(color.blue, runtimeOutputLimit));
}

dukatimer::HeadLightCommand applyGlobalHeadBrightnessLimit(const dukatimer::HeadLightCommand& command) {
	if (command.mode != dukatimer::HeadLightMode::SolidColor) {
		return command;
	}

	const float brightnessLimit = currentHeadBrightnessLimit();
	if (brightnessLimit >= 0.999f) {
		return command;
	}

	return dukatimer::makeHeadLightSolid(applyRuntimeOutputLimit(command.color, brightnessLimit));
}

// Liefert die fachliche Spektrum-Beschreibung fuer den aktuell aktiven
// Belichtungskanal. Diese Funktion entscheidet NUR die fotografische Semantik
// (welcher Kanal, welche Mischung), nicht die physischen RGB-Werte.
FLASHMEM dukatimer::HeadSpectrumCommand resolveExposureSpectrum() {
	const dukatimer::ModeRuntimeState& modeState = modeCoordinator.state();
	dukatimer::HeadSpectrumCommand cmd;

	if (modeState.activeMode == dukatimer::ModeId::Splitgrade) {
		if (isSplitgradeHardExecutionState(modeState.splitgrade.executionState)) {
			cmd.semantic = dukatimer::HeadSpectrumSemantic::SplitgradeHard;
			cmd.channels.hard = 1.0f;
		} else {
			cmd.semantic = dukatimer::HeadSpectrumSemantic::SplitgradeSoft;
			cmd.channels.soft = 1.0f;
		}
		return cmd;
	}

	if (modeState.activeMode == dukatimer::ModeId::BlackWhite) {
		const dukatimer::BwModeRuntimeState& bw = modeState.bw;
		if (bw.whiteLight) {
			// FixedGrade: Weisslicht, beide fotografischen Kanaele voll.
			cmd.semantic = dukatimer::HeadSpectrumSemantic::BwWhite;
			cmd.channels.soft = 1.0f;
			cmd.channels.hard = 1.0f;
		} else {
			// Multigrade: Kanalverhaeltnis gemaess Gradationseinstellung.
			cmd.semantic = dukatimer::HeadSpectrumSemantic::BwGradeMix;
			cmd.channels.soft = bw.softMix;
			cmd.channels.hard = bw.hardMix;
		}
		return cmd;
	}

	// Sonstige Modi: Weisslicht als sicherer Fallback.
	cmd.semantic = dukatimer::HeadSpectrumSemantic::BwWhite;
	cmd.channels.soft = 1.0f;
	cmd.channels.hard = 1.0f;
	return cmd;
}

dukatimer::HeadLightSourceState resolveExposureHeadSource() {
	// ExposureEngine entscheidet nur, ob belichtet wird und wie stark thermisch
	// gedrosselt werden muss. Die Aufbereitung zu einer konkreten Head-Quelle
	// passiert hier im Wiring-Layer, bevor der Head-Arbiter die lokalen Quellen
	// zusammenfuehrt.
	const dukatimer::ExposureRuntimeState& exposureState = exposureEngine.state();
	if (!isExposureHeadOverridePhase(exposureState.phase)) {
		return dukatimer::makeInactiveHeadLightSource();
	}

	if (exposureState.phase != dukatimer::ExposurePhase::Exposing) {
		return dukatimer::makeActiveHeadLightSource(dukatimer::makeHeadLightOff());
	}

	const dukatimer::HeadSpectrumCommand spectrum = resolveExposureSpectrum();
	const dukatimer::LightRgb baseColor = dukatimer::HeadSpectrumMapper::map(spectrum, headCalibrationProfile);
	const dukatimer::LightRgb limitedColor =
		applyRuntimeOutputLimit(baseColor, exposureState.runtimeOutputLimit);

	if (limitedColor.red == 0 && limitedColor.green == 0 && limitedColor.blue == 0) {
		return dukatimer::makeActiveHeadLightSource(dukatimer::makeHeadLightOff());
	}

	return dukatimer::makeActiveHeadLightSource(dukatimer::makeHeadLightSolid(limitedColor));
}

dukatimer::HeadLightCommand resolveHeadLightCommand() {
	headLightArbiter.setLocalSource(lightController.localHeadSource());
	headLightArbiter.setExposureSource(resolveExposureHeadSource());
	return headLightArbiter.resolvedCommand();
}

void applyHeadOutput() {
	if (headTimingDiagnosticsState.activeTestPattern != dukatimer::NeoPixelHeadTestPattern::Disabled) {
		// Diagnosemuster sind ein expliziter Bring-up-Modus. Sie umgehen bewusst
		// den normalen Head-Arbiter, damit Verkabelung und Segmentorientierung
		// isoliert geprueft werden koennen.
		dukatimer::applyNeoPixelHeadTestPattern(neoPixelHead,
		                                     headConfig,
		                                     headTimingDiagnosticsState.activeTestPattern);
		return;
	}

	neoPixelHead.applyCommand(applyGlobalHeadBrightnessLimit(resolveHeadLightCommand()));
	neoPixelHead.present();
}

bool isMeasurementContextArmed(const dukatimer::ModeRuntimeState& modeState) {
	// Wireless-Messwerte werden akzeptiert, wenn der Bediener aktiv im
	// Messpanel ist. Das gilt fuer SG und BW gleichermassen, damit das
	// Handteil in beiden Modi Messwerte einliefern kann. (AP-BW-03)
	if (modeState.activeMode == dukatimer::ModeId::Splitgrade) {
		return modeState.splitgrade.panel == dukatimer::SplitgradePanel::Measurement;
	}
	if (modeState.activeMode == dukatimer::ModeId::BlackWhite) {
		return modeState.bw.panel == dukatimer::BwPanel::Measurement;
	}
	return false;
}

void applySystemSettingsToRuntimeIfNeeded() {
	if (systemSettingsApplied && systemSettings == lastAppliedSystemSettings) {
		return;
	}

	sensorManager.setThermalThresholds(systemSettings.thermalProtection.deratingStartCelsius,
	                               systemSettings.thermalProtection.hardStopCelsius);
	exposureEngine.setThermalProtectionConfig(systemSettings.thermalProtection.deratingStartCelsius,
	                                      systemSettings.thermalProtection.hardStopCelsius);
	lastAppliedSystemSettings = systemSettings;
	systemSettingsApplied = true;
}

void syncHeadTimingIntoExposureEngine() {
	const dukatimer::NeoPixelHeadPresentStats& stats = neoPixelHead.presentStats();
	if (stats.appliedFrames == 0 || stats.lastDurationUs == 0 ||
	    stats.appliedFrames == lastObservedHeadAppliedFrames) {
		return;
	}

	lastObservedHeadAppliedFrames = stats.appliedFrames;

	// Fuer die Laufzeitregelung wird bewusst das juengste present()-Sample
	// beobachtet. `maxDurationUs` bleibt reine Diagnose, weil ein einzelner
	// Spike den praediktiven Vorlauf sonst ueber Minuten unnoetig aufblasen kann.
	exposureEngine.observeHeadPresentDurationUs(stats.lastDurationUs);
}

void reportHeadTimingIfDue() {
	if (!headTimingDiagnosticsState.serialReportEnabled ||
	    headTimingReportElapsed < kHeadTimingSerialReportIntervalMs) {
		return;
	}

	headTimingReportElapsed = 0;
	const dukatimer::NeoPixelHeadPresentStats& stats = neoPixelHead.presentStats();
	Serial.printf("[HEAD] present calls=%lu applied=%lu skipped=%lu last=%luus avg=%luus max=%luus bus=%lums lead=%lums\n",
	             static_cast<unsigned long>(stats.presentCalls),
	             static_cast<unsigned long>(stats.appliedFrames),
	             static_cast<unsigned long>(stats.skippedFrames),
	             static_cast<unsigned long>(stats.lastDurationUs),
	             static_cast<unsigned long>(stats.averageDurationUs),
	             static_cast<unsigned long>(stats.maxDurationUs),
	             static_cast<unsigned long>(exposureEngine.currentHeadBusLatencyMs()),
	             static_cast<unsigned long>(exposureEngine.currentPredictiveShutoffLeadMs()));
}

}  // namespace

FLASHMEM void setup() {
	// setup() initialisiert die Dienste in einer Reihenfolge, in der jede Schicht
	// nur von bereits gueltigen Vorbedingungen ausgeht: Inputs zuerst, danach
	// Persistenz und Licht, dann Mode-/Exposure-/Sensor-Dienste, zuletzt UI.
	Serial.begin(115200);
	Serial.printf("[%s] v%s (%s)\n", dukatimer::build::kFirmwareName, dukatimer::build::kFirmwareVersion,
	              dukatimer::build::kDisplayBackend);
	if (!startupPsramReady()) {
		haltStartupFailure("external-psram-missing");
	}
	if (SRC_GPR5 == kMainLoopWatchdogResetMagic) {
		Serial.println("[WDOG] application loop watchdog reset detected");
		localDoseControlWatchdogResetLatched = true;
		SRC_GPR5 = 0u;
	}
	pinMode(PIN_S3_GPIO0, OUTPUT);
	digitalWrite(PIN_S3_GPIO0, HIGH);

	touchSampler.begin();
	touchSampler.update();
	inputRouter.begin(millis());
	inputRouter.observeTouchState(touchSampler.state(), millis());
	setupInputs();
	updateInputs(millis());
	setupStorage();
	setupSystemSettingsPersistence();
	setupPaperSlotPersistence();
	measurementDomain.begin(millis());
	workflowServices.paperProfiles = &paperProfileQuery;
	workflowServices.paperProfileCommands = &paperProfileCommands;
	workflowServices.measurementQuery = &measurementDomain;
	workflowServices.measurementCommands = &measurementDomain;
	workflowServices.headTimingDiagnosticsQuery = &headTimingDiagnostics;
	workflowServices.headTimingDiagnosticsCommands = &headTimingDiagnostics;
	workflowServices.systemSettingsQuery = &systemSettingsQuery;
	workflowServices.systemSettingsCommands = &systemSettingsCommands;
	// Workflows erhalten ihre Query-Ports einmalig ueber den Coordinator,
	// damit weder globale Dienstzugriffe noch direkte Hardwarepfade noetig sind.
	modeCoordinator.bindServices(workflowServices);
	lightController.begin(focusSwitch.isActive(), saveSwitch.isActive(), roomSwitch.isActive());
	modeCoordinator.begin(millis(), kInitialMode);
	measurementDomain.setWirelessCaptureArmed(isMeasurementContextArmed(modeCoordinator.state()));
	sgModeWorkflow.setDoseControlForcedTime(localDoseControlWatchdogResetLatched);
	bwModeWorkflow.setDoseControlForcedTime(localDoseControlWatchdogResetLatched);
	exposureEngine.begin(millis());
	broadcastExposureStateToWorkflows(millis());
	sensorManager.begin(PIN_TSL_INT, millis());
	applySystemSettingsToRuntimeIfNeeded();
	exposureEngine.observeSensorStatus(sensorManager.state(), millis());
	updateLocalDoseControlPolicy();
	espServiceLink.begin(millis());
	measurementDomain.observeLocalSensor(sensorManager.state(), millis());
	measurementDomain.observeWirelessGateway(espServiceLink.state().wireless,
	                                       espServiceLink.state().health, millis());
	measurementDomain.tick(millis());
	neoPixelHead.begin();
	applyHeadOutput();
	neoPixelHead.resetPresentStats();
	const bool uiReady = ui.begin();
	Serial.printf("[UI] %s / %s\n", uiReady ? "ready" : "failed", dukatimer::build::kUiDesigner);
	if (!uiReady) {
		haltStartupFailure("lvgl-ui-not-ready");
	}
	const dukatimer::SystemSnapshot initialSnapshot = buildSystemSnapshot();
	ui.updateSnapshot(initialSnapshot);
	publishRemoteRenderIfNeeded(initialSnapshot, millis(), true);
	ui.service();
	beginMainLoopWatchdog(millis());
}

void loop() {
	// loop() ist bewusst als deterministische Laufzeitschleife organisiert.
	// Die Reihenfolge bildet den gewuenschten Datenfluss ab:
	// - Touch und lokale Inputs lesen
	// - Modus- und Remote-Ereignisse verarbeiten
	// - Sensorstatus aktualisieren
	// - Exposure/Head-Output fortschreiben
	// - erst danach Snapshot und UI aktualisieren
	const uint32_t nowMs = millis();
	feedMainLoopWatchdog(nowMs);
	const uint32_t lvglTickMs = lvglTickElapsed;
	if (lvglTickMs > 0) {
		ui.tick(lvglTickMs);
		lvglTickElapsed = 0;
	}

	touchSampler.update();
	inputRouter.observeTouchState(touchSampler.state(), nowMs);
	updateInputs(nowMs);
	modeCoordinator.tick(nowMs);
	applySystemSettingsToRuntimeIfNeeded();
	espServiceLink.setRealtimeHold(exposureEngine.state().phase == dukatimer::ExposurePhase::Exposing);
	espServiceLink.tick(nowMs);
	processRemoteCommandTracking(nowMs);
	telemeterRemoteCommandDiagnostics(nowMs);
	consumeEspInputEvents(nowMs);
	measurementDomain.setWirelessCaptureArmed(isMeasurementContextArmed(modeCoordinator.state()));
	applyEspServiceDataToSensorManager();
	sensorManager.tick(nowMs);
	exposureEngine.observeSensorStatus(sensorManager.state(), nowMs);
	updateLocalDoseControlPolicy();
	measurementDomain.observeLocalSensor(sensorManager.state(), nowMs);
	measurementDomain.observeWirelessGateway(espServiceLink.state().wireless,
	                                       espServiceLink.state().health, nowMs);
	measurementDomain.tick(nowMs);
	processSplitgradeExecutionCommands(nowMs);
	processBlackWhiteExecutionCommands(nowMs);
	exposureEngine.tick(nowMs);
	enforceExposureDeadlockRecovery(nowMs);
	broadcastExposureStateToWorkflows(nowMs);
	updateRemoteExecutionFeedback(nowMs);
	lightController.applyRoomOutput();
	applyHeadOutput();
	syncHeadTimingIntoExposureEngine();
	reportHeadTimingIfDue();
	inputRouter.observeRuntimeState(modeCoordinator.state(), exposureEngine.state(), nowMs);
	ui.service();

	// Globale Modal-Buttons werden unabhaengig vom konkreten LVGL-Screen als ein
	// einheitlicher lokaler Modal-Kanal eingespeist. So koennen Busy, kuenftiges
	// BW und spaetere Burn-Overlays denselben semantischen Vertrag teilen.
	{
		emitLocalModalButtonAction(ui.pollModalAction(), nowMs);
	}

	if (uiSnapshotElapsed >= SNAPSHOT_REFRESH_MS) {
		uiSnapshotElapsed = 0;
		const dukatimer::SystemSnapshot snapshot = buildSystemSnapshot();
		ui.updateSnapshot(snapshot);
		publishRemoteRenderIfNeeded(snapshot, nowMs);
	}
}
