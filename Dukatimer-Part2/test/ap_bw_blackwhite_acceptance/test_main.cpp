/*
 * AP-BW Acceptance Harness
 *
 * Prueft vier Kerneigenschaften des Schwarzweiss-Modus ohne Hardware:
 *
 *   1. E17-Check:       kBw darf niemals als Belichtungszeit-Startwert
 *                       interpretiert werden; der Workflow muss immer mit
 *                       kDefaultTargetValue (10.0 s) beginnen.
 *
 *   2. Max-Normalisierung: HeadSpectrumMapper liefert fuer BwGradeMix im
 *                          Identity-Fallback das korrekte normierte RGB.
 *
 *   3. Fixed-Grade Identity: BwWhite ergibt RGB(0, 255, 255).
 *
 *   4. Zustandsmaschine: IdleConfig -> Arming (Start-Input) ->
 *                        Exposing (ExposureState-Rueckkopplung).
 *
 * Struktur und Mocking-Ansatz folgen test/ap07_splitgrade_acceptance/test_main.cpp.
 * Fliesskomma-Assertions nutzen TEST_ASSERT_FLOAT_WITHIN (Unity-Stil-Makro,
 * das intern expectNear aufruft) um die geforderte Praezisionsschwelle
 * explizit zu machen.
 *
 * Kompilierung (nativ, kein Flashen noetig):
 *   g++ -std=c++17 \
 *       -I src/teensy -I test/support \
 *       test/ap_bw_blackwhite_acceptance/test_main.cpp \
 *       -o build/ap_bw_test && ./build/ap_bw_test
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>

// Arduino-Stub fuer native Builds: liefert millis() und erlaubt #include <Arduino.h>
// aus BlackWhiteWorkflow.cpp und HeadSpectrumMapper.cpp aufzuloesen.
#include "../support/Arduino.h"

// FLASHMEM ist auf Teensy ein Linker-Abschnitt-Attribut. Fuer native
// Testbuilds als leeres Makro eintragen, bevor die .cpp-Dateien eingezogen
// werden; so bleibt der Produktions-Code unberuehrt.
#ifndef FLASHMEM
#define FLASHMEM
#endif

// millis()-Stub; die BW-Workflow-Implementierung braucht keinen laufenden Takt,
// aber das Symbol muss linkbar sein.
unsigned long gFakeMillis = 0u;
unsigned long millis() {
	return gFakeMillis;
}

// Unity-Stil Fliesskomma-Assertion: TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual).
// Signatur bewusst wie Unity gewaehlt, damit der Testcode auf echte Unity-Makros
// umgestellt werden kann ohne inhaltliche Aenderungen.
// Die Expansion erfolgt erst an der Aufrufstelle im namespace-Kontext, wo
// expectNear sichtbar ist.
#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
	expectNear((actual), (expected), (delta), #actual " near " #expected)

// Unity-Stil Integer/Bool-Assertions fuer nicht-Float-Vergleiche.
#define TEST_ASSERT_EQUAL_UINT8(expected, actual) \
	expectEqualU8((actual), (expected), #actual " == " #expected)

#define TEST_ASSERT_TRUE(condition) \
	expectTrue((condition), #condition " should be true")

// Direkte Einbindung der Implementierungsdateien (Unity-Compilation-Pattern).
// So entfaellt ein separater Linkschritt und der Harness bleibt eigenstaendig.
#include "../../src/teensy/ExposureValueMath.cpp"
#include "../../src/teensy/HeadSpectrumMapper.cpp"
#include "../../src/teensy/BlackWhiteWorkflow.cpp"

namespace {

// ---------------------------------------------------------------------------
// using-Deklarationen
// ---------------------------------------------------------------------------

using dukatimer::BlackWhiteExecutionCommand;
using dukatimer::BlackWhiteExecutionCommandKind;
using dukatimer::BlackWhiteWorkflow;
using dukatimer::BwExecutionState;
using dukatimer::BwModeRuntimeState;
using dukatimer::BwPanel;
using dukatimer::ExposureControlMode;
using dukatimer::ExposurePhase;
using dukatimer::ExposureRuntimeState;
using dukatimer::HeadCalibrationProfile;
using dukatimer::HeadSpectrumCommand;
using dukatimer::HeadSpectrumMapper;
using dukatimer::HeadSpectrumSemantic;
using dukatimer::InputSemanticAction;
using dukatimer::InputSemanticActionKind;
using dukatimer::LightRgb;
using dukatimer::ModeId;
using dukatimer::ModeRuntimeState;
using dukatimer::ModeWorkflowServices;
using dukatimer::NormalizedInputEvent;
using dukatimer::NormalizedInputEventKind;
using dukatimer::NormalizedInputSource;
using dukatimer::PaperExposureProfile;
using dukatimer::PaperGradeMode;
using dukatimer::PaperProfileQueryPort;

constexpr float kFloatTolerance = 0.01f;

// ---------------------------------------------------------------------------
// Test-Hilfsfunktionen
// ---------------------------------------------------------------------------

[[noreturn]] void fail(const char* message) {
	std::fprintf(stderr, "FAIL: %s\n", message);
	std::exit(1);
}

void expectTrue(bool condition, const char* message) {
	if (!condition) {
		fail(message);
	}
}

void expectNear(float actual, float expected, float tolerance, const char* message) {
	if (std::fabs(actual - expected) > tolerance) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=%.4f expected=%.4f tolerance=%.4f)\n",
		             message,
		             actual,
		             expected,
		             tolerance);
		std::exit(1);
	}
}

void expectEqualU8(uint8_t actual, uint8_t expected, const char* message) {
	if (actual != expected) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=%u expected=%u)\n",
		             message,
		             static_cast<unsigned>(actual),
		             static_cast<unsigned>(expected));
		std::exit(1);
	}
}

// ---------------------------------------------------------------------------
// Mock: PaperProfileQueryPort
// ---------------------------------------------------------------------------

struct FakePaperProfiles final : PaperProfileQueryPort {
	PaperExposureProfile active = {};

	uint8_t slotCount() const override {
		return 1u;
	}

	uint8_t activeSlotIndex() const override {
		return 0u;
	}

	const PaperExposureProfile* activeProfile() const override {
		return &active;
	}

	const PaperExposureProfile* profileAt(uint8_t slotIndex) const override {
		return slotIndex == 0u ? &active : nullptr;
	}

	bool hasCalibratedActiveProfile() const override {
		return active.calibrated;
	}

	uint8_t storageErrorCode() const override {
		return 0u;
	}

	uint32_t storageErrorDetail() const override {
		return 0u;
	}
};

// ---------------------------------------------------------------------------
// Profil-Fabrikfunktionen
// ---------------------------------------------------------------------------

// Erstellt ein Multigrade-Papierprofil mit linearem Gradations-LUT.
// kBw wird bewusst als Parameter uebergeben, um E17-Verletzungen testbar
// zu machen: der Workflow darf kBw niemals als Belichtungszeit benutzen.
PaperExposureProfile makeMultigradeProfile(float kBw = 0.0f,
                                           float kSoft = 10.0f,
                                           float kHard = 10.0f) {
	PaperExposureProfile profile = {};
	profile.schemaVersion  = dukatimer::kPaperExposureProfileSchemaVersion;
	profile.calibrated     = true;
	profile.gradeMode      = PaperGradeMode::Multigrade;
	profile.useIsoMath     = false;
	profile.fixedGradeValue = 2.5f;
	profile.isoP           = 100.0f;
	profile.isoR           = 100.0f;
	// kBw ist ein dimensionsloser Transmissionsfaktor (10^-D_N). Er wird
	// hier explizit gesetzt, um E17 zu testen: egal welchen Wert kBw hat,
	// der Workflow muss kDefaultTargetValue (10.0 s) als Start verwenden.
	profile.kBw  = kBw;
	profile.kSoft = kSoft;
	profile.kHard = kHard;
	for (uint8_t i = 0u; i < dukatimer::kSplitgradeStepCount; ++i) {
		const float hardFraction = static_cast<float>(i) / 10.0f;
		profile.gradeKSoft[i] = kSoft * (1.0f - hardFraction);
		profile.gradeKHard[i] = kHard * hardFraction;
	}
	return profile;
}

PaperExposureProfile makeFixedGradeProfile(float fixedGrade = 3.0f) {
	PaperExposureProfile profile = makeMultigradeProfile();
	profile.gradeMode      = PaperGradeMode::FixedGrade;
	profile.fixedGradeValue = fixedGrade;
	return profile;
}

// ---------------------------------------------------------------------------
// Snapshot- und Dispatch-Helfer
// ---------------------------------------------------------------------------

// Liefert den aktuellen BW-Laufzeitzustand aus dem Workflow-Snapshot.
BwModeRuntimeState bwStateAt(BlackWhiteWorkflow& workflow, uint32_t nowMs) {
	ModeRuntimeState state;
	state.activeMode = ModeId::BlackWhite;
	workflow.populateRuntimeState(state, nowMs);
	return state.bw;
}

NormalizedInputEvent makeEvent(NormalizedInputEventKind eventKind,
                               uint32_t nowMs,
                               uint32_t sequence) {
	return dukatimer::makeNormalizedInputEvent(
		NormalizedInputSource::LocalEncoder1,
		eventKind,
		0,
		nowMs,
		sequence);
}

InputSemanticAction makeAction(InputSemanticActionKind kind,
                               uint32_t nowMs,
                               uint32_t sequence) {
	return dukatimer::makeInputSemanticAction(kind, 0, nowMs, sequence);
}

void dispatch(BlackWhiteWorkflow& workflow,
              InputSemanticActionKind kind,
              uint32_t nowMs,
              uint32_t sequence = 1u,
              NormalizedInputEventKind eventKind = NormalizedInputEventKind::Press) {
	workflow.onInputEvent(makeEvent(eventKind, nowMs, sequence),
	                      makeAction(kind, nowMs, sequence),
	                      nowMs);
}

BlackWhiteExecutionCommand consumeExpectedCommand(BlackWhiteWorkflow& workflow,
                                                 BlackWhiteExecutionCommandKind expectedKind,
                                                 const char* message) {
	BlackWhiteExecutionCommand command;
	expectTrue(workflow.consumeExecutionCommand(command), message);
	expectTrue(command.kind == expectedKind, message);
	return command;
}

ExposureRuntimeState makeIdleExposureState() {
	return dukatimer::makeIdleExposureRuntimeState();
}

ExposureRuntimeState makeExposureState(ExposurePhase phase) {
	ExposureRuntimeState state;
	state.phase = phase;
	return state;
}

// ---------------------------------------------------------------------------
// TEST 1: E17-Check — kBw darf nicht als Zielwert uebernommen werden
//
// Motivation: In fruheren Revisionen wurde profile.kBw faelschlicherweise als
// Belichtungszeit-Defaultwert interpretiert. E17 legt fest, dass kBw ein
// rein dimensionsloser Transmissionsfaktor ist (10^-D_N). Der BW-Workflow muss
// immer mit kDefaultTargetValue (10.0 s) starten, unabhaengig von kBw.
// ---------------------------------------------------------------------------

void runE17KBwNotUsedAsTargetCase() {
	// Extremwert: kBw = 0.0071f entspricht D_N = 2.15 (sehr starkes Papier).
	// Wenn dieser Wert faelschlicherweise als Sekunden interpretiert wuerde,
	// waere targetValue ~0.007 s statt 10.0 s.
	constexpr float kExtremeBwFactor = 0.0071f;

	FakePaperProfiles profiles;
	profiles.active = makeMultigradeProfile(kExtremeBwFactor);

	ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	BlackWhiteWorkflow workflow;
	workflow.bindServices(services);

	constexpr uint32_t nowMs = 1000u;
	workflow.onEnter(nowMs);

	const BwModeRuntimeState state = bwStateAt(workflow, nowMs);

	// E17-Kernbedingung: targetValue muss der Laufzeit-Default sein, nicht kBw.
	TEST_ASSERT_FLOAT_WITHIN(kFloatTolerance, 10.0f, state.targetValue);

	// Zusaetliche Plausibilitaetspruefung: 0.0071f waere als Zeit weit ausserhalb
	// des erwarteten Bereichs und darf daher nicht sichtbar sein.
	expectTrue(state.targetValue > 1.0f,
	           "E17: targetValue muss groesser als 1s sein (kBw-Wert waere << 1s)");

	// Multigrade-Profil: Gradation muss editierbar sein, Weisslicht aus.
	expectTrue(state.gradeEditable, "E17: Multigrade-Profil muss gradeEditable = true liefern");
	expectTrue(!state.whiteLight,   "E17: Multigrade-Profil darf kein Weisslicht aktivieren");
}

// ---------------------------------------------------------------------------
// TEST 2: Max-Normalisierung fuer BwGradeMix (Identity-Fallback)
//
// Motivation: Bei unkalibriertem Profil (calibrated=false) muss der
// HeadSpectrumMapper den dominanten Kanal auf 255 normieren, statt beide
// absolut zu skalieren. So wird kein unnnoetiger Helligkeitsverlust erzeugt.
//
// Beispiel: soft=0.6, hard=0.4 -> maxMix=0.6 -> scale=255/0.6=425
//   green = round(0.6 * 425) = 255
//   blue  = round(0.4 * 425) = round(170.0) = 170
// ---------------------------------------------------------------------------

void runMaxNormalisierungBwGradeMixCase() {
	HeadSpectrumCommand cmd;
	cmd.semantic       = HeadSpectrumSemantic::BwGradeMix;
	cmd.channels.soft  = 0.6f;
	cmd.channels.hard  = 0.4f;

	// Unkalibriertes Profil: Identity-Fallback greift.
	HeadCalibrationProfile profile;
	// calibrated = false ist der Default-Konstruktor-Wert.

	const LightRgb result = HeadSpectrumMapper::map(cmd, profile);

	// Rot-Kanal muss 0 sein (BwGradeMix hat keine Rot-Komponente).
	TEST_ASSERT_EQUAL_UINT8(0u, result.red);

	// Gruen-Kanal: dominanter Kanal (soft=0.6), muss auf 255 normiert sein.
	TEST_ASSERT_EQUAL_UINT8(255u, result.green);

	// Blau-Kanal: proportional skaliert -> 0.4 / 0.6 * 255 = 170.0 -> 170.
	TEST_ASSERT_EQUAL_UINT8(170u, result.blue);
}

// ---------------------------------------------------------------------------
// TEST 3: Fixed-Grade Identity fuer BwWhite (Identity-Fallback)
//
// Motivation: FixedGrade-Papiere nutzen Weisslicht (beide Kanaele gleichzeitig
// voll). Im unkalibriertem Zustand muss der Mapper RGB(0, 255, 255) liefern
// (Gruen + Blau voll, kein Rot).
// ---------------------------------------------------------------------------

void runFixedGradeIdentityBwWhiteCase() {
	HeadSpectrumCommand cmd;
	cmd.semantic       = HeadSpectrumSemantic::BwWhite;
	cmd.channels.soft  = 1.0f;
	cmd.channels.hard  = 1.0f;

	HeadCalibrationProfile profile;
	// calibrated = false ist der Default.

	const LightRgb result = HeadSpectrumMapper::map(cmd, profile);

	// BwWhite: Rot=0, Gruen=255 (Soft), Blau=255 (Hard).
	TEST_ASSERT_EQUAL_UINT8(0u,   result.red);
	TEST_ASSERT_EQUAL_UINT8(255u, result.green);
	TEST_ASSERT_EQUAL_UINT8(255u, result.blue);
}

// ---------------------------------------------------------------------------
// TEST 4: Workflow-Zustandsmaschine IdleConfig -> Arming -> Exposing
//
// Motivation: Der vollstaendige Start-Pfad laeuft ueber zwei Stufen:
//   (a) Start-Input: IdleConfig -> Arming (Kommando an Wiring-Layer)
//   (b) ExposureEngine-Rueckkopplung: Arming -> Exposing
// Beide Uebergaenge muessen korrekt ausgefuehrt werden, damit der Wiring-Layer
// weiss, wann er die Belichtung tatsaechlich startet und wann sie laeuft.
// ---------------------------------------------------------------------------

void runWorkflowStateMachineCase() {
	FakePaperProfiles profiles;
	profiles.active = makeMultigradeProfile();

	ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	BlackWhiteWorkflow workflow;
	workflow.bindServices(services);

	uint32_t nowMs   = 2000u;
	uint32_t sequence = 1u;
	workflow.onEnter(nowMs);

	// --- Schritt A: Ausgangszustand ---
	BwModeRuntimeState state = bwStateAt(workflow, nowMs);
	TEST_ASSERT_TRUE(state.executionState == BwExecutionState::IdleConfig);
	TEST_ASSERT_TRUE(state.panel == BwPanel::Target);

	// --- Schritt B: Start-Input -> Arming ---
	// Der Workflow liefert ein Start-Kommando an den Wiring-Layer und
	// wechselt intern auf Arming. Die ExposureEngine hat noch nicht
	// zugestimmt; der physische Belichtungsstart folgt erst nach
	// observeExposureState.
	dispatch(workflow, InputSemanticActionKind::Start, ++nowMs, ++sequence);

	state = bwStateAt(workflow, nowMs);
	TEST_ASSERT_TRUE(state.executionState == BwExecutionState::Arming);

	// Das Start-Kommando muss konsumierbar sein.
	consumeExpectedCommand(workflow,
	                       BlackWhiteExecutionCommandKind::Start,
	                       "Workflow muss nach Start-Input ein Start-Kommando queuen");

	// --- Schritt C: ExposureEngine zeigt aktive Belichtung -> Exposing ---
	// In main.cpp wuerde processBlackWhiteExecutionCommands() das Kommando
	// verarbeiten und ExposureEngine::start*() aufrufen. Sobald die Engine
	// ExposurePhase::Exposing zurueckmeldet, muss der Workflow auf Exposing
	// wechseln.
	workflow.observeExposureState(makeExposureState(ExposurePhase::Exposing), ++nowMs);

	state = bwStateAt(workflow, nowMs);
	TEST_ASSERT_TRUE(state.executionState == BwExecutionState::Exposing);

	// --- Schritt D: Belichtung abgeschlossen -> Completed -> IdleConfig ---
	// Nach Done-Meldung der Engine muss der Workflow nach Completed wechseln,
	// und nach einer weiteren Idle-Meldung zurueck auf IdleConfig.
	workflow.observeExposureState(makeExposureState(ExposurePhase::Done), ++nowMs);
	state = bwStateAt(workflow, nowMs);
	TEST_ASSERT_TRUE(state.executionState == BwExecutionState::Completed);

	workflow.observeExposureState(makeIdleExposureState(), ++nowMs);
	state = bwStateAt(workflow, nowMs);
	TEST_ASSERT_TRUE(state.executionState == BwExecutionState::IdleConfig);
}

}  // namespace

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main() {
	runE17KBwNotUsedAsTargetCase();
	runMaxNormalisierungBwGradeMixCase();
	runFixedGradeIdentityBwWhiteCase();
	runWorkflowStateMachineCase();
	std::puts("AP-BW acceptance harness passed.");
	return 0;
}
