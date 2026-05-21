#include "PaperWorkflow.h"

#include <cmath>

namespace dukatimer {

namespace {

}  // namespace

ModeId PaperWorkflow::modeId() const {
	return ModeId::Paper;
}

const char* PaperWorkflow::shortName() const {
	return "PAPER";
}

void PaperWorkflow::onEnter(uint32_t nowMs) {
	active_ = true;
	editingActive_ = false;
	parametersDirty_ = false;
	persistFailed_ = false;
	panel_ = PaperWorkspacePanel::Select;
	selectedItem_ = PaperCalibrationItem::GradeMode;
	enteredAtMs_ = nowMs;
	panelChangedAtMs_ = nowMs;
	selectedItemChangedAtMs_ = nowMs;
	// Beim Eintritt folgt die Auswahl zunaechst dem bereits aktiven Slot. So
	// startet PAPER immer im produktiv wirksamen Kontext statt in einem stale
	// Browse-Zustand aus einem frueheren Besuch.
	// Stufeneingaben werden auf sichere Defaults zurueckgesetzt, damit kein
	// Zufallszustand aus einem frueheren Besuch weiterbesteht.
	stepWhite_ = kStepWhiteDefault;
	stepBlack_ = kStepBlackDefault;
	syncSelectedSlotFromActive(nowMs, true);
	refreshSelectedProfile(true);
}

void PaperWorkflow::onExit(uint32_t nowMs) {
	(void)nowMs;
	active_ = false;
	editingActive_ = false;
	parametersDirty_ = false;
	persistFailed_ = false;
	panel_ = PaperWorkspacePanel::Inactive;
	selectedSlot_ = 0u;
	selectedItem_ = PaperCalibrationItem::GradeMode;
	enteredAtMs_ = 0u;
	panelChangedAtMs_ = 0u;
	selectedSlotChangedAtMs_ = 0u;
	selectedItemChangedAtMs_ = 0u;
	storedProfile_ = {};
	stagedProfile_ = {};
	editBackupProfile_ = {};
}

void PaperWorkflow::onTick(uint32_t nowMs) {
	if (!active_) {
		return;
	}

	// Externe Slot-Bank-Aenderungen duerfen die lokale Auswahl nicht ausserhalb
	// des gueltigen Bereichs stehen lassen. Mehr als ein Clamp ist hier bewusst
	// noch nicht erlaubt; produktive Datenmigration bleibt weiter Sache der
	// Storage-/Codec-Schicht.
	const uint8_t slotCount = currentSlotCount();
	if (slotCount == 0u) {
		selectedSlot_ = 0u;
		storedProfile_ = {};
		stagedProfile_ = {};
		parametersDirty_ = false;
		editingActive_ = false;
		return;
	}

	if (selectedSlot_ >= slotCount) {
		selectedSlot_ = static_cast<uint8_t>(slotCount - 1u);
		selectedSlotChangedAtMs_ = nowMs;
	}

	if (panel_ == PaperWorkspacePanel::Calibrate && !editingActive_ && !parametersDirty_) {
		refreshSelectedProfile(true);
	}
	}

void PaperWorkflow::onInputEvent(const NormalizedInputEvent& event,
	                            const InputSemanticAction& action,
	                            uint32_t nowMs) {
	if (!active_ || !event.isMeaningful() || !action.isMeaningful()) {
		return;
	}

	if (panel_ == PaperWorkspacePanel::Calibrate) {
		switch (action.kind) {
			case InputSemanticActionKind::ContextPrevious:
				if (!editingActive_) {
					cycleSelectedItem(-1, nowMs);
				}
				break;

			case InputSemanticActionKind::ContextNext:
				if (!editingActive_) {
					cycleSelectedItem(1, nowMs);
				}
				break;

			case InputSemanticActionKind::AdjustPrimaryDecrease:
				if (editingActive_) {
					adjustSelectedItem(-1, false, nowMs);
				}
				break;

			case InputSemanticActionKind::AdjustPrimaryIncrease:
				if (editingActive_) {
					adjustSelectedItem(1, false, nowMs);
				}
				break;

			case InputSemanticActionKind::AdjustSecondaryDecrease:
				if (editingActive_) {
					adjustSelectedItem(-1, true, nowMs);
				}
				break;

			case InputSemanticActionKind::AdjustSecondaryIncrease:
				if (editingActive_) {
					adjustSelectedItem(1, true, nowMs);
				}
				break;

			case InputSemanticActionKind::Confirm:
				handleCalibrationConfirm(nowMs);
				break;

			case InputSemanticActionKind::Undo:
				handleCalibrationUndo(nowMs);
				break;

			case InputSemanticActionKind::Start:
			case InputSemanticActionKind::Measure:
			case InputSemanticActionKind::Pause:
			case InputSemanticActionKind::Resume:
			case InputSemanticActionKind::None:
				break;
		}
		return;
	}

	switch (action.kind) {
		case InputSemanticActionKind::ContextPrevious:
			enterCalibrationPanel(nowMs);
			break;

		case InputSemanticActionKind::ContextNext:
			enterCalibrationPanel(nowMs);
			break;

		case InputSemanticActionKind::AdjustPrimaryDecrease:
			// Enc1 bleibt die feine lokale Wertachse und browse't deshalb einzeln
			// durch die Papierbank.
			cycleSelectedSlot(-1, nowMs);
			break;

		case InputSemanticActionKind::AdjustPrimaryIncrease:
			cycleSelectedSlot(1, nowMs);
			break;

		case InputSemanticActionKind::AdjustSecondaryDecrease:
			// Enc2 bleibt die zweite lokale Wertachse, wird hier aber bewusst als
			// grober Slot-Sprung genutzt, damit 20 Plaetze ohne Touch schnell
			// erreichbar bleiben.
			cycleSelectedSlot(-kSecondarySlotStep, nowMs);
			break;

		case InputSemanticActionKind::AdjustSecondaryIncrease:
			cycleSelectedSlot(kSecondarySlotStep, nowMs);
			break;

		case InputSemanticActionKind::Confirm:
			activateSelectedSlot();
			break;

		case InputSemanticActionKind::Undo:
			syncSelectedSlotFromActive(nowMs, true);
			refreshSelectedProfile(true);
			break;

		case InputSemanticActionKind::Start:
		case InputSemanticActionKind::Measure:
		case InputSemanticActionKind::Pause:
		case InputSemanticActionKind::Resume:
		case InputSemanticActionKind::None:
			break;
	}
}

void PaperWorkflow::populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const {
	if (!active_) {
		return;
	}

	state.paper.panel = panel_;
	state.paper.selectedSlot = selectedSlot_;
	state.paper.selectionDirty = selectedSlot_ != currentActiveSlot();
	state.paper.selectedItem = selectedItem_;
	state.paper.itemCount = static_cast<uint8_t>(PaperCalibrationItem::Count);
	state.paper.editingActive = editingActive_;
	state.paper.parametersDirty = parametersDirty_;
	state.paper.persistFailed = persistFailed_;
	state.paper.panelAgeMs = nowMs - panelChangedAtMs_;
	state.paper.selectedSlotAgeMs = nowMs - selectedSlotChangedAtMs_;
	state.paper.selectedItemAgeMs = nowMs - selectedItemChangedAtMs_;
	state.paper.stagedProfile.available = currentSlotCount() > 0u;
	state.paper.stagedProfile.calibrated = stagedProfile_.calibrated;
	state.paper.stagedProfile.gradeMode = stagedProfile_.gradeMode;
	state.paper.stagedProfile.useIsoMath = stagedProfile_.useIsoMath;
	state.paper.stagedProfile.fixedGradeValue = stagedProfile_.fixedGradeValue;
	state.paper.stagedProfile.isoP = stagedProfile_.isoP;
	state.paper.stagedProfile.isoR = stagedProfile_.isoR;
	state.paper.stagedProfile.kBw = stagedProfile_.kBw;
	state.paper.stagedProfile.kSoft = stagedProfile_.kSoft;
	state.paper.stagedProfile.kHard = stagedProfile_.kHard;
	state.paper.stepWhite = stepWhite_;
	state.paper.stepBlack = stepBlack_;

	const ModeWorkflowServices* boundServices = services();
	if (boundServices != nullptr && boundServices->paperProfiles != nullptr) {
		state.paper.storageErrorCode = boundServices->paperProfiles->storageErrorCode();
		state.paper.storageErrorDetail = boundServices->paperProfiles->storageErrorDetail();
	}
}

uint8_t PaperWorkflow::currentSlotCount() const {
	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->paperProfiles == nullptr) {
		return 0u;
	}

	return boundServices->paperProfiles->slotCount();
}

uint8_t PaperWorkflow::currentActiveSlot() const {
	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->paperProfiles == nullptr) {
		return 0u;
	}

	return boundServices->paperProfiles->activeSlotIndex();
}

const PaperExposureProfile* PaperWorkflow::currentSelectedProfile() const {
	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->paperProfiles == nullptr) {
		return nullptr;
	}

	return boundServices->paperProfiles->profileAt(selectedSlot_);
}

void PaperWorkflow::syncSelectedSlotFromActive(uint32_t nowMs, bool forceAgeReset) {
	const uint8_t nextSlot = currentActiveSlot();
	if (forceAgeReset || selectedSlot_ != nextSlot) {
		selectedSlot_ = nextSlot;
		selectedSlotChangedAtMs_ = nowMs;
	}
}

void PaperWorkflow::enterCalibrationPanel(uint32_t nowMs) {
	if (!active_ || panel_ == PaperWorkspacePanel::Calibrate) {
		return;
	}

	// CAL darf den aktuell gewaehlten Slot bearbeiten, ohne ihn vorher heimlich
	// zum aktiven Druckslot zu machen. Deshalb wird hier nur der lokale Editier-
	// kontext auf genau diesen Slot geladen.
	panel_ = PaperWorkspacePanel::Calibrate;
	panelChangedAtMs_ = nowMs;
	selectedItem_ = PaperCalibrationItem::GradeMode;
	selectedItemChangedAtMs_ = nowMs;
	editingActive_ = false;
	persistFailed_ = false;
	refreshSelectedProfile(true);
}

void PaperWorkflow::leaveCalibrationPanel(uint32_t nowMs) {
	if (!active_ || panel_ != PaperWorkspacePanel::Calibrate || editingActive_ || parametersDirty_) {
		return;
	}

	panel_ = PaperWorkspacePanel::Select;
	panelChangedAtMs_ = nowMs;
	selectedItem_ = PaperCalibrationItem::GradeMode;
	selectedItemChangedAtMs_ = nowMs;
	persistFailed_ = false;
}

void PaperWorkflow::refreshSelectedProfile(bool overwriteStaged) {
	const PaperExposureProfile* selectedProfile = currentSelectedProfile();
	storedProfile_ = selectedProfile != nullptr ? *selectedProfile : PaperExposureProfile{};
	if (overwriteStaged) {
		stagedProfile_ = storedProfile_;
		parametersDirty_ = false;
		// Inverse Rekonstruktion: Stufen N (Weissunkt) und M (Schwarzpunkt) aus
		// den persistierten Werten kBw und isoR zurueckrechnen, damit die
		// STEP WHITE / STEP BLACK-Eingabe nach dem Laden des Profils sofort
		// einen konsistenten Ausgangswert zeigt.
		//
		// Mathematik (invers zu recalculateFromSteps):
		//   kBw = 10^(-D(N))  =>  D(N) = -log10(kBw)
		//   D(N) = 0.05 + (N-1)*0.15  =>  N = 1 + (D(N) - 0.05) / 0.15
		//   isoR = (N - M) * 15  =>  M = N - round(isoR / 15)
		//
		// Guards: kBw muss im offenen Intervall (0, 1) liegen und isoR > 0 sein;
		// bei ungueltigem Profil werden die sicheren Defaults verwendet.
		if (stagedProfile_.kBw > 0.0f && stagedProfile_.kBw < 1.0f &&
		    stagedProfile_.isoR > 0.0f) {
			const float dN   = -log10f(stagedProfile_.kBw);
			const int   n    = static_cast<int>(roundf(1.0f + (dN - 0.05f) / 0.15f));
			const int   span = static_cast<int>(roundf(stagedProfile_.isoR / 15.0f));
			const int   m    = n - span;
			const int   nC   = (n < static_cast<int>(kStepMin)) ? static_cast<int>(kStepMin) :
			                   (n > static_cast<int>(kStepMax)) ? static_cast<int>(kStepMax) : n;
			const int   mC   = (m < static_cast<int>(kStepMin)) ? static_cast<int>(kStepMin) :
			                   (m >= nC)                         ? nC - 1 : m;
			stepWhite_ = static_cast<uint8_t>(nC);
			stepBlack_ = static_cast<uint8_t>((mC < static_cast<int>(kStepMin)) ?
			                                  static_cast<int>(kStepMin) : mC);
		} else {
			stepWhite_ = kStepWhiteDefault;
			stepBlack_ = kStepBlackDefault;
		}
	}
}

void PaperWorkflow::cycleSelectedSlot(int8_t direction, uint32_t nowMs) {
	const uint8_t slotCount = currentSlotCount();
	if (slotCount == 0u || direction == 0) {
		return;
	}

	int nextSlot = static_cast<int>(selectedSlot_) + static_cast<int>(direction);
	while (nextSlot < 0) {
		nextSlot += static_cast<int>(slotCount);
	}
	while (nextSlot >= static_cast<int>(slotCount)) {
		nextSlot -= static_cast<int>(slotCount);
	}

	selectedSlot_ = static_cast<uint8_t>(nextSlot);
	selectedSlotChangedAtMs_ = nowMs;
	refreshSelectedProfile(true);
}

void PaperWorkflow::cycleSelectedItem(int8_t direction, uint32_t nowMs) {
	selectedItem_ = wrapSelectedItem(static_cast<int>(selectedItem_) + static_cast<int>(direction));
	selectedItemChangedAtMs_ = nowMs;
	persistFailed_ = false;
}

void PaperWorkflow::activateSelectedSlot() {
	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->paperProfileCommands == nullptr) {
		return;
	}

	// Der Workflow bestaetigt nur die Auswahl. Die Persistenz- und Fehlerpolitik
	// fuer die Slot-Bank bleibt zentral im Command-Adapter gebuendelt.
	(void)boundServices->paperProfileCommands->selectActiveSlot(selectedSlot_);
}

void PaperWorkflow::adjustSelectedItem(int8_t direction, bool coarseStep, uint32_t nowMs) {
	(void)nowMs;
	if (direction == 0) {
		return;
	}

	persistFailed_ = false;
	switch (selectedItem_) {
		case PaperCalibrationItem::GradeMode:
			stagedProfile_.gradeMode = isFixedGradeMode(stagedProfile_.gradeMode)
				? PaperGradeMode::Multigrade
				: PaperGradeMode::FixedGrade;
			break;

		case PaperCalibrationItem::FixedGradeValue: {
			const float step = coarseStep ? kFixedGradeCoarseStep : kFixedGradeStep;
			stagedProfile_.fixedGradeValue =
				clampFixedGrade(stagedProfile_.fixedGradeValue + (static_cast<float>(direction) * step));
			break;
		}

		case PaperCalibrationItem::IsoMath:
			stagedProfile_.useIsoMath = !stagedProfile_.useIsoMath;
			break;

		case PaperCalibrationItem::IsoP: {
			const float step = coarseStep ? kIsoCoarseStep : kIsoStep;
			stagedProfile_.isoP = clampIso(stagedProfile_.isoP + (static_cast<float>(direction) * step));
			break;
		}

		case PaperCalibrationItem::IsoR: {
			const float step = coarseStep ? kIsoCoarseStep : kIsoStep;
			stagedProfile_.isoR = clampIso(stagedProfile_.isoR + (static_cast<float>(direction) * step));
			break;
		}

		case PaperCalibrationItem::KBw: {
			const float step = coarseStep ? kKFactorCoarseStep : kKFactorStep;
			stagedProfile_.kBw = clampKFactor(stagedProfile_.kBw + (static_cast<float>(direction) * step));
			break;
		}

		case PaperCalibrationItem::KSoft: {
			const float step = coarseStep ? kKFactorCoarseStep : kKFactorStep;
			stagedProfile_.kSoft =
				clampKFactor(stagedProfile_.kSoft + (static_cast<float>(direction) * step));
			break;
		}

		case PaperCalibrationItem::KHard: {
			const float step = coarseStep ? kKFactorCoarseStep : kKFactorStep;
			stagedProfile_.kHard =
				clampKFactor(stagedProfile_.kHard + (static_cast<float>(direction) * step));
			break;
		}

		case PaperCalibrationItem::Calibrated:
			stagedProfile_.calibrated = !stagedProfile_.calibrated;
			break;

		case PaperCalibrationItem::StepWhite: {
			// Weissunkt N; Untergrenze: stepBlack_ + 1 (Kontrast-Invariante)
			const int next = static_cast<int>(stepWhite_) + static_cast<int>(direction);
			const uint8_t lo = static_cast<uint8_t>(stepBlack_ + 1u);
			if (next >= static_cast<int>(lo) && next <= static_cast<int>(kStepMax)) {
				stepWhite_ = static_cast<uint8_t>(next);
				recalculateFromSteps();
			}
			break;
		}

		case PaperCalibrationItem::StepBlack: {
			// Schwarzpunkt M; Obergrenze: stepWhite_ - 1 (Kontrast-Invariante)
			const int next = static_cast<int>(stepBlack_) + static_cast<int>(direction);
			const uint8_t hi = static_cast<uint8_t>(stepWhite_ - 1u);
			if (next >= static_cast<int>(kStepMin) && next <= static_cast<int>(hi)) {
				stepBlack_ = static_cast<uint8_t>(next);
				recalculateFromSteps();
			}
			break;
		}

		case PaperCalibrationItem::Apply:
		case PaperCalibrationItem::Discard:
		case PaperCalibrationItem::Count:
			break;
	}

	updateDirtyState();
}

void PaperWorkflow::handleCalibrationConfirm(uint32_t nowMs) {
	if (editingActive_) {
		editingActive_ = false;
		updateDirtyState();
		persistFailed_ = false;
		return;
	}

	if (isActionItem(selectedItem_)) {
		executeCalibrationAction(selectedItem_, nowMs);
		return;
	}

	editBackupProfile_ = stagedProfile_;
	editingActive_ = true;
	persistFailed_ = false;
}

void PaperWorkflow::handleCalibrationUndo(uint32_t nowMs) {
	if (editingActive_) {
		stagedProfile_ = editBackupProfile_;
		editingActive_ = false;
		updateDirtyState();
		persistFailed_ = false;
		return;
	}

	if (parametersDirty_) {
		stagedProfile_ = storedProfile_;
		parametersDirty_ = false;
		persistFailed_ = false;
		return;
	}

	leaveCalibrationPanel(nowMs);
}

void PaperWorkflow::executeCalibrationAction(PaperCalibrationItem item, uint32_t nowMs) {
	(void)nowMs;
	persistFailed_ = false;

	const ModeWorkflowServices* boundServices = services();
	switch (item) {
		case PaperCalibrationItem::Apply:
			if (boundServices == nullptr || boundServices->paperProfileCommands == nullptr) {
				persistFailed_ = true;
				return;
			}

			// CAL schreibt bewusst auf den gerade gewaehlten Slot statt still auf das
			// aktive Druckpapier. So kann ein Profil vorbereitet werden, ohne den
			// laufenden PRINT-Kontext zu ueberschreiben.
			if (!boundServices->paperProfileCommands->saveProfileAt(selectedSlot_, stagedProfile_)) {
				persistFailed_ = true;
				return;
			}

			refreshSelectedProfile(true);
			editingActive_ = false;
			persistFailed_ = false;
			break;

		case PaperCalibrationItem::Discard:
			stagedProfile_ = storedProfile_;
			parametersDirty_ = false;
			editingActive_ = false;
			persistFailed_ = false;
			break;

		case PaperCalibrationItem::GradeMode:
		case PaperCalibrationItem::FixedGradeValue:
		case PaperCalibrationItem::IsoMath:
		case PaperCalibrationItem::IsoP:
		case PaperCalibrationItem::IsoR:
		case PaperCalibrationItem::KBw:
		case PaperCalibrationItem::KSoft:
		case PaperCalibrationItem::KHard:
		case PaperCalibrationItem::Calibrated:
		case PaperCalibrationItem::StepWhite:
		case PaperCalibrationItem::StepBlack:
		case PaperCalibrationItem::Count:
			break;
	}
}

void PaperWorkflow::updateDirtyState() {
	parametersDirty_ = stagedProfile_ != storedProfile_;
}

void PaperWorkflow::recalculateFromSteps() {
	// Visuelle Schwellenwert-Methode (Methode 1).
	// Eingaben:  N = stepWhite_ (Weissunkt, hoehere Stufennummer)
	//            M = stepBlack_ (Schwarzpunkt, niedrigere Stufennummer)
	// Bedingung: N > M ist durch die Eingabegrenzen in adjustSelectedItem() garantiert.
	//
	// Keilformel: D_Keil(s) = 0.05 + (s - 1) * 0.15
	// ISO-R       = (N - M) * 15
	// k_Bw        = 10^(-D_Keil(N))         (dimensionsloser Schwellenfaktor)
	// ISO-P       = H_ref * 10^(-D_Keil((N+M)/2))
	//             mit H_ref = 150.0 lx * 10.0 s = 1500.0 lx*s

	// Sicherheitsguard: keine Berechnung bei ungueltiger Stufenkonstellation
	if (stepWhite_ <= stepBlack_ || stepWhite_ < kStepMin || stepWhite_ > kStepMax ||
	    stepBlack_ < kStepMin || stepBlack_ > kStepMax) {
		return;
	}

	static constexpr float kHRef     = 1500.0f;  // lx*s
	static constexpr float kDensBase = 0.05f;
	static constexpr float kDensStep = 0.15f;

	const float dWhite = kDensBase + (static_cast<float>(stepWhite_) - 1.0f) * kDensStep;
	const float dMid   = kDensBase + ((static_cast<float>(stepWhite_) +
	                                   static_cast<float>(stepBlack_)) * 0.5f - 1.0f) * kDensStep;

	stagedProfile_.isoR       = static_cast<float>((stepWhite_ - stepBlack_) * 15u);
	stagedProfile_.kBw        = powf(10.0f, -dWhite);
	stagedProfile_.isoP       = kHRef * powf(10.0f, -dMid);
	// Ein erfolgreicher Methode-1-Durchlauf kennzeichnet das Profil als kalibriert,
	// damit Apply und die nachfolgende Dosisrechnung das Profil als gueltig ansehen.
	stagedProfile_.calibrated = true;
}

bool PaperWorkflow::isActionItem(PaperCalibrationItem item) {
	return item == PaperCalibrationItem::Apply || item == PaperCalibrationItem::Discard;
}


PaperCalibrationItem PaperWorkflow::wrapSelectedItem(int nextIndex) {
	const int itemCount = static_cast<int>(PaperCalibrationItem::Count);
	if (nextIndex < 0) {
		nextIndex = itemCount - 1;
	} else if (nextIndex >= itemCount) {
		nextIndex = 0;
	}

	return static_cast<PaperCalibrationItem>(nextIndex);
}

float PaperWorkflow::clampFixedGrade(float value) {
	if (value < kMinimumFixedGrade) {
		return kMinimumFixedGrade;
	}
	if (value > kMaximumFixedGrade) {
		return kMaximumFixedGrade;
	}
	return value;
}

float PaperWorkflow::clampIso(float value) {
	if (value < kMinimumIso) {
		return kMinimumIso;
	}
	if (value > kMaximumIso) {
		return kMaximumIso;
	}
	return value;
}

float PaperWorkflow::clampKFactor(float value) {
	if (value < kMinimumKFactor) {
		return kMinimumKFactor;
	}
	if (value > kMaximumKFactor) {
		return kMaximumKFactor;
	}
	return value;
}

}  // namespace dukatimer