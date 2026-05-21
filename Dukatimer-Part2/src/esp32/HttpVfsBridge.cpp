/*
 * HttpVfsBridge
 *
 * Diese Datei koppelt den ESP32-seitigen WLAN/HTTP-Zugang an die serielle VFS-
 * Uploadmaschine. Sie bleibt bewusst klein und synchron im Requestkontext, damit
 * Diagnose und Fehlerabbildung fuer den Bring-up-Pfad leicht nachvollziehbar bleiben.
 */

#include "HttpVfsBridge.h"

#include <esp_heap_caps.h>

#include <stdlib.h>
#include <string.h>

#include <WiFi.h>

#include <DukatimerProtocol.h>
#include <ProductDataStoragePaths.h>

#include "../teensy/PaperSlotPersistenceCodec.h"

#include "ServiceNetworkConfig.h"
#include "TeensyLinkService.h"

namespace dukatimer {

namespace {

constexpr int kHttpStatusInsufficientStorage = 507;
constexpr int kHttpStatusUnprocessableContent = 422;
constexpr uint32_t kPaperSlotRecoveryRefreshIntervalMs = 5000u;

struct PaperSlotBlobInspection {
	bool valid = false;
	PaperSlotBlobParseError parseError = PaperSlotBlobParseError::None;
	PaperSlotBlobHeader header = {};
	uint8_t activeSlot = 0u;
	uint8_t slotCount = 0u;
	bool activeSlotCalibrated = false;
	PaperGradeMode activeGradeMode = PaperGradeMode::Multigrade;
	bool activeSlotUseIsoMath = false;
	float activeSlotFixedGradeValue = 0.0f;
	char activeSlotName[kPaperProfileNameCapacity] = {};
};

struct PaperSlotPathAssessment {
	bool fetchSucceeded = false;
	int httpStatusCode = 500;
	dukatimer::protocol::VfsStatusCode vfsStatus = dukatimer::protocol::VfsStatusCode::None;
	char path[dukatimer::protocol::kVfsMaxPathLength + 1] = {};
	char statusText[48] = "not_ready";
	char detail[96] = "request not handled";
	uint32_t stagedBytes = 0u;
	bool stagingUsesPsram = false;
	PaperSlotBlobInspection inspection = {};
};

struct PaperSlotRecoveryDecision {
	bool decisionStable = false;
	bool canRestoreBackup = false;
	bool restoreWouldChangeActive = false;
	bool samePayloadSignature = false;
	bool manualRecoverUploadSuggested = false;
	uint16_t flags = dukatimer::protocol::kPaperSlotRecoveryNone;
	dukatimer::protocol::PaperSlotRecoveryRecommendation recommendation =
		dukatimer::protocol::PaperSlotRecoveryRecommendation::RetryInspection;
	const char* responseStatus = "degraded";
	const char* responseDetail =
		"recovery decision is blocked by a transient bridge or storage failure";
	const char* recommendationReason =
		"a stable recommendation requires readable blobs or an explicit open_failed absence on both paths";
};

const char* vfsStatusName(dukatimer::protocol::VfsStatusCode status);

uint16_t foldToPaperSlotCrc16(uint32_t hash) {
	return static_cast<uint16_t>(((hash >> 16u) ^ (hash & 0xFFFFu)) & 0xFFFFu);
}

uint16_t calculatePaperSlotPayloadCrc(const void* payload, size_t payloadSizeBytes) {
	if (payload == nullptr || payloadSizeBytes == 0u) {
		return 0u;
	}

	const uint8_t* bytes = static_cast<const uint8_t*>(payload);
	uint32_t hash = 2166136261u;
	for (size_t index = 0u; index < payloadSizeBytes; ++index) {
		hash ^= bytes[index];
		hash *= 16777619u;
	}

	return foldToPaperSlotCrc16(hash);
}

bool validatePaperSlotBankShape(const PaperSlotBank& bank) {
	if (bank.schemaVersion != kPaperSlotBankSchemaVersion) {
		return false;
	}

	if (bank.slotCount != kPaperSlotCount) {
		return false;
	}

	if (bank.activeSlot >= kPaperSlotCount) {
		return false;
	}

	for (uint8_t slot = 0u; slot < kPaperSlotCount; ++slot) {
		const PaperExposureProfile& profile = bank.slots[slot];
		if (profile.schemaVersion != kPaperExposureProfileSchemaVersion) {
			return false;
		}

		if (!isValidPaperGradeMode(profile.gradeMode)) {
			return false;
		}
	}

	return true;
}

const char* paperSlotBlobParseErrorName(PaperSlotBlobParseError error) {
	switch (error) {
		case PaperSlotBlobParseError::None:
			return "none";
		case PaperSlotBlobParseError::InputTooSmall:
			return "input_too_small";
		case PaperSlotBlobParseError::HeaderMagicMismatch:
			return "header_magic_mismatch";
		case PaperSlotBlobParseError::UnsupportedFormatVersion:
			return "unsupported_format_version";
		case PaperSlotBlobParseError::PayloadSizeMismatch:
			return "payload_size_mismatch";
		case PaperSlotBlobParseError::BlobSizeMismatch:
			return "blob_size_mismatch";
		case PaperSlotBlobParseError::CrcMismatch:
			return "crc_mismatch";
		case PaperSlotBlobParseError::InvalidBank:
			return "invalid_bank";
	}

	return "unknown";
}

const char* paperGradeModeName(PaperGradeMode gradeMode) {
	return isFixedGradeMode(gradeMode) ? "fixed_grade" : "multigrade";
}

void appendJsonEscaped(String& body, const char* text) {
	if (text == nullptr) {
		return;
	}

	for (const char* cursor = text; *cursor != '\0'; ++cursor) {
		switch (*cursor) {
			case '\\':
				body += "\\\\";
				break;
				body += "\"contractVersion\":4,";
				body += "\\\"";
				break;
			case '\n':
				body += "\\n";
				break;
			case '\r':
				body += "\\r";
				body += "\"serviceRecoveryTelemetry\":true,";
				break;
			case '\t':
				body += "\\t";
				break;
			default:
				body += *cursor;
				break;
		}
	}
}

const char* paperSlotAssessmentStorageName(bool stagingUsesPsram, uint32_t stagedBytes) {
	if (stagedBytes == 0u) {
		return "none";
	}

	return stagingUsesPsram ? "psram" : "heap";
}

bool paperSlotAssessmentHasStableDecisionState(const PaperSlotPathAssessment& assessment) {
	return assessment.fetchSucceeded ||
	       assessment.vfsStatus == dukatimer::protocol::VfsStatusCode::OpenFailed;
}

bool paperSlotPayloadSignatureMatches(const PaperSlotPathAssessment& active,
	                                  const PaperSlotPathAssessment& backup) {
	if (!active.fetchSucceeded || !backup.fetchSucceeded || !active.inspection.valid ||
	    !backup.inspection.valid) {
		return false;
	}

	return active.inspection.header.formatVersion == backup.inspection.header.formatVersion &&
	       active.inspection.header.payloadSize == backup.inspection.header.payloadSize &&
	       active.inspection.header.payloadCrc == backup.inspection.header.payloadCrc;
}

const char* paperSlotRecoveryRecommendationName(
	dukatimer::protocol::PaperSlotRecoveryRecommendation recommendation) {
	switch (recommendation) {
		case dukatimer::protocol::PaperSlotRecoveryRecommendation::Unknown:
			return "unknown";
		case dukatimer::protocol::PaperSlotRecoveryRecommendation::RetryInspection:
			return "retry_inspection";
		case dukatimer::protocol::PaperSlotRecoveryRecommendation::NoAction:
			return "no_action";
		case dukatimer::protocol::PaperSlotRecoveryRecommendation::KeepActive:
			return "keep_active";
		case dukatimer::protocol::PaperSlotRecoveryRecommendation::OfferRestoreBackup:
			return "offer_restore_backup";
		case dukatimer::protocol::PaperSlotRecoveryRecommendation::RecoverViaUpload:
			return "recover_via_upload";
	}

	return "unknown";
}

PaperSlotRecoveryDecision evaluatePaperSlotRecoveryDecision(
	const PaperSlotPathAssessment& activeAssessment,
	const PaperSlotPathAssessment& backupAssessment) {
	PaperSlotRecoveryDecision decision;
	decision.decisionStable = paperSlotAssessmentHasStableDecisionState(activeAssessment) &&
	                        paperSlotAssessmentHasStableDecisionState(backupAssessment);
	decision.samePayloadSignature =
		paperSlotPayloadSignatureMatches(activeAssessment, backupAssessment);
	decision.canRestoreBackup = decision.decisionStable && backupAssessment.fetchSucceeded &&
	                         backupAssessment.inspection.valid;
	decision.restoreWouldChangeActive =
		decision.canRestoreBackup &&
		(!(activeAssessment.fetchSucceeded && activeAssessment.inspection.valid) ||
		 !decision.samePayloadSignature);
	decision.manualRecoverUploadSuggested =
		decision.decisionStable && !decision.canRestoreBackup &&
		!(activeAssessment.fetchSucceeded && activeAssessment.inspection.valid);

	if (decision.decisionStable) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoveryDecisionStable;
	}
	if (activeAssessment.fetchSucceeded) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoveryActiveFetchSucceeded;
	}
	if (activeAssessment.inspection.valid) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoveryActiveBlobValid;
	}
	if (backupAssessment.fetchSucceeded) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoveryBackupFetchSucceeded;
	}
	if (backupAssessment.inspection.valid) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoveryBackupBlobValid;
	}
	if (decision.restoreWouldChangeActive) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoveryRestoreWouldChangeActive;
	}
	if (decision.samePayloadSignature) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoverySamePayloadSignature;
	}
	if (decision.manualRecoverUploadSuggested) {
		decision.flags |= dukatimer::protocol::kPaperSlotRecoveryManualRecoverUploadSuggested;
	}

	if (decision.decisionStable) {
		decision.responseStatus = "ok";
		decision.responseDetail = "paper slot recovery state computed";

		if (backupAssessment.fetchSucceeded && backupAssessment.inspection.valid) {
			if (activeAssessment.fetchSucceeded && activeAssessment.inspection.valid) {
				if (decision.samePayloadSignature) {
					decision.recommendation =
						dukatimer::protocol::PaperSlotRecoveryRecommendation::NoAction;
					decision.recommendationReason =
						"active and backup blobs already share the same payload signature";
				} else {
					decision.recommendation =
						dukatimer::protocol::PaperSlotRecoveryRecommendation::OfferRestoreBackup;
					decision.recommendationReason =
						"backup blob is valid and differs from the active blob";
				}
			} else {
				decision.recommendation =
					dukatimer::protocol::PaperSlotRecoveryRecommendation::OfferRestoreBackup;
				decision.recommendationReason =
					"backup blob is valid while the active blob is missing or invalid";
			}
		} else if (activeAssessment.fetchSucceeded && activeAssessment.inspection.valid) {
			decision.recommendation = dukatimer::protocol::PaperSlotRecoveryRecommendation::KeepActive;
			decision.recommendationReason =
				"active blob is valid and no usable backup blob is available";
		} else {
			decision.recommendation =
				dukatimer::protocol::PaperSlotRecoveryRecommendation::RecoverViaUpload;
			decision.recommendationReason =
				"neither active nor backup currently provides a valid stable blob";
		}
	}

	return decision;
}

void appendPaperSlotAssessmentJson(String& body,
	                               const char* key,
	                               const PaperSlotPathAssessment& assessment) {
	char headerMagicBuffer[11] = {};
	snprintf(headerMagicBuffer,
	         sizeof(headerMagicBuffer),
	         "0x%08lX",
	         static_cast<unsigned long>(assessment.inspection.header.magic));

	body += "\"";
	body += key;
	body += "\":{";
	body += "\"path\":\"";
	body += assessment.path;
	body += "\",\"fetchSucceeded\":";
	body += assessment.fetchSucceeded ? "true" : "false";
	body += ",\"httpStatus\":";
	body += String(assessment.httpStatusCode);
	body += ",\"status\":\"";
	appendJsonEscaped(body, assessment.statusText);
	body += "\",\"detail\":\"";
	appendJsonEscaped(body, assessment.detail);
	body += "\",\"vfsStatus\":\"";
	body += vfsStatusName(assessment.vfsStatus);
	body += "\",\"vfsStatusCode\":";
	body += String(static_cast<uint16_t>(assessment.vfsStatus));
	body += ",\"stagedBytes\":";
	body += String(assessment.stagedBytes);
	body += ",\"stagingStorage\":\"";
	body += paperSlotAssessmentStorageName(assessment.stagingUsesPsram, assessment.stagedBytes);
	body += "\",\"blobValid\":";
	body += assessment.inspection.valid ? "true" : "false";
	body += ",\"parseError\":\"";
	body += paperSlotBlobParseErrorName(assessment.inspection.parseError);
	body += "\",\"parseErrorCode\":";
	body += String(static_cast<uint8_t>(assessment.inspection.parseError));

	if (assessment.fetchSucceeded) {
		body += ",\"headerMagic\":\"";
		body += headerMagicBuffer;
		body += "\",\"formatVersion\":";
		body += String(assessment.inspection.header.formatVersion);
		body += ",\"payloadSize\":";
		body += String(assessment.inspection.header.payloadSize);
		body += ",\"payloadCrc\":";
		body += String(assessment.inspection.header.payloadCrc);
	}

	if (assessment.inspection.valid) {
		body += ",\"activeSlot\":";
		body += String(assessment.inspection.activeSlot);
		body += ",\"slotCount\":";
		body += String(assessment.inspection.slotCount);
		body += ",\"activeSlotCalibrated\":";
		body += assessment.inspection.activeSlotCalibrated ? "true" : "false";
		body += ",\"activeGradeMode\":\"";
		body += paperGradeModeName(assessment.inspection.activeGradeMode);
		body += "\",\"activeSlotUseIsoMath\":";
		body += assessment.inspection.activeSlotUseIsoMath ? "true" : "false";
		body += ",\"activeSlotFixedGradeValue\":";
		body += String(assessment.inspection.activeSlotFixedGradeValue, 2);
		body += ",\"activeSlotName\":\"";
		appendJsonEscaped(body, assessment.inspection.activeSlotName);
		body += "\"";
	}

	body += "}";
}

bool inspectPaperSlotBlob(const uint8_t* blob, size_t blobSizeBytes, PaperSlotBlobInspection* inspectionOut) {
	PaperSlotBlobInspection inspection;
	if (blob == nullptr || blobSizeBytes < sizeof(PaperSlotBlobHeader)) {
		inspection.parseError = PaperSlotBlobParseError::InputTooSmall;
		if (inspectionOut != nullptr) {
			*inspectionOut = inspection;
		}
		return false;
	}

	memcpy(&inspection.header, blob, sizeof(PaperSlotBlobHeader));
	inspection.parseError = PaperSlotBlobParseError::None;

	if (inspection.header.magic != kPaperSlotBlobMagic) {
		inspection.parseError = PaperSlotBlobParseError::HeaderMagicMismatch;
	} else if (inspection.header.formatVersion == 0u ||
	           inspection.header.formatVersion > kPaperSlotBlobFormatVersion) {
		inspection.parseError = PaperSlotBlobParseError::UnsupportedFormatVersion;
	} else if (inspection.header.payloadSize != PaperSlotPersistenceCodec::payloadSize()) {
		inspection.parseError = PaperSlotBlobParseError::PayloadSizeMismatch;
	} else {
		const size_t expectedBlobSize = sizeof(PaperSlotBlobHeader) +
		                               static_cast<size_t>(inspection.header.payloadSize);
		if (blobSizeBytes != expectedBlobSize) {
			inspection.parseError = PaperSlotBlobParseError::BlobSizeMismatch;
		} else {
			PaperSlotBank bank = {};
			memcpy(&bank, blob + sizeof(PaperSlotBlobHeader), sizeof(PaperSlotBank));
			const uint16_t expectedCrc = calculatePaperSlotPayloadCrc(&bank, sizeof(PaperSlotBank));
			if (expectedCrc != inspection.header.payloadCrc) {
				inspection.parseError = PaperSlotBlobParseError::CrcMismatch;
			} else if (!validatePaperSlotBankShape(bank)) {
				inspection.parseError = PaperSlotBlobParseError::InvalidBank;
			} else {
				inspection.valid = true;
				inspection.activeSlot = bank.activeSlot;
				inspection.slotCount = bank.slotCount;
				inspection.activeSlotCalibrated = bank.slots[bank.activeSlot].calibrated;
				inspection.activeGradeMode = bank.slots[bank.activeSlot].gradeMode;
				inspection.activeSlotUseIsoMath = bank.slots[bank.activeSlot].useIsoMath;
				inspection.activeSlotFixedGradeValue = bank.slots[bank.activeSlot].fixedGradeValue;
				memcpy(inspection.activeSlotName,
				       bank.slots[bank.activeSlot].name,
				       sizeof(inspection.activeSlotName));
				inspection.activeSlotName[sizeof(inspection.activeSlotName) - 1u] = '\0';
			}
		}
	}

	if (inspectionOut != nullptr) {
		*inspectionOut = inspection;
	}
	return inspection.valid;
}

// Upload-Pfade werden streng eingeschraenkt, damit der HTTP-Zugang nur wohldefinierte
// absolute Zielpfade an die VFS-Bruecke weitergeben kann.
bool isAllowedUploadPath(const char* path) {
	if (path == nullptr || path[0] != '/') {
		return false;
	}

	if (strstr(path, "..") != nullptr) {
		return false;
	}

	const size_t pathLength = strnlen(path, dukatimer::protocol::kVfsMaxPathLength + 1u);
	return pathLength > 0u && pathLength <= dukatimer::protocol::kVfsMaxPathLength;
}

dukatimer::protocol::VfsStatusCode statusFromError(const dukatimer::protocol::VfsErrorPayload& payload) {
	return static_cast<dukatimer::protocol::VfsStatusCode>(payload.statusCode);
}

const char* vfsStatusName(dukatimer::protocol::VfsStatusCode status) {
	switch (status) {
		case dukatimer::protocol::VfsStatusCode::None:
			return "none";
		case dukatimer::protocol::VfsStatusCode::Ok:
			return "ok";
		case dukatimer::protocol::VfsStatusCode::Busy:
			return "busy";
		case dukatimer::protocol::VfsStatusCode::InvalidPath:
			return "invalid_path";
		case dukatimer::protocol::VfsStatusCode::SdUnavailable:
			return "sd_unavailable";
		case dukatimer::protocol::VfsStatusCode::Unsupported:
			return "unsupported";
		case dukatimer::protocol::VfsStatusCode::OpenFailed:
			return "open_failed";
		case dukatimer::protocol::VfsStatusCode::WriteFailed:
			return "write_failed";
		case dukatimer::protocol::VfsStatusCode::ReadFailed:
			return "read_failed";
		case dukatimer::protocol::VfsStatusCode::InvalidState:
			return "invalid_state";
		case dukatimer::protocol::VfsStatusCode::TransactionMismatch:
			return "transaction_mismatch";
		case dukatimer::protocol::VfsStatusCode::StorageNotReady:
			return "storage_not_ready";
		case dukatimer::protocol::VfsStatusCode::FlowHeld:
			return "flow_held";
		case dukatimer::protocol::VfsStatusCode::ProtocolRejected:
			return "protocol_rejected";
	}

	return "unknown";
}

int httpStatusForVfsStatus(dukatimer::protocol::VfsStatusCode status) {
	switch (status) {
		case dukatimer::protocol::VfsStatusCode::Ok:
			return 201;
		case dukatimer::protocol::VfsStatusCode::Busy:
		case dukatimer::protocol::VfsStatusCode::FlowHeld:
			return 409;
		case dukatimer::protocol::VfsStatusCode::InvalidPath:
		case dukatimer::protocol::VfsStatusCode::ProtocolRejected:
		case dukatimer::protocol::VfsStatusCode::InvalidState:
		case dukatimer::protocol::VfsStatusCode::TransactionMismatch:
			return 400;
		case dukatimer::protocol::VfsStatusCode::SdUnavailable:
		case dukatimer::protocol::VfsStatusCode::StorageNotReady:
			return 503;
		case dukatimer::protocol::VfsStatusCode::Unsupported:
			return 501;
		case dukatimer::protocol::VfsStatusCode::OpenFailed:
		case dukatimer::protocol::VfsStatusCode::WriteFailed:
		case dukatimer::protocol::VfsStatusCode::ReadFailed:
			return 500;
		case dukatimer::protocol::VfsStatusCode::None:
		default:
			return 502;
	}
}

const char* networkModeName(HttpVfsBridge::NetworkMode mode) {
	switch (mode) {
		case HttpVfsBridge::NetworkMode::Station:
			return "station";
		case HttpVfsBridge::NetworkMode::AccessPoint:
			return "access_point";
		case HttpVfsBridge::NetworkMode::Offline:
		default:
			return "offline";
	}
}

}  // namespace

HttpVfsBridge::HttpVfsBridge(TeensyLinkService& teensyLink)
	: teensyLink_(teensyLink), server_(service_config::kHttpPort) {}

void HttpVfsBridge::setCooperativeTickHook(CooperativeTickHook hook) {
	cooperativeTickHook_ = hook;
}

void HttpVfsBridge::begin(uint32_t nowMs) {
	// begin() konfiguriert Netzwerk und Server nur einmal. Danach bedient tick()
	// sowohl den Teensy-Link als auch eingehende HTTP-Clients.
	(void)nowMs;
	if (initialized_) {
		return;
	}

	configureNetwork();
	if (networkReady_) {
		startServer();
	}

	initialized_ = true;
}

void HttpVfsBridge::tick(uint32_t nowMs) {
	teensyLink_.tick(nowMs);
	if (networkReady_) {
		server_.handleClient();
	}

	const bool fileTransactionActive = uploadRequest_.active || downloadRequest_.active ||
	                               teensyLink_.uploadInProgress() ||
	                               teensyLink_.downloadInProgress();
	teensyLink_.setFileTransactionActive(fileTransactionActive, nowMs);

	if (!initialized_ || uploadRequest_.active || downloadRequest_.active ||
	    teensyLink_.uploadInProgress() || teensyLink_.downloadInProgress() ||
	    teensyLink_.teensyExposureActive(nowMs) ||
	    (nowMs - lastPaperSlotRecoveryRefreshMs_) < kPaperSlotRecoveryRefreshIntervalMs) {
		return;
	}

	auto collectAssessment = [this](const char* path,
	                              const char* stagedDetail,
	                              PaperSlotPathAssessment& assessment) {
		assessment = {};
		assessment.inspection.parseError = PaperSlotBlobParseError::None;
		strlcpy(assessment.path, path, sizeof(assessment.path));
		resetDownloadRequest();

		if (stageDownloadToBuffer(path)) {
			assessment.fetchSucceeded = true;
			assessment.httpStatusCode = 200;
			assessment.vfsStatus = dukatimer::protocol::VfsStatusCode::Ok;
			strlcpy(assessment.statusText, "ok", sizeof(assessment.statusText));
			strlcpy(assessment.detail, stagedDetail, sizeof(assessment.detail));
			assessment.stagedBytes = static_cast<uint32_t>(uploadStagingFill_);
			assessment.stagingUsesPsram = uploadStagingUsesPsram_;
			(void)inspectPaperSlotBlob(uploadStagingBuffer_, uploadStagingFill_, &assessment.inspection);
			releaseUploadStaging();
			resetDownloadRequest();
			return;
		}

		assessment.fetchSucceeded = false;
		assessment.httpStatusCode = downloadRequest_.httpStatusCode;
		assessment.vfsStatus = downloadRequest_.vfsStatus;
		assessment.stagedBytes = downloadRequest_.stagedBytes;
		assessment.stagingUsesPsram = downloadRequest_.stagingUsesPsram;
		strlcpy(assessment.statusText,
		        downloadRequest_.statusText != nullptr ? downloadRequest_.statusText : "failed",
		        sizeof(assessment.statusText));
		strlcpy(assessment.detail,
		        downloadRequest_.detail != nullptr ? downloadRequest_.detail : "download failed",
		        sizeof(assessment.detail));
		releaseUploadStaging();
		resetDownloadRequest();
	};

	PaperSlotPathAssessment activeAssessment = {};
	PaperSlotPathAssessment backupAssessment = {};
	collectAssessment(dukatimer::product_data::kPaperSlotsPath,
	                "paper slot active blob staged for recovery monitoring",
	                activeAssessment);
	collectAssessment(dukatimer::product_data::kPaperSlotsBackupPath,
	                "paper slot backup blob staged for recovery monitoring",
	                backupAssessment);
	const PaperSlotRecoveryDecision decision =
		evaluatePaperSlotRecoveryDecision(activeAssessment, backupAssessment);
	teensyLink_.setPaperSlotRecoveryStatus(
		decision.flags,
		decision.recommendation,
		activeAssessment.vfsStatus,
		activeAssessment.fetchSucceeded ? static_cast<uint8_t>(activeAssessment.inspection.parseError) : 0u,
		backupAssessment.vfsStatus,
		backupAssessment.fetchSucceeded ? static_cast<uint8_t>(backupAssessment.inspection.parseError) : 0u);
	lastPaperSlotRecoveryRefreshMs_ = nowMs;
}

void HttpVfsBridge::configureNetwork() {
	// Station hat Prioritaet; der Access-Point ist der Bring-up- und Fallbackpfad,
	// damit die Upload-Bruecke auch ohne vorhandenes WLAN nutzbar bleibt.
	if (networkReady_) {
		return;
	}

	WiFi.setSleep(false);
	if (!tryStartStation()) {
		tryStartAccessPoint();
	}
}

bool HttpVfsBridge::tryStartStation() {
	if (service_config::kStationSsid[0] == '\0') {
		return false;
	}

	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);
	WiFi.begin(service_config::kStationSsid, service_config::kStationPassword);

	const uint32_t startMs = millis();
	while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < service_config::kStationConnectTimeoutMs) {
		delay(50);
	}

	if (WiFi.status() != WL_CONNECTED) {
		WiFi.disconnect(true, true);
		return false;
	}

	strlcpy(networkAddress_, WiFi.localIP().toString().c_str(), sizeof(networkAddress_));
	networkMode_ = NetworkMode::Station;
	networkReady_ = true;
	Serial.printf("[HTTP] WiFi STA verbunden: %s\n", networkAddress_);
	return true;
}

bool HttpVfsBridge::tryStartAccessPoint() {
	WiFi.mode(WIFI_AP);
	const bool started = WiFi.softAP(service_config::kAccessPointSsid,
	                               service_config::kAccessPointPassword,
	                               service_config::kAccessPointChannel,
	                               false,
	                               1);
	if (!started) {
		return false;
	}

	strlcpy(networkAddress_, WiFi.softAPIP().toString().c_str(), sizeof(networkAddress_));
	networkMode_ = NetworkMode::AccessPoint;
	networkReady_ = true;
	Serial.printf("[HTTP] AP aktiv: SSID=%s IP=%s\n", service_config::kAccessPointSsid, networkAddress_);
	return true;
}

void HttpVfsBridge::startServer() {
	static const char* kCollectedHeaders[] = {"X-Dukatimer-Path"};
	server_.collectHeaders(kCollectedHeaders, 1);
	registerRoutes();
	server_.begin();
	Serial.printf("[HTTP] VFS-Bridge bereit auf http://%s:%u/\n", networkAddress_, service_config::kHttpPort);
}

void HttpVfsBridge::registerRoutes() {
	server_.on("/", HTTP_GET, [this]() { handleRoot(); });
	server_.on("/api/v1/health", HTTP_GET, [this]() { handleHealth(); });
	server_.on("/api/v1/product/paperslots", HTTP_GET, [this]() { handlePaperSlotsContract(); });
	server_.on("/api/v1/product/paperslots/recovery-state", HTTP_GET,
	         [this]() { handlePaperSlotsRecoveryState(); });
	server_.on("/api/v1/product/paperslots/inspect", HTTP_GET, [this]() { handlePaperSlotsInspect(); });
	server_.on("/api/v1/product/paperslots/inspect/backup", HTTP_GET,
	         [this]() { handlePaperSlotsBackupInspect(); });
	server_.on("/api/v1/product/paperslots/export", HTTP_GET, [this]() { handlePaperSlotsExport(); });
	server_.on("/api/v1/product/paperslots/export/backup", HTTP_GET,
	         [this]() { handlePaperSlotsBackupExport(); });
	server_.on("/api/v1/product/paperslots/restore-backup", HTTP_POST,
	         [this]() { handlePaperSlotsRestoreBackup(); });
	server_.on("/api/v1/product/paperslots/recover", HTTP_POST,
	         [this]() { handlePaperSlotsRecover(); },
	         [this]() { handleUploadRaw(dukatimer::product_data::kPaperSlotsPath); });
	server_.on("/api/v1/files/upload", HTTP_POST, [this]() { handleUpload(); },
	         [this]() { handleUploadRaw(); });
	server_.onNotFound([this]() { handleNotFound(); });
}

void HttpVfsBridge::resetUploadRequest() {
	releaseUploadStaging();
	uploadRequest_ = {};
}

void HttpVfsBridge::resetDownloadRequest() {
	releaseUploadStaging();
	downloadRequest_ = {};
}

void HttpVfsBridge::releaseUploadStaging() {
	if (uploadStagingBuffer_ != nullptr) {
		free(uploadStagingBuffer_);
	}

	uploadStagingBuffer_ = nullptr;
	uploadStagingCapacity_ = 0u;
	uploadStagingFill_ = 0u;
	uploadStagingUsesPsram_ = false;
}

bool HttpVfsBridge::allocateUploadStaging(size_t byteCount) {
	releaseUploadStaging();
	if (byteCount == 0u) {
		return true;
	}

	// Grosse Import-/Exportdaten sollen bevorzugt in externem RAM landen, damit
	// der kleine interne ESP-Heap nicht fuer Produktdaten-Staging verbrannt wird.
	// Faellt PSRAM aus oder ist keines frei, folgt genau eine normale Heap-Probe.
#if defined(MALLOC_CAP_SPIRAM)
	uploadStagingBuffer_ = static_cast<uint8_t*>(
		heap_caps_malloc(byteCount, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (uploadStagingBuffer_ != nullptr) {
		uploadStagingUsesPsram_ = true;
	}
#endif

	if (uploadStagingBuffer_ == nullptr) {
		uploadStagingBuffer_ = static_cast<uint8_t*>(heap_caps_malloc(byteCount, MALLOC_CAP_8BIT));
		uploadStagingUsesPsram_ = false;
	}

	if (uploadStagingBuffer_ == nullptr) {
		return false;
	}

	uploadStagingCapacity_ = byteCount;
	uploadStagingFill_ = 0u;
	Serial.printf("[HTTP] VFS-Staging reserviert: %u Byte in %s\n",
	              static_cast<unsigned>(byteCount),
	              uploadStagingUsesPsram_ ? "PSRAM" : "Heap");
	return true;
}

bool HttpVfsBridge::flushStagedUploadToTeensy() {
	uploadRequest_.forwarding = true;
	if (teensyLink_.uploadInProgress()) {
		uploadRequest_.forwarding = false;
		markUploadFailure(409, dukatimer::protocol::VfsStatusCode::Busy, "upload_busy",
		              "another upload transaction is already active");
		return false;
	}

	if (!teensyLink_.beginFileUpload(uploadRequest_.path)) {
		uploadRequest_.forwarding = false;
		markUploadFailure(409, dukatimer::protocol::VfsStatusCode::None, "upload_rejected",
		              "VFS upload request was rejected before start");
		return false;
	}

	if (!waitForUploadReady(service_config::kVfsAckTimeoutMs)) {
		teensyLink_.abortFileUpload();
		uploadRequest_.forwarding = false;
		markUploadFailureFromLink(504, "open_failed", "timeout waiting for VFS open acknowledgement");
		return false;
	}

	for (size_t offset = 0u; offset < uploadStagingFill_;) {
		size_t chunkSize = uploadStagingFill_ - offset;
		if (chunkSize > dukatimer::protocol::kVfsChunkDataSize) {
			chunkSize = dukatimer::protocol::kVfsChunkDataSize;
		}

		if (!teensyLink_.streamFileChunk(uploadStagingBuffer_ + offset, chunkSize, false)) {
			teensyLink_.abortFileUpload();
			uploadRequest_.forwarding = false;
			markUploadFailure(409, dukatimer::protocol::VfsStatusCode::Busy, "chunk_rejected",
			              "VFS bridge rejected the next staged upload chunk");
			return false;
		}

		if (!waitForUploadReady(service_config::kVfsAckTimeoutMs)) {
			teensyLink_.abortFileUpload();
			uploadRequest_.forwarding = false;
			markUploadFailureFromLink(504, "chunk_failed", "timeout waiting for chunk acknowledgement");
			return false;
		}

		offset += chunkSize;
		uploadRequest_.bytesWritten += static_cast<uint32_t>(chunkSize);
	}

	if (!teensyLink_.finishFileUpload()) {
		teensyLink_.abortFileUpload();
		uploadRequest_.forwarding = false;
		markUploadFailure(409, dukatimer::protocol::VfsStatusCode::Busy, "finalize_rejected",
		              "VFS bridge rejected the finalize chunk");
		return false;
	}

	if (!waitForUploadCompletion(service_config::kVfsAckTimeoutMs)) {
		teensyLink_.abortFileUpload();
		uploadRequest_.forwarding = false;
		markUploadFailureFromLink(504, "finalize_failed",
		                        "timeout waiting for finalize acknowledgement");
		return false;
	}

	uploadRequest_.bytesWritten = teensyLink_.uploadedByteCount();
	uploadRequest_.vfsStatus = dukatimer::protocol::VfsStatusCode::Ok;
	uploadRequest_.forwarding = false;
	return true;
}

bool HttpVfsBridge::stageDownloadToBuffer(const char* path) {
	if (path == nullptr || !isAllowedUploadPath(path)) {
		markDownloadFailure(400, dukatimer::protocol::VfsStatusCode::InvalidPath,
		                "invalid_product_path",
		                "product-data export path must stay absolute and local");
		return false;
	}

	downloadRequest_.active = true;
	downloadRequest_.started = true;
	strlcpy(downloadRequest_.path, path, sizeof(downloadRequest_.path));

	if (teensyLink_.uploadInProgress() || teensyLink_.downloadInProgress()) {
		markDownloadFailure(409, dukatimer::protocol::VfsStatusCode::Busy, "download_busy",
		                "another VFS transaction is already active");
		return false;
	}

	uint32_t nextOffset = 0u;
	while (true) {
		if (!teensyLink_.beginFileDownload(path, nextOffset, dukatimer::protocol::kVfsChunkDataSize)) {
			markDownloadFailure(409, dukatimer::protocol::VfsStatusCode::None, "download_rejected",
			                "VFS download request was rejected before start");
			releaseUploadStaging();
			return false;
		}

		if (!waitForDownloadChunk(service_config::kVfsAckTimeoutMs)) {
			markDownloadFailureFromLink(504, "download_timeout",
			                        "timeout waiting for read acknowledgement or data chunk");
			releaseUploadStaging();
			return false;
		}

		const uint32_t fileSize = teensyLink_.downloadFileSize();
		if (downloadRequest_.expectedLength == 0u && nextOffset == 0u) {
			downloadRequest_.expectedLength = fileSize;
			if (!allocateUploadStaging(static_cast<size_t>(fileSize))) {
				markDownloadFailure(kHttpStatusInsufficientStorage,
				                dukatimer::protocol::VfsStatusCode::None,
				                "staging_unavailable",
				                "product data could not be staged in RAM or PSRAM");
				return false;
			}
			downloadRequest_.stagingUsesPsram = uploadStagingUsesPsram_;
			downloadRequest_.stagedBytes = 0u;
		} else if (fileSize != downloadRequest_.expectedLength) {
			markDownloadFailure(502, dukatimer::protocol::VfsStatusCode::ProtocolRejected,
			                "download_size_changed",
			                "VFS export size changed while the file was being staged");
			releaseUploadStaging();
			return false;
		}

		dukatimer::protocol::VfsChunkPayload chunkPayload = {};
		if (!teensyLink_.consumeDownloadedChunk(&chunkPayload)) {
			markDownloadFailure(502, dukatimer::protocol::VfsStatusCode::InvalidState,
			                "download_state_lost",
			                "download chunk was acknowledged but not retained locally");
			releaseUploadStaging();
			return false;
		}

		if (chunkPayload.fileOffset != nextOffset ||
		    (static_cast<size_t>(chunkPayload.fileOffset) + chunkPayload.dataLength) > uploadStagingCapacity_) {
			markDownloadFailure(502, dukatimer::protocol::VfsStatusCode::ProtocolRejected,
			                "download_protocol_mismatch",
			                "download chunk did not match the requested offset or local staging size");
			releaseUploadStaging();
			return false;
		}

		if (chunkPayload.dataLength > 0u) {
			memcpy(uploadStagingBuffer_ + chunkPayload.fileOffset, chunkPayload.data, chunkPayload.dataLength);
		}
		uploadStagingFill_ = chunkPayload.fileOffset + chunkPayload.dataLength;
		downloadRequest_.stagedBytes = static_cast<uint32_t>(uploadStagingFill_);
		downloadRequest_.bytesRead = downloadRequest_.stagedBytes;

		const bool finalize = (chunkPayload.flags & dukatimer::protocol::kVfsChunkFinalize) != 0u;
		if (finalize) {
			if (downloadRequest_.stagedBytes != downloadRequest_.expectedLength) {
				markDownloadFailure(502, dukatimer::protocol::VfsStatusCode::ProtocolRejected,
				                "download_length_mismatch",
				                "download reached finalize before the promised file size was fully staged");
				releaseUploadStaging();
				return false;
			}

			downloadRequest_.vfsStatus = dukatimer::protocol::VfsStatusCode::Ok;
			return true;
		}

		if (chunkPayload.dataLength == 0u) {
			markDownloadFailure(502, dukatimer::protocol::VfsStatusCode::ProtocolRejected,
			                "download_stalled",
			                "download delivered an empty non-final chunk");
			releaseUploadStaging();
			return false;
		}

		nextOffset += chunkPayload.dataLength;
	}
}

void HttpVfsBridge::captureCompletedUploadSnapshot() {
	if (!uploadRequest_.active) {
		return;
	}

	lastUpload_.valid = true;
	lastUpload_.failed = uploadRequest_.failed;
	strlcpy(lastUpload_.statusText,
	        uploadRequest_.statusText != nullptr ? uploadRequest_.statusText : "unknown",
	        sizeof(lastUpload_.statusText));
	strlcpy(lastUpload_.detail,
	        uploadRequest_.detail != nullptr ? uploadRequest_.detail : "no detail",
	        sizeof(lastUpload_.detail));
	strlcpy(lastUpload_.path, uploadRequest_.path, sizeof(lastUpload_.path));
	lastUpload_.bytesWritten = uploadRequest_.bytesWritten;
	lastUpload_.stagedBytes = uploadRequest_.stagedBytes;
	lastUpload_.expectedLength = uploadRequest_.expectedLength;
	lastUpload_.vfsStatusCode = static_cast<uint16_t>(uploadRequest_.failed ? uploadRequest_.vfsStatus
	                                                                       : dukatimer::protocol::VfsStatusCode::Ok);
	lastUpload_.stagingUsesPsram = uploadRequest_.stagingUsesPsram;

	Serial.printf("[HTTP] Upload-Resultat: status=%s path=%s staged=%lu written=%lu storage=%s\n",
	              lastUpload_.statusText,
	              lastUpload_.path[0] != '\0' ? lastUpload_.path : "-",
	              static_cast<unsigned long>(lastUpload_.stagedBytes),
	              static_cast<unsigned long>(lastUpload_.bytesWritten),
	              stagingStorageName(lastUpload_.stagingUsesPsram, lastUpload_.stagedBytes));
}

void HttpVfsBridge::captureCompletedDownloadSnapshot() {
	if (!downloadRequest_.active) {
		return;
	}

	lastDownload_.valid = true;
	lastDownload_.failed = downloadRequest_.failed;
	strlcpy(lastDownload_.statusText,
	        downloadRequest_.statusText != nullptr ? downloadRequest_.statusText : "unknown",
	        sizeof(lastDownload_.statusText));
	strlcpy(lastDownload_.detail,
	        downloadRequest_.detail != nullptr ? downloadRequest_.detail : "no detail",
	        sizeof(lastDownload_.detail));
	strlcpy(lastDownload_.path, downloadRequest_.path, sizeof(lastDownload_.path));
	lastDownload_.bytesRead = downloadRequest_.bytesRead;
	lastDownload_.stagedBytes = downloadRequest_.stagedBytes;
	lastDownload_.expectedLength = downloadRequest_.expectedLength;
	lastDownload_.vfsStatusCode = static_cast<uint16_t>(downloadRequest_.failed ? downloadRequest_.vfsStatus
	                                                                         : dukatimer::protocol::VfsStatusCode::Ok);
	lastDownload_.stagingUsesPsram = downloadRequest_.stagingUsesPsram;

	Serial.printf("[HTTP] Export-Resultat: status=%s path=%s staged=%lu read=%lu storage=%s\n",
	              lastDownload_.statusText,
	              lastDownload_.path[0] != '\0' ? lastDownload_.path : "-",
	              static_cast<unsigned long>(lastDownload_.stagedBytes),
	              static_cast<unsigned long>(lastDownload_.bytesRead),
	              stagingStorageName(lastDownload_.stagingUsesPsram, lastDownload_.stagedBytes));
}

const char* HttpVfsBridge::currentUploadPhaseName() const {
	if (!uploadRequest_.active) {
		return "idle";
	}
	if (uploadRequest_.failed) {
		return "fault";
	}
	if (uploadRequest_.forwarding) {
		return "forwarding";
	}
	if (uploadRequest_.started) {
		return "staging";
	}
	return "starting";
}

const char* HttpVfsBridge::currentDownloadPhaseName() const {
	if (!downloadRequest_.active) {
		return "idle";
	}
	if (downloadRequest_.failed) {
		return "fault";
	}
	if (downloadRequest_.serving) {
		return "serving";
	}
	if (downloadRequest_.started) {
		return "staging";
	}
	return "starting";
}

const char* HttpVfsBridge::stagingStorageName(bool stagingUsesPsram, uint32_t stagedBytes) {
	if (stagedBytes == 0u) {
		return "none";
	}

	return stagingUsesPsram ? "psram" : "heap";
}

void HttpVfsBridge::markUploadSuccess(const char* detail) {
	uploadRequest_.httpStatusCode = 201;
	uploadRequest_.statusText = "ok";
	uploadRequest_.detail = detail;
	uploadRequest_.vfsStatus = dukatimer::protocol::VfsStatusCode::Ok;
	uploadRequest_.failed = false;
	uploadRequest_.forwarding = false;
}

void HttpVfsBridge::markUploadFailure(int httpStatusCode, dukatimer::protocol::VfsStatusCode vfsStatus,
	                              const char* statusText, const char* detail) {
	if (uploadRequest_.failed) {
		return;
	}

	uploadRequest_.failed = true;
	uploadRequest_.httpStatusCode = httpStatusCode;
	uploadRequest_.statusText = statusText;
	uploadRequest_.detail = detail;
	uploadRequest_.vfsStatus = vfsStatus;
	uploadRequest_.forwarding = false;
}

void HttpVfsBridge::markUploadFailureFromLink(int fallbackStatusCode, const char* fallbackStatusText,
	                                     const char* fallbackDetail) {
	if (teensyLink_.uploadFaulted()) {
		const dukatimer::protocol::VfsStatusCode status = statusFromError(teensyLink_.lastVfsError());
		markUploadFailure(httpStatusForVfsStatus(status), status, fallbackStatusText, vfsStatusName(status));
		return;
	}

	markUploadFailure(fallbackStatusCode, dukatimer::protocol::VfsStatusCode::None, fallbackStatusText,
	               fallbackDetail);
}

void HttpVfsBridge::markDownloadSuccess(const char* detail) {
	downloadRequest_.httpStatusCode = 200;
	downloadRequest_.statusText = "ok";
	downloadRequest_.detail = detail;
	downloadRequest_.vfsStatus = dukatimer::protocol::VfsStatusCode::Ok;
	downloadRequest_.failed = false;
}

void HttpVfsBridge::markDownloadFailure(int httpStatusCode, dukatimer::protocol::VfsStatusCode vfsStatus,
	                                 const char* statusText, const char* detail) {
	if (downloadRequest_.failed) {
		return;
	}

	downloadRequest_.failed = true;
	downloadRequest_.httpStatusCode = httpStatusCode;
	downloadRequest_.statusText = statusText;
	downloadRequest_.detail = detail;
	downloadRequest_.vfsStatus = vfsStatus;
	downloadRequest_.serving = false;
}

void HttpVfsBridge::markDownloadFailureFromLink(int fallbackStatusCode, const char* fallbackStatusText,
	                                         const char* fallbackDetail) {
	if (teensyLink_.downloadFaulted()) {
		const dukatimer::protocol::VfsStatusCode status = statusFromError(teensyLink_.lastVfsError());
		markDownloadFailure(httpStatusForVfsStatus(status), status, fallbackStatusText, vfsStatusName(status));
		return;
	}

	markDownloadFailure(fallbackStatusCode, dukatimer::protocol::VfsStatusCode::None, fallbackStatusText,
	                fallbackDetail);
}

void HttpVfsBridge::handleRoot() {
	String body;
	body.reserve(768);
	body += "Dukatimer Part2 ESP32-S3 HTTP VFS bridge\n";
	body += "mode=";
	body += networkModeName(networkMode_);
	body += "\naddress=";
	body += networkAddress_;
	body += "\nendpoint=POST /api/v1/files/upload\n";
	body += "product=GET /api/v1/product/paperslots\n";
	body += "recovery-state=GET /api/v1/product/paperslots/recovery-state\n";
	body += "inspect=GET /api/v1/product/paperslots/inspect\n";
	body += "inspect-backup=GET /api/v1/product/paperslots/inspect/backup\n";
	body += "export=GET /api/v1/product/paperslots/export\n";
	body += "backup=GET /api/v1/product/paperslots/export/backup\n";
	body += "restore-backup=POST /api/v1/product/paperslots/restore-backup\n";
	body += "recover=POST /api/v1/product/paperslots/recover\n";
	body += "health=GET /api/v1/health (inkl. Staging-Diagnose)\n";
	body += "header=X-Dukatimer-Path: /target/file.bin\n";
	body += "body=raw binary with Content-Length\n";
	body += "example=curl -H \"X-Dukatimer-Path: /upload/local.bin\" --data-binary @local.bin \"http://";
	body += networkAddress_;
	body += "/api/v1/files/upload\"\n";
	body += "paper-export=curl -OJ \"http://";
	body += networkAddress_;
	body += "/api/v1/product/paperslots/export\"\n";
	body += "paper-recovery-state=curl \"http://";
	body += networkAddress_;
	body += "/api/v1/product/paperslots/recovery-state\"\n";
	body += "paper-inspect=curl \"http://";
	body += networkAddress_;
	body += "/api/v1/product/paperslots/inspect\"\n";
	body += "paper-recover=curl --data-binary @paperslots.bin \"http://";
	body += networkAddress_;
	body += "/api/v1/product/paperslots/recover\"\n";
	body += "paper-restore-backup=curl -X POST \"http://";
	body += networkAddress_;
	body += "/api/v1/product/paperslots/restore-backup\"\n";
	server_.send(200, "text/plain; charset=utf-8", body);
}

void HttpVfsBridge::handleHealth() {
	const dukatimer::protocol::VfsStatusCode lastStatus = teensyLink_.uploadFaulted()
		? statusFromError(teensyLink_.lastVfsError())
		: static_cast<dukatimer::protocol::VfsStatusCode>(teensyLink_.lastVfsAck().statusCode);
	String body;
	body.reserve(768);
	body += "{";
	body += "\"networkMode\":\"";
	body += networkModeName(networkMode_);
	body += "\",";
	body += "\"address\":\"";
	body += networkAddress_;
	body += "\",";
	body += "\"uploadBusy\":";
	body += teensyLink_.uploadInProgress() ? "true" : "false";
	body += ",\"uploadCompleted\":";
	body += teensyLink_.uploadCompletedSuccessfully() ? "true" : "false";
	body += ",\"uploadFaulted\":";
	body += teensyLink_.uploadFaulted() ? "true" : "false";
	body += ",\"uploadedBytes\":";
	body += String(teensyLink_.uploadedByteCount());
	body += ",\"lastVfsStatus\":\"";
	body += vfsStatusName(lastStatus);
	body += "\",\"httpUploadActive\":";
	body += uploadRequest_.active ? "true" : "false";
	body += ",\"httpUploadPhase\":\"";
	body += currentUploadPhaseName();
	body += "\",\"httpUploadExpectedBytes\":";
	body += String(uploadRequest_.expectedLength);
	body += ",\"httpUploadStagedBytes\":";
	body += String(uploadRequest_.stagedBytes);
	body += ",\"httpUploadStagingStorage\":\"";
	body += stagingStorageName(uploadRequest_.stagingUsesPsram, uploadRequest_.stagedBytes);
	body += "\"";
	if (uploadRequest_.path[0] != '\0') {
		body += ",\"httpUploadPath\":\"";
		body += uploadRequest_.path;
		body += "\"";
	}
	body += ",\"httpDownloadActive\":";
	body += downloadRequest_.active ? "true" : "false";
	body += ",\"httpDownloadPhase\":\"";
	body += currentDownloadPhaseName();
	body += "\",\"httpDownloadExpectedBytes\":";
	body += String(downloadRequest_.expectedLength);
	body += ",\"httpDownloadStagedBytes\":";
	body += String(downloadRequest_.stagedBytes);
	body += ",\"httpDownloadStagingStorage\":\"";
	body += stagingStorageName(downloadRequest_.stagingUsesPsram, downloadRequest_.stagedBytes);
	body += "\"";
	if (downloadRequest_.path[0] != '\0') {
		body += ",\"httpDownloadPath\":\"";
		body += downloadRequest_.path;
		body += "\"";
	}
	body += ",\"lastHttpUploadValid\":";
	body += lastUpload_.valid ? "true" : "false";
	body += ",\"lastHttpUploadFailed\":";
	body += lastUpload_.failed ? "true" : "false";
	body += ",\"lastHttpUploadStatus\":\"";
	body += lastUpload_.statusText;
	body += "\",\"lastHttpUploadDetail\":\"";
	body += lastUpload_.detail;
	body += "\",\"lastHttpUploadExpectedBytes\":";
	body += String(lastUpload_.expectedLength);
	body += ",\"lastHttpUploadStagedBytes\":";
	body += String(lastUpload_.stagedBytes);
	body += ",\"lastHttpUploadBytesWritten\":";
	body += String(lastUpload_.bytesWritten);
	body += ",\"lastHttpUploadVfsStatusCode\":";
	body += String(lastUpload_.vfsStatusCode);
	body += ",\"lastHttpUploadStagingStorage\":\"";
	body += stagingStorageName(lastUpload_.stagingUsesPsram, lastUpload_.stagedBytes);
	body += "\"";
	if (lastUpload_.path[0] != '\0') {
		body += ",\"lastHttpUploadPath\":\"";
		body += lastUpload_.path;
		body += "\"";
	}
	body += ",\"lastHttpDownloadValid\":";
	body += lastDownload_.valid ? "true" : "false";
	body += ",\"lastHttpDownloadFailed\":";
	body += lastDownload_.failed ? "true" : "false";
	body += ",\"lastHttpDownloadStatus\":\"";
	body += lastDownload_.statusText;
	body += "\",\"lastHttpDownloadDetail\":\"";
	body += lastDownload_.detail;
	body += "\",\"lastHttpDownloadExpectedBytes\":";
	body += String(lastDownload_.expectedLength);
	body += ",\"lastHttpDownloadStagedBytes\":";
	body += String(lastDownload_.stagedBytes);
	body += ",\"lastHttpDownloadBytesRead\":";
	body += String(lastDownload_.bytesRead);
	body += ",\"lastHttpDownloadVfsStatusCode\":";
	body += String(lastDownload_.vfsStatusCode);
	body += ",\"lastHttpDownloadStagingStorage\":\"";
	body += stagingStorageName(lastDownload_.stagingUsesPsram, lastDownload_.stagedBytes);
	body += "\"";
	if (lastDownload_.path[0] != '\0') {
		body += ",\"lastHttpDownloadPath\":\"";
		body += lastDownload_.path;
		body += "\"";
	}
	body += "}";
	server_.send(200, "application/json", body);
}

void HttpVfsBridge::handlePaperSlotsRecoveryState() {
	PaperSlotPathAssessment activeAssessment = {};
	PaperSlotPathAssessment backupAssessment = {};

	// Die Recovery-Entscheidung soll denselben Blob-Vertrag wie Recover/Restore
	// benutzen, aber keinerlei Schreibpfad anstossen. So kann spaetere UI einen
	// expliziten Restore-Vorschlag anzeigen, ohne wertvolle Papierdaten blind zu
	// ueberschreiben oder den letzten Export-Healthzustand zu verfremden.
	auto collectAssessment = [this](const char* path,
	                              const char* stagedDetail,
	                              PaperSlotPathAssessment& assessment) {
		assessment = {};
		assessment.inspection.parseError = PaperSlotBlobParseError::None;
		strlcpy(assessment.path, path, sizeof(assessment.path));
		resetDownloadRequest();

		if (stageDownloadToBuffer(path)) {
			assessment.fetchSucceeded = true;
			assessment.httpStatusCode = 200;
			assessment.vfsStatus = dukatimer::protocol::VfsStatusCode::Ok;
			strlcpy(assessment.statusText, "ok", sizeof(assessment.statusText));
			strlcpy(assessment.detail, stagedDetail, sizeof(assessment.detail));
			assessment.stagedBytes = static_cast<uint32_t>(uploadStagingFill_);
			assessment.stagingUsesPsram = uploadStagingUsesPsram_;
			(void)inspectPaperSlotBlob(uploadStagingBuffer_, uploadStagingFill_, &assessment.inspection);
			releaseUploadStaging();
			resetDownloadRequest();
			return;
		}

		assessment.fetchSucceeded = false;
		assessment.httpStatusCode = downloadRequest_.httpStatusCode;
		assessment.vfsStatus = downloadRequest_.vfsStatus;
		assessment.stagedBytes = downloadRequest_.stagedBytes;
		assessment.stagingUsesPsram = downloadRequest_.stagingUsesPsram;
		strlcpy(assessment.statusText,
		        downloadRequest_.statusText != nullptr ? downloadRequest_.statusText : "failed",
		        sizeof(assessment.statusText));
		strlcpy(assessment.detail,
		        downloadRequest_.detail != nullptr ? downloadRequest_.detail : "download failed",
		        sizeof(assessment.detail));
		releaseUploadStaging();
		resetDownloadRequest();
	};

	collectAssessment(dukatimer::product_data::kPaperSlotsPath,
	                "paper slot active blob staged for recovery inspection",
	                activeAssessment);
	collectAssessment(dukatimer::product_data::kPaperSlotsBackupPath,
	                "paper slot backup blob staged for recovery inspection",
	                backupAssessment);

	const PaperSlotRecoveryDecision decision =
		evaluatePaperSlotRecoveryDecision(activeAssessment, backupAssessment);

	String body;
	body.reserve(1800);
	body += "{";
	body += "\"status\":\"";
	body += decision.responseStatus;
	body += "\",\"detail\":\"";
	body += decision.responseDetail;
	body += "\",\"product\":\"paper_slots\",\"decisionStable\":";
	body += decision.decisionStable ? "true" : "false";
	body += ",\"canRestoreBackup\":";
	body += decision.canRestoreBackup ? "true" : "false";
	body += ",\"restoreWouldChangeActive\":";
	body += decision.restoreWouldChangeActive ? "true" : "false";
	body += ",\"samePayloadSignature\":";
	body += decision.samePayloadSignature ? "true" : "false";
	body += ",\"manualRecoverUploadSuggested\":";
	body += decision.manualRecoverUploadSuggested ? "true" : "false";
	body += ",\"recoveryFlags\":";
	body += String(decision.flags);
	body += ",\"recommendedAction\":\"";
	body += paperSlotRecoveryRecommendationName(decision.recommendation);
	body += "\",\"recommendedActionCode\":";
	body += String(static_cast<uint8_t>(decision.recommendation));
	body += "\",\"recommendationReason\":\"";
	body += decision.recommendationReason;
	body += "\",";
	appendPaperSlotAssessmentJson(body, "active", activeAssessment);
	body += ",";
	appendPaperSlotAssessmentJson(body, "backup", backupAssessment);
	body += "}";
	server_.send(200, "application/json", body);
}

void HttpVfsBridge::handlePaperSlotsContract() {
	String body;
	body.reserve(640);
	body += "{";
	body += "\"product\":\"paper_slots\",";
	body += "\"contractVersion\":3,";
	body += "\"contentType\":\"application/octet-stream\",";
	body += "\"validationRequired\":true,";
	body += "\"storagePath\":\"";
	body += dukatimer::product_data::kPaperSlotsPath;
	body += "\",\"backupPath\":\"";
	body += dukatimer::product_data::kPaperSlotsBackupPath;
	body += "\",\"recoveryStateEndpoint\":\"/api/v1/product/paperslots/recovery-state\",";
	body += "\",\"inspectEndpoint\":\"/api/v1/product/paperslots/inspect\",";
	body += "\"inspectBackupEndpoint\":\"/api/v1/product/paperslots/inspect/backup\",";
	body += "\"exportEndpoint\":\"/api/v1/product/paperslots/export\",";
	body += "\"backupExportEndpoint\":\"/api/v1/product/paperslots/export/backup\",";
	body += "\"restoreBackupEndpoint\":\"/api/v1/product/paperslots/restore-backup\",";
	body += "\"recoverEndpoint\":\"/api/v1/product/paperslots/recover\",";
	body += "\"healthEndpoint\":\"/api/v1/health\",";
	body += "\"invalidBlobStatus\":422,";
	body += "\"recoveryPolicy\":\"reject_invalid_blob\",";
	body += "\"recoveryOfferMode\":\"explicit_non_destructive\",";
	body += "\"lastExportStatus\":\"";
	body += lastDownload_.statusText;
	body += "\",\"lastRecoverStatus\":\"";
	body += lastUpload_.statusText;
	body += "\"}";
	server_.send(200, "application/json", body);
}

void HttpVfsBridge::handlePaperSlotsInspect() {
	resetDownloadRequest();
	if (!stageDownloadToBuffer(dukatimer::product_data::kPaperSlotsPath)) {
		sendJsonResponse(downloadRequest_.httpStatusCode,
		               downloadRequest_.statusText,
		               downloadRequest_.detail,
		               downloadRequest_.path,
		               downloadRequest_.bytesRead,
		               static_cast<uint16_t>(downloadRequest_.vfsStatus),
		               downloadRequest_.stagedBytes,
		               downloadRequest_.stagingUsesPsram);
		resetDownloadRequest();
		return;
	}

	PaperSlotBlobInspection inspection;
	const bool valid = inspectPaperSlotBlob(uploadStagingBuffer_, uploadStagingFill_, &inspection);
	char headerMagicBuffer[11] = {};
	snprintf(headerMagicBuffer,
	         sizeof(headerMagicBuffer),
	         "0x%08lX",
	         static_cast<unsigned long>(inspection.header.magic));
	String body;
	body.reserve(640);
	body += "{";
	body += "\"status\":\"";
	body += valid ? "ok" : "invalid_blob";
	body += "\",\"detail\":\"";
	body += valid ? "paper slot blob validated" : paperSlotBlobParseErrorName(inspection.parseError);
	body += "\",\"product\":\"paper_slots\",\"path\":\"";
	body += dukatimer::product_data::kPaperSlotsPath;
	body += "\",\"stagedBytes\":";
	body += String(uploadStagingFill_);
	body += ",\"stagingStorage\":\"";
	body += stagingStorageName(uploadStagingUsesPsram_, static_cast<uint32_t>(uploadStagingFill_));
	body += "\",\"blobValid\":";
	body += valid ? "true" : "false";
	body += ",\"parseError\":\"";
	body += paperSlotBlobParseErrorName(inspection.parseError);
	body += "\",\"parseErrorCode\":";
	body += String(static_cast<uint8_t>(inspection.parseError));
	body += ",\"headerMagic\":\"";
	body += headerMagicBuffer;
	body += "\",\"formatVersion\":";
	body += String(inspection.header.formatVersion);
	body += ",\"payloadSize\":";
	body += String(inspection.header.payloadSize);
	body += ",\"payloadCrc\":";
	body += String(inspection.header.payloadCrc);
	body += ",\"expectedBlobSize\":";
	body += String(PaperSlotPersistenceCodec::blobSize());
	if (valid) {
		body += ",\"activeSlot\":";
		body += String(inspection.activeSlot);
		body += ",\"slotCount\":";
		body += String(inspection.slotCount);
		body += ",\"activeSlotCalibrated\":";
		body += inspection.activeSlotCalibrated ? "true" : "false";
		body += ",\"activeGradeMode\":\"";
		body += paperGradeModeName(inspection.activeGradeMode);
		body += "\",\"activeSlotUseIsoMath\":";
		body += inspection.activeSlotUseIsoMath ? "true" : "false";
		body += ",\"activeSlotFixedGradeValue\":";
		body += String(inspection.activeSlotFixedGradeValue, 2);
		body += ",\"activeSlotName\":\"";
		appendJsonEscaped(body, inspection.activeSlotName);
		body += "\"";
	}
	body += "}";
	server_.send(valid ? 200 : kHttpStatusUnprocessableContent, "application/json", body);
	resetDownloadRequest();
}

void HttpVfsBridge::handlePaperSlotsBackupInspect() {
	resetDownloadRequest();
	if (!stageDownloadToBuffer(dukatimer::product_data::kPaperSlotsBackupPath)) {
		sendJsonResponse(downloadRequest_.httpStatusCode,
		               downloadRequest_.statusText,
		               downloadRequest_.detail,
		               downloadRequest_.path,
		               downloadRequest_.bytesRead,
		               static_cast<uint16_t>(downloadRequest_.vfsStatus),
		               downloadRequest_.stagedBytes,
		               downloadRequest_.stagingUsesPsram);
		resetDownloadRequest();
		return;
	}

	PaperSlotBlobInspection inspection;
	const bool valid = inspectPaperSlotBlob(uploadStagingBuffer_, uploadStagingFill_, &inspection);
	char headerMagicBuffer[11] = {};
	snprintf(headerMagicBuffer,
	         sizeof(headerMagicBuffer),
	         "0x%08lX",
	         static_cast<unsigned long>(inspection.header.magic));
	String body;
	body.reserve(640);
	body += "{";
	body += "\"status\":\"";
	body += valid ? "ok" : "invalid_blob";
	body += "\",\"detail\":\"";
	body += valid ? "paper slot backup blob validated" : paperSlotBlobParseErrorName(inspection.parseError);
	body += "\",\"product\":\"paper_slots_backup\",\"path\":\"";
	body += dukatimer::product_data::kPaperSlotsBackupPath;
	body += "\",\"stagedBytes\":";
	body += String(uploadStagingFill_);
	body += ",\"stagingStorage\":\"";
	body += stagingStorageName(uploadStagingUsesPsram_, static_cast<uint32_t>(uploadStagingFill_));
	body += "\",\"blobValid\":";
	body += valid ? "true" : "false";
	body += ",\"parseError\":\"";
	body += paperSlotBlobParseErrorName(inspection.parseError);
	body += "\",\"parseErrorCode\":";
	body += String(static_cast<uint8_t>(inspection.parseError));
	body += ",\"headerMagic\":\"";
	body += headerMagicBuffer;
	body += "\",\"formatVersion\":";
	body += String(inspection.header.formatVersion);
	body += ",\"payloadSize\":";
	body += String(inspection.header.payloadSize);
	body += ",\"payloadCrc\":";
	body += String(inspection.header.payloadCrc);
	body += ",\"expectedBlobSize\":";
	body += String(PaperSlotPersistenceCodec::blobSize());
	if (valid) {
		body += ",\"activeSlot\":";
		body += String(inspection.activeSlot);
		body += ",\"slotCount\":";
		body += String(inspection.slotCount);
		body += ",\"activeSlotCalibrated\":";
		body += inspection.activeSlotCalibrated ? "true" : "false";
		body += ",\"activeGradeMode\":\"";
		body += paperGradeModeName(inspection.activeGradeMode);
		body += "\",\"activeSlotUseIsoMath\":";
		body += inspection.activeSlotUseIsoMath ? "true" : "false";
		body += ",\"activeSlotFixedGradeValue\":";
		body += String(inspection.activeSlotFixedGradeValue, 2);
		body += ",\"activeSlotName\":\"";
		appendJsonEscaped(body, inspection.activeSlotName);
		body += "\"";
	}
	body += "}";
	server_.send(valid ? 200 : kHttpStatusUnprocessableContent, "application/json", body);
	resetDownloadRequest();
}

void HttpVfsBridge::handlePaperSlotsExport() {
	resetDownloadRequest();
	if (!stageDownloadToBuffer(dukatimer::product_data::kPaperSlotsPath)) {
		captureCompletedDownloadSnapshot();
		sendJsonResponse(downloadRequest_.httpStatusCode,
		               downloadRequest_.statusText,
		               downloadRequest_.detail,
		               downloadRequest_.path,
		               downloadRequest_.bytesRead,
		               static_cast<uint16_t>(downloadRequest_.vfsStatus),
		               downloadRequest_.stagedBytes,
		               downloadRequest_.stagingUsesPsram);
		resetDownloadRequest();
		return;
	}

	downloadRequest_.serving = true;
	markDownloadSuccess("paper slot bank staged for export");
	captureCompletedDownloadSnapshot();
	server_.sendHeader("Cache-Control", "no-store");
	server_.sendHeader("Content-Disposition", "attachment; filename=\"paperslots.bin\"");
	server_.sendHeader("X-Dukatimer-Product", "paper_slots");
	server_.sendHeader("X-Dukatimer-Path", dukatimer::product_data::kPaperSlotsPath);
	server_.sendHeader("X-Dukatimer-Staged-Bytes", String(downloadRequest_.stagedBytes));
	server_.sendHeader("X-Dukatimer-Staging-Storage",
	                 stagingStorageName(downloadRequest_.stagingUsesPsram, downloadRequest_.stagedBytes));
	server_.sendHeader("X-Dukatimer-File-Size", String(downloadRequest_.expectedLength));
	server_.setContentLength(downloadRequest_.stagedBytes);
	server_.send(200, "application/octet-stream", "");
	if (uploadStagingFill_ > 0u && uploadStagingBuffer_ != nullptr) {
		(void)server_.client().write(uploadStagingBuffer_, uploadStagingFill_);
	}
	resetDownloadRequest();
}

void HttpVfsBridge::handlePaperSlotsBackupExport() {
	resetDownloadRequest();
	if (!stageDownloadToBuffer(dukatimer::product_data::kPaperSlotsBackupPath)) {
		captureCompletedDownloadSnapshot();
		sendJsonResponse(downloadRequest_.httpStatusCode,
		               downloadRequest_.statusText,
		               downloadRequest_.detail,
		               downloadRequest_.path,
		               downloadRequest_.bytesRead,
		               static_cast<uint16_t>(downloadRequest_.vfsStatus),
		               downloadRequest_.stagedBytes,
		               downloadRequest_.stagingUsesPsram);
		resetDownloadRequest();
		return;
	}

	downloadRequest_.serving = true;
	markDownloadSuccess("paper slot backup staged for export");
	captureCompletedDownloadSnapshot();
	server_.sendHeader("Cache-Control", "no-store");
	server_.sendHeader("Content-Disposition", "attachment; filename=\"paperslots.bin.bak\"");
	server_.sendHeader("X-Dukatimer-Product", "paper_slots_backup");
	server_.sendHeader("X-Dukatimer-Path", dukatimer::product_data::kPaperSlotsBackupPath);
	server_.sendHeader("X-Dukatimer-Staged-Bytes", String(downloadRequest_.stagedBytes));
	server_.sendHeader("X-Dukatimer-Staging-Storage",
	                 stagingStorageName(downloadRequest_.stagingUsesPsram, downloadRequest_.stagedBytes));
	server_.sendHeader("X-Dukatimer-File-Size", String(downloadRequest_.expectedLength));
	server_.setContentLength(downloadRequest_.stagedBytes);
	server_.send(200, "application/octet-stream", "");
	if (uploadStagingFill_ > 0u && uploadStagingBuffer_ != nullptr) {
		(void)server_.client().write(uploadStagingBuffer_, uploadStagingFill_);
	}
	resetDownloadRequest();
}

void HttpVfsBridge::handlePaperSlotsRestoreBackup() {
	resetUploadRequest();
	resetDownloadRequest();
	if (!stageDownloadToBuffer(dukatimer::product_data::kPaperSlotsBackupPath)) {
		captureCompletedDownloadSnapshot();
		sendJsonResponse(downloadRequest_.httpStatusCode,
		               downloadRequest_.statusText,
		               downloadRequest_.detail,
		               downloadRequest_.path,
		               downloadRequest_.bytesRead,
		               static_cast<uint16_t>(downloadRequest_.vfsStatus),
		               downloadRequest_.stagedBytes,
		               downloadRequest_.stagingUsesPsram);
		resetDownloadRequest();
		return;
	}

	PaperSlotBlobInspection inspection;
	if (!inspectPaperSlotBlob(uploadStagingBuffer_, uploadStagingFill_, &inspection)) {
		markDownloadFailure(kHttpStatusUnprocessableContent,
		                dukatimer::protocol::VfsStatusCode::ProtocolRejected,
		                "invalid_paperslot_backup",
		                paperSlotBlobParseErrorName(inspection.parseError));
		captureCompletedDownloadSnapshot();
		sendJsonResponse(downloadRequest_.httpStatusCode,
		               downloadRequest_.statusText,
		               downloadRequest_.detail,
		               downloadRequest_.path,
		               downloadRequest_.bytesRead,
		               static_cast<uint16_t>(downloadRequest_.vfsStatus),
		               downloadRequest_.stagedBytes,
		               downloadRequest_.stagingUsesPsram);
		resetDownloadRequest();
		return;
	}

	markDownloadSuccess("paper slot backup staged for internal restore");
	captureCompletedDownloadSnapshot();
	uploadRequest_ = {};
	uploadRequest_.active = true;
	uploadRequest_.started = true;
	strlcpy(uploadRequest_.path, dukatimer::product_data::kPaperSlotsPath, sizeof(uploadRequest_.path));
	uploadRequest_.expectedLength = downloadRequest_.expectedLength;
	uploadRequest_.stagedBytes = downloadRequest_.stagedBytes;
	uploadRequest_.stagingUsesPsram = downloadRequest_.stagingUsesPsram;
	if (!flushStagedUploadToTeensy()) {
		captureCompletedUploadSnapshot();
		sendJsonResponse(uploadRequest_.httpStatusCode,
		               uploadRequest_.statusText,
		               uploadRequest_.detail,
		               uploadRequest_.path,
		               uploadRequest_.bytesWritten,
		               static_cast<uint16_t>(uploadRequest_.vfsStatus),
		               uploadRequest_.stagedBytes,
		               uploadRequest_.stagingUsesPsram);
		resetUploadRequest();
		downloadRequest_ = {};
		return;
	}

	markUploadSuccess("paper slot backup restored to active storage");
	captureCompletedUploadSnapshot();
	sendJsonResponse(200,
	               "ok",
	               "paper slot backup restored to active storage",
	               dukatimer::product_data::kPaperSlotsPath,
	               uploadRequest_.bytesWritten,
	               static_cast<uint16_t>(dukatimer::protocol::VfsStatusCode::Ok),
	               uploadRequest_.stagedBytes,
	               uploadRequest_.stagingUsesPsram);
	resetUploadRequest();
	downloadRequest_ = {};
}

void HttpVfsBridge::handlePaperSlotsRecover() {
	handleUpload();
}

void HttpVfsBridge::handleUpload() {
	if (!uploadRequest_.active) {
		sendJsonResponse(400,
		               "missing_raw_state",
		               "raw upload handler did not initialize the request",
		               nullptr,
		               0u,
		               0u,
		               uploadRequest_.stagedBytes,
		               uploadRequest_.stagingUsesPsram);
		return;
	}

	if (!networkReady_) {
		sendJsonResponse(503,
		               "network_offline",
		               "network not ready",
		               uploadRequest_.path,
		               uploadRequest_.bytesWritten,
		               static_cast<uint16_t>(uploadRequest_.vfsStatus),
		               uploadRequest_.stagedBytes,
		               uploadRequest_.stagingUsesPsram);
		resetUploadRequest();
		return;
	}

	const char* path = uploadRequest_.path[0] != '\0' ? uploadRequest_.path : nullptr;
	if (!uploadRequest_.failed) {
		markUploadSuccess("upload completed");
	}
	captureCompletedUploadSnapshot();
	sendJsonResponse(uploadRequest_.failed ? uploadRequest_.httpStatusCode : 201,
	               uploadRequest_.failed ? uploadRequest_.statusText : "ok",
	               uploadRequest_.failed ? uploadRequest_.detail : "upload completed",
	               path,
	               uploadRequest_.bytesWritten,
	               static_cast<uint16_t>(uploadRequest_.failed ? uploadRequest_.vfsStatus
	                                                         : dukatimer::protocol::VfsStatusCode::Ok),
	               uploadRequest_.stagedBytes,
	               uploadRequest_.stagingUsesPsram);
	resetUploadRequest();
}

void HttpVfsBridge::handleUploadRaw(const char* fixedPath) {
	// Der rohe Uploadpfad arbeitet jetzt RAM-first: HTTP nimmt den kompletten Body
	// zunaechst lokal an und startet den Teensy-VFS-Transfer erst nach RAW_END.
	// Dadurch erzeugen abgebrochene Requests keine halbfertigen Temp-Dateien auf
	// dem Zielsystem, und spaetere Import-/Backup-Pfade koennen denselben Buffer
	// zuerst validieren oder vorverarbeiten.
	HTTPRaw& raw = server_.raw();
	if (raw.status == RAW_START) {
		resetUploadRequest();
		uploadRequest_.active = true;
		const int contentLength = server_.clientContentLength();
		if (contentLength < 0) {
			markUploadFailure(411, dukatimer::protocol::VfsStatusCode::None, "missing_length",
			              "Content-Length is required for raw uploads");
			return;
		}

		uploadRequest_.expectedLength = static_cast<uint32_t>(contentLength);
		if (!allocateUploadStaging(static_cast<size_t>(uploadRequest_.expectedLength))) {
			markUploadFailure(kHttpStatusInsufficientStorage, dukatimer::protocol::VfsStatusCode::None,
			              "staging_unavailable",
			              "upload body could not be staged in RAM or PSRAM");
			return;
		}
		uploadRequest_.stagingUsesPsram = uploadStagingUsesPsram_;
		uploadRequest_.stagedBytes = 0u;

		const char* resolvedPath = fixedPath;
		String pathHeader;
		if (resolvedPath == nullptr) {
			pathHeader = server_.header("X-Dukatimer-Path");
			resolvedPath = pathHeader.c_str();
		}

		if (!isAllowedUploadPath(resolvedPath)) {
			markUploadFailure(400, dukatimer::protocol::VfsStatusCode::InvalidPath, "invalid_path",
			              "X-Dukatimer-Path must start with '/' and must not contain '..'");
			return;
		}

		strlcpy(uploadRequest_.path, resolvedPath, sizeof(uploadRequest_.path));
		if (teensyLink_.uploadInProgress()) {
			markUploadFailure(409, dukatimer::protocol::VfsStatusCode::Busy, "upload_busy",
			              "another upload transaction is already active");
			return;
		}

		uploadRequest_.started = true;
		return;
	}

	if (!uploadRequest_.active || uploadRequest_.failed || !uploadRequest_.started) {
		return;
	}

	if (raw.status == RAW_WRITE) {
		if ((uploadStagingFill_ + raw.currentSize) > uploadStagingCapacity_) {
			markUploadFailure(400, dukatimer::protocol::VfsStatusCode::ProtocolRejected, "raw_overflow",
			              "raw upload exceeded the announced Content-Length");
			return;
		}

		if (raw.currentSize > 0u && raw.buf != nullptr) {
			memcpy(uploadStagingBuffer_ + uploadStagingFill_, raw.buf, raw.currentSize);
			uploadStagingFill_ += raw.currentSize;
			uploadRequest_.stagedBytes = static_cast<uint32_t>(uploadStagingFill_);
		}
		return;
	}

	if (raw.status == RAW_END) {
		if (uploadStagingFill_ != static_cast<size_t>(uploadRequest_.expectedLength)) {
			markUploadFailure(400, dukatimer::protocol::VfsStatusCode::ProtocolRejected, "length_mismatch",
			              "raw upload ended before the announced Content-Length was fully staged");
			releaseUploadStaging();
			return;
		}

		if (fixedPath != nullptr && strcmp(fixedPath, dukatimer::product_data::kPaperSlotsPath) == 0) {
			PaperSlotBlobInspection inspection;
			if (!inspectPaperSlotBlob(uploadStagingBuffer_, uploadStagingFill_, &inspection)) {
				markUploadFailure(kHttpStatusUnprocessableContent,
				              dukatimer::protocol::VfsStatusCode::ProtocolRejected,
				              "invalid_paperslot_blob",
				              paperSlotBlobParseErrorName(inspection.parseError));
				releaseUploadStaging();
				return;
			}
		}

		(void)flushStagedUploadToTeensy();
		releaseUploadStaging();
		return;
	}

	if (raw.status == RAW_ABORTED) {
		if (teensyLink_.uploadInProgress()) {
			teensyLink_.abortFileUpload();
		}
		releaseUploadStaging();
		markUploadFailure(408, dukatimer::protocol::VfsStatusCode::None, "raw_aborted",
		              "HTTP client aborted the raw upload");
	}
}

void HttpVfsBridge::handleNotFound() {
	sendJsonResponse(404, "not_found", "route not found");
}

void HttpVfsBridge::runCooperativeWaitWork(uint32_t nowMs) {
	if (cooperativeTickHook_ != nullptr) {
		cooperativeTickHook_(nowMs);
	}

	yield();
}

bool HttpVfsBridge::waitForDownloadChunk(uint32_t timeoutMs) {
	const uint32_t startMs = millis();
	while ((millis() - startMs) < timeoutMs) {
		const uint32_t nowMs = millis();
		teensyLink_.tick(nowMs);
		if (teensyLink_.downloadCompletedSuccessfully() && teensyLink_.downloadChunkPending()) {
			return true;
		}
		if (teensyLink_.downloadFaulted()) {
			return false;
		}
		runCooperativeWaitWork(nowMs);
	}

	return false;
}

bool HttpVfsBridge::waitForUploadReady(uint32_t timeoutMs) {
	// Diese Wartefunktion ist bewusst ein lokaler Brueckenkompromiss fuer den Upload-
	// Requestpfad. Sie pumpt den Teensy-Link weiter, bis der naechste Chunk erlaubt ist
	// oder ein Timeout/Fault auftritt.
	const uint32_t startMs = millis();
	while ((millis() - startMs) < timeoutMs) {
		const uint32_t nowMs = millis();
		teensyLink_.tick(nowMs);
		if (teensyLink_.uploadReadyForNextChunk()) {
			return true;
		}
		if (teensyLink_.uploadFaulted()) {
			return false;
		}
		runCooperativeWaitWork(nowMs);
	}

	return false;
}

bool HttpVfsBridge::waitForUploadCompletion(uint32_t timeoutMs) {
	const uint32_t startMs = millis();
	while ((millis() - startMs) < timeoutMs) {
		const uint32_t nowMs = millis();
		teensyLink_.tick(nowMs);
		if (teensyLink_.uploadCompletedSuccessfully()) {
			return true;
		}
		if (teensyLink_.uploadFaulted()) {
			return false;
		}
		runCooperativeWaitWork(nowMs);
	}

	return false;
}

void HttpVfsBridge::sendJsonResponse(int statusCode, const char* statusText, const char* detail,
	                               const char* path, uint32_t bytesWritten, uint16_t vfsStatusCode,
	                               uint32_t stagedBytes, bool stagingUsesPsram) {
	// JSON-Antworten bleiben bewusst klein und maschinenlesbar, damit sowohl Browser
	// als auch einfache Skripte denselben Uploadpfad verwenden koennen.
	String body;
	body.reserve(320 + (path != nullptr ? strlen(path) : 0u));
	body += "{";
	body += "\"status\":\"";
	body += statusText;
	body += "\",\"detail\":\"";
	body += detail;
	body += "\",\"networkMode\":\"";
	body += networkModeName(networkMode_);
	body += "\",\"address\":\"";
	body += networkAddress_;
	body += "\",\"bytesWritten\":";
	body += String(bytesWritten);
	body += ",\"vfsStatusCode\":";
	body += String(vfsStatusCode);
	body += ",\"stagedBytes\":";
	body += String(stagedBytes);
	body += ",\"stagingStorage\":\"";
	body += stagingStorageName(stagingUsesPsram, stagedBytes);
	body += "\"";
	if (path != nullptr) {
		body += ",\"path\":\"";
		body += path;
		body += "\"";
	}
	body += "}";
	server_.send(statusCode, "application/json", body);
}

}  // namespace dukatimer