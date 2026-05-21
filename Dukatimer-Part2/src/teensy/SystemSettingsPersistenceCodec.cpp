/*
 * SystemSettingsPersistenceCodec
 *
 * Diese Datei definiert das einzige gueltige Blob-Format fuer die globalen
 * Systemsettings. Damit bleiben Setup-Daten getrennt von Paper-Slots und werden
 * trotzdem versioniert und integritaetsgeprueft gespeichert.
 */

#include "SystemSettingsPersistenceCodec.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace dukatimer {

namespace {

constexpr float kMinimumDeratingStartCelsius = 30.0f;
constexpr float kMaximumHardStopCelsius = 90.0f;
constexpr float kMinimumThermalGapCelsius = 5.0f;
constexpr uint8_t kMinimumHeadBrightnessPercent = 10u;

uint16_t foldToCrc16(uint32_t hash) {
	return static_cast<uint16_t>(((hash >> 16u) ^ (hash & 0xFFFFu)) & 0xFFFFu);
}

void setParseError(SystemSettingsBlobParseError* outError, SystemSettingsBlobParseError error) {
	if (outError != nullptr) {
		*outError = error;
	}
}

}  // namespace

FLASHMEM uint16_t SystemSettingsPersistenceCodec::calculatePayloadCrc(const void* payload,
	                                                                 size_t payloadSizeBytes) {
	if (payload == nullptr || payloadSizeBytes == 0u) {
		return 0u;
	}

	const uint8_t* bytes = static_cast<const uint8_t*>(payload);
	uint32_t hash = 2166136261u;
	for (size_t index = 0u; index < payloadSizeBytes; ++index) {
		hash ^= bytes[index];
		hash *= 16777619u;
	}

	return foldToCrc16(hash);
}

FLASHMEM bool SystemSettingsPersistenceCodec::validateSettings(const SystemSettings& settings) {
	if (settings.schemaVersion != kSystemSettingsSchemaVersion) {
		return false;
	}

	if (static_cast<uint8_t>(settings.soundMode) > static_cast<uint8_t>(SoundFeedbackMode::Normal)) {
		return false;
	}

	if (static_cast<uint8_t>(settings.soundVolume) > static_cast<uint8_t>(SoundVolumeLevel::High)) {
		return false;
	}

	if (settings.vibrationEnabled > 1u) {
		return false;
	}

	if (settings.maxHeadBrightnessPercent < kMinimumHeadBrightnessPercent ||
	    settings.maxHeadBrightnessPercent > 100u) {
		return false;
	}

	const float deratingStart = settings.thermalProtection.deratingStartCelsius;
	const float hardStop = settings.thermalProtection.hardStopCelsius;
	if (!isfinite(deratingStart) || !isfinite(hardStop)) {
		return false;
	}

	if (deratingStart < kMinimumDeratingStartCelsius || hardStop > kMaximumHardStopCelsius) {
		return false;
	}

	if ((hardStop - deratingStart) < kMinimumThermalGapCelsius) {
		return false;
	}

	return true;
}

FLASHMEM void SystemSettingsPersistenceCodec::initializeDefaults(SystemSettings& settings) {
	settings = makeDefaultSystemSettings();
}

FLASHMEM bool SystemSettingsPersistenceCodec::buildBlob(const SystemSettings& settings,
	                                                    uint8_t* outBuffer,
	                                                    size_t outCapacity,
	                                                    size_t& outWrittenBytes) {
	outWrittenBytes = 0u;
	if (outBuffer == nullptr || outCapacity < blobSize() || !validateSettings(settings)) {
		return false;
	}

	SystemSettingsBlobHeader header = {};
	header.magic = kSystemSettingsBlobMagic;
	header.formatVersion = kSystemSettingsBlobFormatVersion;
	header.payloadSize = static_cast<uint16_t>(sizeof(SystemSettings));
	header.payloadCrc = calculatePayloadCrc(&settings, sizeof(SystemSettings));

	memcpy(outBuffer, &header, sizeof(SystemSettingsBlobHeader));
	memcpy(outBuffer + sizeof(SystemSettingsBlobHeader), &settings, sizeof(SystemSettings));
	outWrittenBytes = blobSize();
	return true;
}

FLASHMEM bool SystemSettingsPersistenceCodec::parseBlobWithError(const uint8_t* blob,
	                                                            size_t blobSizeBytes,
	                                                            SystemSettings& outSettings,
	                                                            SystemSettingsBlobParseError* outError) {
	setParseError(outError, SystemSettingsBlobParseError::None);
	if (blob == nullptr || blobSizeBytes < sizeof(SystemSettingsBlobHeader)) {
		setParseError(outError, SystemSettingsBlobParseError::InputTooSmall);
		return false;
	}

	SystemSettingsBlobHeader header = {};
	memcpy(&header, blob, sizeof(SystemSettingsBlobHeader));
	if (header.magic != kSystemSettingsBlobMagic) {
		setParseError(outError, SystemSettingsBlobParseError::HeaderMagicMismatch);
		return false;
	}

	if (header.formatVersion == 0u || header.formatVersion > kSystemSettingsBlobFormatVersion) {
		setParseError(outError, SystemSettingsBlobParseError::UnsupportedFormatVersion);
		return false;
	}

	if (header.payloadSize != sizeof(SystemSettings)) {
		setParseError(outError, SystemSettingsBlobParseError::PayloadSizeMismatch);
		return false;
	}

	const size_t requiredSize = sizeof(SystemSettingsBlobHeader) + static_cast<size_t>(header.payloadSize);
	if (blobSizeBytes != requiredSize) {
		setParseError(outError, SystemSettingsBlobParseError::BlobSizeMismatch);
		return false;
	}

	SystemSettings loadedSettings = {};
	memcpy(&loadedSettings, blob + sizeof(SystemSettingsBlobHeader), sizeof(SystemSettings));
	const uint16_t expectedCrc = calculatePayloadCrc(&loadedSettings, sizeof(SystemSettings));
	if (expectedCrc != header.payloadCrc) {
		setParseError(outError, SystemSettingsBlobParseError::CrcMismatch);
		return false;
	}

	if (!validateSettings(loadedSettings)) {
		setParseError(outError, SystemSettingsBlobParseError::InvalidSettings);
		return false;
	}

	outSettings = loadedSettings;
	return true;
}

FLASHMEM bool SystemSettingsPersistenceCodec::parseBlob(const uint8_t* blob,
	                                                   size_t blobSizeBytes,
	                                                   SystemSettings& outSettings) {
	return parseBlobWithError(blob, blobSizeBytes, outSettings, nullptr);
}

}  // namespace dukatimer