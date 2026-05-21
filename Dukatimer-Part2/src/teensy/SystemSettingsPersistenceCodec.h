#pragma once

#include <stddef.h>
#include <stdint.h>

#include "SystemSettings.h"

namespace dukatimer {

constexpr uint32_t kSystemSettingsBlobMagic = 0x44555354u;
constexpr uint16_t kSystemSettingsBlobFormatVersion = 1;

struct SystemSettingsBlobHeader {
	uint32_t magic = kSystemSettingsBlobMagic;
	uint16_t formatVersion = kSystemSettingsBlobFormatVersion;
	uint16_t payloadSize = 0;
	uint16_t payloadCrc = 0;
	uint16_t reserved = 0;
};

static_assert(sizeof(SystemSettingsBlobHeader) == 12, "Unexpected SystemSettingsBlobHeader size");

enum class SystemSettingsBlobParseError : uint8_t {
	None,
	InputTooSmall,
	HeaderMagicMismatch,
	UnsupportedFormatVersion,
	PayloadSizeMismatch,
	BlobSizeMismatch,
	CrcMismatch,
	InvalidSettings,
};

class SystemSettingsPersistenceCodec {
public:
	static constexpr size_t payloadSize() {
		return sizeof(SystemSettings);
	}

	static constexpr size_t blobSize() {
		return sizeof(SystemSettingsBlobHeader) + sizeof(SystemSettings);
	}

	static uint16_t calculatePayloadCrc(const void* payload, size_t payloadSizeBytes);
	static bool validateSettings(const SystemSettings& settings);
	static void initializeDefaults(SystemSettings& settings);
	static bool buildBlob(const SystemSettings& settings,
	                     uint8_t* outBuffer,
	                     size_t outCapacity,
	                     size_t& outWrittenBytes);
	static bool parseBlobWithError(const uint8_t* blob,
	                             size_t blobSizeBytes,
	                             SystemSettings& outSettings,
	                             SystemSettingsBlobParseError* outError);
	static bool parseBlob(const uint8_t* blob, size_t blobSizeBytes, SystemSettings& outSettings);
};

}  // namespace dukatimer