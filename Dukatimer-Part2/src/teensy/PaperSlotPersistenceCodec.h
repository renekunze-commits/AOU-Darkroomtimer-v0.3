#pragma once

#include <stddef.h>
#include <stdint.h>

#include "PaperSlotBank.h"

namespace dukatimer {

constexpr uint32_t kPaperSlotBlobMagic = 0x44555042u;
constexpr uint16_t kPaperSlotBlobFormatVersion = 1;

// Der Blob-Header macht die rohe Bank-Persistenz selbstbeschreibend: Magic,
// Formatversion, Payloadgroesse und einfache Integritaetspruefung begleiten die
// eigentlichen PaperSlotBank-Daten in jeder gespeicherten Datei.
struct PaperSlotBlobHeader {
	uint32_t magic = kPaperSlotBlobMagic;
	uint16_t formatVersion = kPaperSlotBlobFormatVersion;
	uint16_t payloadSize = 0;
	uint16_t payloadCrc = 0;
	uint16_t reserved = 0;
};

static_assert(sizeof(PaperSlotBlobHeader) == 12, "Unexpected PaperSlotBlobHeader size");

enum class PaperSlotBlobParseError : uint8_t {
	None,
	InputTooSmall,
	HeaderMagicMismatch,
	UnsupportedFormatVersion,
	PayloadSizeMismatch,
	BlobSizeMismatch,
	CrcMismatch,
	InvalidBank,
};

/*
 * PaperSlotPersistenceCodec
 *
 * Zweck:
 * - definiert das einzige gueltige Binärformat fuer die PaperSlotBank
 * - validiert Bankinhalte, erzeugt Defaultdaten und serialisiert/parst den Blob
 * - entkoppelt damit die inhaltliche Struktur der Profile vom SD-Dateizugriff
 */
class PaperSlotPersistenceCodec {
public:
	static constexpr size_t payloadSize() {
		return sizeof(PaperSlotBank);
	}

	static constexpr size_t blobSize() {
		return sizeof(PaperSlotBlobHeader) + sizeof(PaperSlotBank);
	}

	static uint16_t calculatePayloadCrc(const void* payload, size_t payloadSizeBytes);
	static bool validateBank(const PaperSlotBank& bank);
	static void initializeDefaultBank(PaperSlotBank& bank);
	static bool buildBlob(const PaperSlotBank& bank,
	                     uint8_t* outBuffer,
	                     size_t outCapacity,
	                     size_t& outWrittenBytes);
	static bool parseBlobWithError(const uint8_t* blob,
	                             size_t blobSizeBytes,
	                             PaperSlotBank& outBank,
	                             PaperSlotBlobParseError* outError);
	static bool parseBlob(const uint8_t* blob, size_t blobSizeBytes, PaperSlotBank& outBank);
};

}  // namespace dukatimer
