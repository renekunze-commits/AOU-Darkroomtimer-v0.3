/*
 * PaperSlotPersistenceCodec
 *
 * Diese Datei besitzt das zentrale Binärformat der Paper-Slot-Daten. Genau hier
 * werden Defaultprofile aufgebaut, Header und Integritaetspruefung geschrieben
 * und eingelesene Bloecke gegen die erwartete Bankschema-Struktur validiert.
 */

#include "PaperSlotPersistenceCodec.h"

#include <Arduino.h>
#include <string.h>

namespace dukatimer {

namespace {

// Der Hash wird auf 16 Bit gefaltet, damit der Blob mit kleinem Header eine
// leichte Integritaetspruefung erhaelt, ohne auf schwerere CRC-Infrastruktur
// angewiesen zu sein.
uint16_t foldToCrc16(uint32_t hash) {
	return static_cast<uint16_t>(((hash >> 16u) ^ (hash & 0xFFFFu)) & 0xFFFFu);
}

void copyName(char* destination, size_t capacity, const char* source) {
	if (destination == nullptr || capacity == 0u) {
		return;
	}

	snprintf(destination, capacity, "%s", source != nullptr ? source : "");
}

void formatEmptySlotName(char* destination, size_t capacity, uint8_t slotIndex) {
	if (destination == nullptr || capacity == 0u) {
		return;
	}

	snprintf(destination, capacity, "Leer %u", static_cast<unsigned>(slotIndex));
}

void setParseError(PaperSlotBlobParseError* outError, PaperSlotBlobParseError error) {
	if (outError != nullptr) {
		*outError = error;
	}
}

}  // namespace

FLASHMEM uint16_t PaperSlotPersistenceCodec::calculatePayloadCrc(const void* payload,
																 size_t payloadSizeBytes) {
	if (payload == nullptr || payloadSizeBytes == 0) {
		return 0;
	}

	const uint8_t* bytes = static_cast<const uint8_t*>(payload);
	uint32_t hash = 2166136261u;
	for (size_t index = 0; index < payloadSizeBytes; ++index) {
		hash ^= bytes[index];
		hash *= 16777619u;
	}

	return foldToCrc16(hash);
}

FLASHMEM bool PaperSlotPersistenceCodec::validateBank(const PaperSlotBank& bank) {
	// validateBank() prueft nur die strukturelle Gueltigkeit des Bankobjekts.
	// Fachliche Kalibrierungsqualitaet wird hier bewusst nicht bewertet.
	if (bank.schemaVersion != kPaperSlotBankSchemaVersion) {
		return false;
	}

	if (bank.slotCount != kPaperSlotCount) {
		return false;
	}

	if (bank.activeSlot >= kPaperSlotCount) {
		return false;
	}

	for (uint8_t slot = 0; slot < kPaperSlotCount; ++slot) {
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

FLASHMEM void PaperSlotPersistenceCodec::initializeDefaultBank(PaperSlotBank& bank) {
	// Die Defaultbank sorgt dafuer, dass das System auch ohne gespeicherte Datei
	// mit konsistenten, voll belegten Profilslots starten kann.
	bank = {};
	bank.schemaVersion = kPaperSlotBankSchemaVersion;
	bank.activeSlot = 1;
	bank.slotCount = kPaperSlotCount;

	for (uint8_t slot = 0; slot < kPaperSlotCount; ++slot) {
		PaperExposureProfile profile = {};
		profile.schemaVersion = kPaperExposureProfileSchemaVersion;
		formatEmptySlotName(profile.name, sizeof(profile.name), slot);
		profile.calibrated = false;
		profile.gradeMode = PaperGradeMode::Multigrade;
		profile.useIsoMath = false;
		profile.fixedGradeValue = 2.5f;
		profile.isoP = 100.0f;
		profile.isoR = 100.0f;
		profile.kBw = 10.0f;
		profile.kSoft = 10.0f;
		profile.kHard = 10.0f;
		profile.preflash.calibrated = false;
		profile.preflash.enabled = false;
		profile.preflash.thresholdSeconds = 0.0f;
		profile.preflash.factor = 1.0f;
		profile.preflash.level = 0;
		profile.preflash.colorMode = 0;

		for (uint8_t grade = 0; grade < kSplitgradeStepCount; ++grade) {
			const float hardFactor = static_cast<float>(grade) / 10.0f;
			profile.gradeKSoft[grade] = 10.0f * (1.0f - hardFactor);
			profile.gradeKHard[grade] = 10.0f * hardFactor;
		}

		bank.slots[slot] = profile;
	}

	PaperExposureProfile& ilford = bank.slots[1];
	copyName(ilford.name, sizeof(ilford.name), "Ilford MGIV RC");
	ilford.calibrated = true;
	ilford.kBw = 15.0f;
	ilford.kSoft = 12.0f;
	ilford.kHard = 12.0f;
	for (uint8_t grade = 0; grade < kSplitgradeStepCount; ++grade) {
		const float hardFactor = static_cast<float>(grade) / 10.0f;
		ilford.gradeKSoft[grade] = ilford.kSoft * (1.0f - hardFactor);
		ilford.gradeKHard[grade] = ilford.kHard * hardFactor;
	}
}

FLASHMEM bool PaperSlotPersistenceCodec::buildBlob(const PaperSlotBank& bank,
												   uint8_t* outBuffer,
												   size_t outCapacity,
												   size_t& outWrittenBytes) {
	// buildBlob() serialisiert immer die vollstaendige Bank samt Header. Teil- oder
	// Feldweise-Persistenz ist absichtlich nicht Teil dieses Formats.
	outWrittenBytes = 0;
	if (outBuffer == nullptr) {
		return false;
	}

	if (!validateBank(bank)) {
		return false;
	}

	const size_t requiredSize = blobSize();
	if (outCapacity < requiredSize) {
		return false;
	}

	PaperSlotBlobHeader header;
	header.magic = kPaperSlotBlobMagic;
	header.formatVersion = kPaperSlotBlobFormatVersion;
	header.payloadSize = static_cast<uint16_t>(sizeof(PaperSlotBank));
	header.payloadCrc = calculatePayloadCrc(&bank, sizeof(PaperSlotBank));
	header.reserved = 0;

	memcpy(outBuffer, &header, sizeof(PaperSlotBlobHeader));
	memcpy(outBuffer + sizeof(PaperSlotBlobHeader), &bank, sizeof(PaperSlotBank));
	outWrittenBytes = requiredSize;
	return true;
}

FLASHMEM bool PaperSlotPersistenceCodec::parseBlobWithError(const uint8_t* blob,
													 size_t blobSizeBytes,
													 PaperSlotBank& outBank,
													 PaperSlotBlobParseError* outError) {
	// parseBlob() akzeptiert nur Bloecke, deren Header, Groesse, Integritaetswert
	// und Bankschema gleichzeitig stimmen. Erst dann wird die Bank sichtbar.
	setParseError(outError, PaperSlotBlobParseError::None);

	if (blob == nullptr || blobSizeBytes < sizeof(PaperSlotBlobHeader)) {
		setParseError(outError, PaperSlotBlobParseError::InputTooSmall);
		return false;
	}

	PaperSlotBlobHeader header;
	memcpy(&header, blob, sizeof(PaperSlotBlobHeader));

	if (header.magic != kPaperSlotBlobMagic) {
		setParseError(outError, PaperSlotBlobParseError::HeaderMagicMismatch);
		return false;
	}

	if (header.formatVersion == 0u || header.formatVersion > kPaperSlotBlobFormatVersion) {
		setParseError(outError, PaperSlotBlobParseError::UnsupportedFormatVersion);
		return false;
	}

	if (header.payloadSize != sizeof(PaperSlotBank)) {
		setParseError(outError, PaperSlotBlobParseError::PayloadSizeMismatch);
		return false;
	}

	const size_t requiredSize = sizeof(PaperSlotBlobHeader) + static_cast<size_t>(header.payloadSize);
	if (blobSizeBytes < requiredSize) {
		setParseError(outError, PaperSlotBlobParseError::BlobSizeMismatch);
		return false;
	}

	PaperSlotBank loadedBank;
	memcpy(&loadedBank, blob + sizeof(PaperSlotBlobHeader), sizeof(PaperSlotBank));

	const uint16_t expectedCrc = calculatePayloadCrc(&loadedBank, sizeof(PaperSlotBank));
	if (expectedCrc != header.payloadCrc) {
		setParseError(outError, PaperSlotBlobParseError::CrcMismatch);
		return false;
	}

	if (!validateBank(loadedBank)) {
		setParseError(outError, PaperSlotBlobParseError::InvalidBank);
		return false;
	}

	outBank = loadedBank;
	setParseError(outError, PaperSlotBlobParseError::None);
	return true;
}

FLASHMEM bool PaperSlotPersistenceCodec::parseBlob(const uint8_t* blob,
											   size_t blobSizeBytes,
											   PaperSlotBank& outBank) {
	return parseBlobWithError(blob, blobSizeBytes, outBank, nullptr);
}

}  // namespace dukatimer
