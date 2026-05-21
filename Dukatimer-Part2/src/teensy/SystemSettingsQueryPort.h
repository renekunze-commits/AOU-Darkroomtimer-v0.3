#pragma once

#include <stdint.h>

#include "SystemSettings.h"

namespace dukatimer {

class SystemSettingsQueryPort {
public:
	virtual ~SystemSettingsQueryPort() = default;

	virtual const SystemSettings& currentSettings() const = 0;
	virtual uint8_t storageErrorCode() const = 0;
	virtual uint32_t storageErrorDetail() const = 0;
};

}  // namespace dukatimer