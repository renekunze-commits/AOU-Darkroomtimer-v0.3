#pragma once

#include "SystemSettings.h"

namespace dukatimer {

class SystemSettingsCommandPort {
public:
	virtual ~SystemSettingsCommandPort() = default;

	virtual bool saveSettings(const SystemSettings& settings) = 0;
};

}  // namespace dukatimer