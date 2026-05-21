#pragma once

namespace dukatimer {

class HeadTimingDiagnosticsCommandPort {
public:
	virtual ~HeadTimingDiagnosticsCommandPort() = default;

	virtual void setHeadTimingDiagnosticsEnabled(bool enabled) = 0;
};

}  // namespace dukatimer