#pragma once

namespace dukatimer {

class HeadTimingDiagnosticsQueryPort {
public:
	virtual ~HeadTimingDiagnosticsQueryPort() = default;

	virtual bool headTimingDiagnosticsEnabled() const = 0;
};

}  // namespace dukatimer