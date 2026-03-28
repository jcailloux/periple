#pragma once

#include <periple/core/moves.hpp>

#include <string>
#include <vector>

namespace periple {

// Utility for protocol tests: records every callback invocation.
struct LoggingCallbacks {
	mutable std::vector<std::string> log;

	template <typename CityT>
	bool move_filter(const AppendMove<CityT>&) const {
		log.push_back("move_filter");
		return true;
	}

	template <typename CityT>
	void move_prepare(const AppendMove<CityT>&) const {
		log.push_back("move_prepare");
	}
};

} // namespace periple