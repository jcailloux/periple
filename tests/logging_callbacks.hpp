#pragma once

#include <periple/core/moves.hpp>

#include <span>
#include <string>
#include <vector>

namespace periple {

// Utility for protocol tests: records every callback invocation.
struct LoggingCallbacks {
	mutable std::vector<std::string> log;

	template <typename CityT, typename CostT>
	bool move_filter(const AppendMove<CityT, CostT>&) const {
		log.push_back("move_filter");
		return true;
	}

	template <typename CityT, typename CostT>
	void on_move(const AppendMove<CityT, CostT>&) const {
		log.push_back("on_move");
	}
};

} // namespace periple
