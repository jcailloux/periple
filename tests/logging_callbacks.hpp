#pragma once

#include <periple/core/moves.hpp>

#include <span>
#include <string>
#include <vector>

namespace periple {

// Utility for protocol tests: records every callback invocation.
struct LoggingCallbacks {
	mutable std::vector<std::string> log;

	template <typename CityT>
	bool move_filter(std::span<const CityT>, const AppendMove<CityT>&) const {
		log.push_back("move_filter");
		return true;
	}

	template <typename CityT>
	void on_move(std::span<const CityT>, const AppendMove<CityT>&) const {
		log.push_back("on_move");
	}
};

} // namespace periple
