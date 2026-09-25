#pragma once

#include <periple/core/moves/append_move.hpp>
#include <periple/core/moves/dp_move.hpp>

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

	template <typename CityT>
	bool move_filter(const DPMove<CityT>&) const {
		log.push_back("dp_move_filter");
		return true;
	}

	template <typename CityT>
	void move_prepare(const DPMove<CityT>&) const {
		log.push_back("dp_move_prepare");
	}
};

} // namespace periple