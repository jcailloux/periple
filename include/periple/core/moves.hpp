#pragma once

#include <cstddef>
#include <span>

namespace periple {

template <typename CityT, typename CostT>
struct AppendMove {
	CityT city;
	std::span<const CityT> tour;  // partial tour before this append
	CostT cost;                   // accumulated path cost before this append
};

template <typename CityT, typename CostT>
struct DPMove {
	CityT from;
	CityT to;
	CostT cost;          // accumulated DP cost before this transition
	std::size_t set;     // visited cities bitmask (includes from, excludes to)
};

} // namespace periple
