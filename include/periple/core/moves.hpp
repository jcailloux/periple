#pragma once

#include <cstddef>

namespace periple {

template <typename CityT>
struct AppendMove {
	CityT city;
};

template <typename CityT, typename CostT>
struct DPMove {
	CityT from;
	CityT to;
	CostT cost;          // cumulative DP cost at 'from' (after move_eval adjustments)
	CostT distance;      // raw dist(from, to), before move_eval
	std::size_t set;     // visited cities bitmask (includes from, excludes to)
};

} // namespace periple
