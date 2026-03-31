#pragma once

#include <cstddef>

namespace periple {

template <typename CityT>
struct DPMove {
	CityT from;
	CityT to;
	std::size_t set; // visited cities bitmask (includes from, excludes to)
};

} // namespace periple