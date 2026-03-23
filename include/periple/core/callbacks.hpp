#pragma once

#include <cstddef>

namespace periple {

struct DefaultTourCost {};
struct DefaultCallbacks {};

template <typename CityT>
struct AppendMove {
	CityT city;
};

struct ConstructParams {
	std::size_t start_city = 0;
};

} // namespace periple
