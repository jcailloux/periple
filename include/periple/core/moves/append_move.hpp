#pragma once

#include <cstddef>

namespace periple {

template <typename CityT>
struct AppendMove {
	CityT city;
	CityT prev_city;
	std::size_t pos;

	AppendMove(CityT c, CityT prev, std::size_t p) : city(c), prev_city(prev), pos(p) {}
};

} // namespace periple