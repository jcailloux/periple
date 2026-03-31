#pragma once

#include <cstddef>

namespace periple {

template <typename CityT>
struct TwoOptMove {
	std::size_t i; // last unchanged position before reversal
	std::size_t j; // last reversed position

	template <typename Tour>
	auto city_at(const Tour& tour) const {
		return [&tour, i = i, j = j](std::size_t pos) -> CityT {
			return (pos > i && pos <= j)
			    ? tour[i + j + 1 - pos]
			    : tour[pos];
		};
	}

	template <typename Position>
	auto position_of(const Position& position) const {
		return [&position, i = i, j = j](CityT city) -> std::size_t {
			auto pos = position[static_cast<std::size_t>(city)];
			return (pos > i && pos <= j) ? i + j + 1 - pos : pos;
		};
	}

	std::size_t from_pos() const { return i + 1; }
};

} // namespace periple