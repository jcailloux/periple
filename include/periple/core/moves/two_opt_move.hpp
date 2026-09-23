#pragma once

#include <cstddef>

namespace periple {

// Coordinate change for a segment reversal: positions i+1..j read backwards,
// every other position unchanged, without touching the tour. It feeds the
// replay that scores a candidate reversal (Solver::evaluate_reversal); should
// dimensions ever score a reversal in O(1), this is the move they would take.
template <typename CityT>
struct TwoOptMove {
	std::size_t i; // last unchanged position before the reversal
	std::size_t j; // last reversed position

	template <typename Tour>
	auto city_at(const Tour& tour) const {
		return [&tour, i = i, j = j](std::size_t pos) -> CityT {
			return (pos > i && pos <= j)
			    ? tour[i + j + 1 - pos]
			    : tour[pos];
		};
	}
};

} // namespace periple
