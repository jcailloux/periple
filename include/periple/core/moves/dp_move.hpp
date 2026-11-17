#pragma once

#include <cstddef>
#include <limits>
#include <type_traits>

namespace periple {

template <typename CityT>
struct DPMove {
	static constexpr std::size_t no_state = std::numeric_limits<std::size_t>::max();

	CityT from;
	CityT to;
	std::size_t set; // visited cities bitmask (includes from, excludes to)

	// Dimension storage indices, assigned by the algorithm. to_state is set only
	// once the move improves its target, so it is valid in commit alone.
	std::size_t from_state = 0;
	std::size_t to_state = no_state;
};

namespace detail {

// A variant under which the cheapest of the states that share a (set, city)
// is never worse for the rest of the tour, so held_karp() keeps that one alone.
// The shipped variants that qualify specialize it.
template <typename V>
struct cheapest_state_suffices : std::false_type {};

// A variant under which every tour must reach each city by a due time, and
// nothing makes a time earlier: held_karp() then drops a state from which an
// unvisited city, or the return to city 0, can no longer be reached in time.
// The specialization gives due_times(v, out), one per city, and departure(ctx),
// the departure from the city just entered. Time windows specialize it under a
// single Strict.
template <typename V>
struct dp_deadlines : std::false_type {};

} // namespace detail

} // namespace periple
