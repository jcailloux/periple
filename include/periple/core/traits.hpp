#pragma once

#include <concepts>
#include <cstddef>

namespace periple {

// Extracts associated types from a distance source.
// The default definition pulls nested typedefs from Dist itself.
// Specialize this struct to adapt third-party types that do not
// expose cost_type / city_type as nested members.
template <typename Dist>
struct dist_traits {
	using cost_type = typename Dist::cost_type;
	using city_type = typename Dist::city_type;
};

// A DistanceSource provides pairwise costs between cities.
//   d(i, j) — cost of travelling from city i to city j.
//   d.size() — number of cities.
template <typename Dist>
concept DistanceSource = requires(
	const Dist& d,
	typename dist_traits<Dist>::city_type i)
{
	{ d(i, i) } -> std::convertible_to<typename dist_traits<Dist>::cost_type>;
	{ d.size() } -> std::convertible_to<std::size_t>;
};

} // namespace periple
