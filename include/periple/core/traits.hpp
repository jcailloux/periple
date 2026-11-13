#pragma once

#include <concepts>
#include <cstddef>
#include <type_traits>

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

// A source declares symmetry, d(i, j) == d(j, i) for every pair, with
// symmetric(): static constexpr when its type guarantees it, a member when its
// data decide. Without one it is asymmetric.

// Tag declaring a DistanceMatrix symmetric: DistanceMatrix m(n, data, periple::symmetric).
struct symmetric_t {};
inline constexpr symmetric_t symmetric{};

namespace detail {

// symmetric() is a constant expression: the type decides, whatever the data.
template <typename Dist>
concept static_symmetry = requires { typename std::bool_constant<Dist::symmetric()>; };

template <typename Dist>
concept symmetric_by_type = static_symmetry<Dist> && Dist::symmetric();

template <typename Dist>
constexpr bool declares_symmetry(const Dist& dist) {
	if constexpr (requires { { dist.symmetric() } -> std::convertible_to<bool>; })
		return dist.symmetric();
	else
		return false;
}

} // namespace detail

} // namespace periple
