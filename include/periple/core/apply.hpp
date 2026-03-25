#pragma once

#include <periple/core/dispatch.hpp>
#include <periple/core/moves.hpp>
#include <periple/core/traits.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

namespace periple::detail {

// Applies an AppendMove: places the city in the tour, marks it as visited,
// accumulates the open-path cost, and notifies the variant via on_move.
// Returns the updated path cost.
// Shared by all constructive algorithms (NN, greedy, future cheapest insertion, etc.).
template <DistanceSource Dist, typename Variant>
auto apply_append(
	const Dist& dist,
	std::span<typename dist_traits<Dist>::city_type> tour,
	std::size_t step,
	std::span<uint8_t> visited,
	typename dist_traits<Dist>::city_type city,
	typename dist_traits<Dist>::cost_type path_cost,
	const Variant& variant)
	-> typename dist_traits<Dist>::cost_type
{
	using city_type = typename dist_traits<Dist>::city_type;

	// Accumulate edge cost for the open path.
	if (step > 0)
		path_cost += dist(tour[step - 1], city);

	// Place and mark visited.
	tour[step] = city;
	visited[static_cast<std::size_t>(city)] = 1;

	// Notify variant.
	auto placed_tour = std::span<const city_type>(tour.data(), step + 1);
	dispatch_on_move(variant, placed_tour, AppendMove<city_type>{city});

	return path_cost;
}

} // namespace periple::detail