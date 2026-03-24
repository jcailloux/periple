#pragma once

// Nearest neighbor construction heuristic
//
// Rosenkrantz, Stearns, Lewis (1977), "An Analysis of Several Heuristics for the Traveling Salesman Problem"

#include <periple/algorithms/greedy_construct.hpp>

#include <span>

namespace periple {

// Selects the candidate that minimizes the metric (move_score > move_eval > dist).
struct NearestSelector {
	// Return type is deduced from the active callback,
	// so there is no conversion overhead when no callbacks are used.
	template <DistanceSource Dist, typename Callbacks>
	auto evaluate(const Dist& dist,
	              std::span<const typename dist_traits<Dist>::city_type> tour,
	              typename dist_traits<Dist>::city_type candidate,
	              const Callbacks& cb) const
	{
		using city_type = typename dist_traits<Dist>::city_type;

		if constexpr (requires(const Callbacks& c, std::span<const city_type> t,
		                       const AppendMove<city_type>& am) {
			c.move_score(t, am);
		}) {
			return cb.move_score(tour, AppendMove<city_type>{candidate});
		} else if constexpr (requires(const Callbacks& c, std::span<const city_type> t,
		                              const AppendMove<city_type>& am) {
			c.move_eval(t, am);
		}) {
			return cb.move_eval(tour, AppendMove<city_type>{candidate});
		} else {
			return dist(tour.back(), candidate);
		}
	}
};

// ---------------------------------------------------------------------------
// Solver::nearest_neighbor -- facade using NearestSelector
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename TourCost>
template <typename Callbacks>
auto Solver<Dist, TourCost>::nearest_neighbor(
	NearestNeighborParams params, const Callbacks& cb)
	-> Solver&
{
	return greedy_construct(NearestSelector{}, cb,
	                        ConstructParams{.start_city = params.start_city});
}

} // namespace periple
