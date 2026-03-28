#pragma once

// Nearest neighbor construction heuristic
//
// Rosenkrantz, Stearns, Lewis (1977), "An Analysis of Several Heuristics for the Traveling Salesman Problem"

#include <periple/algorithms/greedy_construct.hpp>

#include <limits>
#include <optional>

namespace periple {

// Strategy for greedy_construct: selects the nearest unvisited city.
// Uses solver.evaluate() for filtering and scoring (variant-aware).
struct NearestSelector {

	template <DistanceSource Dist, typename Variant>
	auto select_next(const Solver<Dist, Variant>& solver) const
		-> std::optional<typename dist_traits<Dist>::city_type>
	{
		using city_type = typename dist_traits<Dist>::city_type;

		if (solver.tour().empty()) return std::nullopt;

		double best_score = std::numeric_limits<double>::max();
		city_type best_city{};
		bool found = false;

		for (std::size_t j = 0; j < solver.size(); ++j) {
			auto candidate = static_cast<city_type>(j);
			if (solver.is_visited(candidate)) continue;

			auto result = solver.evaluate(candidate);
			if (!result) continue;

			if (!found || *result < best_score) {
				best_score = *result;
				best_city = candidate;
				found = true;
			}
		}

		if (!found) return std::nullopt;
		return best_city;
	}
};

// ---------------------------------------------------------------------------
// Solver::nearest_neighbor -- facade using NearestSelector
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::nearest_neighbor(NearestNeighborParams params) -> Solver& {
	return greedy_construct(NearestSelector{}, ConstructParams{.start_city = params.start_city, .resume_at = params.resume_at});
}

} // namespace periple