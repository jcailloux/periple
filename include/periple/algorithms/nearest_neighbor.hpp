#pragma once

// Nearest neighbor construction heuristic
//
// Rosenkrantz, Stearns, Lewis (1977), "An Analysis of Several Heuristics for the Traveling Salesman Problem"

#include <periple/algorithms/greedy_construct.hpp>

#include <limits>
#include <optional>

namespace periple {

// Strategy for greedy_construct: selects the nearest unvisited city.
// Uses solver.can_append for filtering and the variant's move_eval for
// scoring when available, otherwise raw distance.
struct NearestSelector {

	template <DistanceSource Dist, typename Variant>
	auto select_next(const Solver<Dist, Variant>& solver) const
		-> std::optional<typename dist_traits<Dist>::city_type>
	{
		using city_type = typename dist_traits<Dist>::city_type;
		using score_type = decltype(score(solver, city_type{}));

		auto tour = solver.tour();
		if (tour.empty()) return std::nullopt;

		score_type best_score = std::numeric_limits<score_type>::max();
		city_type best_city{};
		bool found = false;

		for (std::size_t j = 0; j < solver.size(); ++j) {
			auto candidate = static_cast<city_type>(j);
			if (solver.is_visited(candidate)) continue;
			if (!solver.can_append(candidate)) continue;

			auto s = score(solver, candidate);
			if (!found || s < best_score) {
				best_score = s;
				best_city = candidate;
				found = true;
			}
		}

		if (!found) return std::nullopt;
		return best_city;
	}

private:
	template <DistanceSource Dist, typename Variant>
	static auto score(const Solver<Dist, Variant>& solver, typename dist_traits<Dist>::city_type candidate) {
		using city_type = typename dist_traits<Dist>::city_type;
		using cost_type = typename dist_traits<Dist>::cost_type;

		auto tour = solver.tour();
		auto raw = solver.distance(tour.back(), candidate);
		AppendMove<city_type, cost_type> move{candidate, tour, solver.cost()};
		return solver.eval(raw, move);
	}
};

// ---------------------------------------------------------------------------
// Solver::nearest_neighbor -- facade using NearestSelector
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::nearest_neighbor(NearestNeighborParams params)
	-> Solver&
{
	return greedy_construct(NearestSelector{},
	                        ConstructParams{.start_city = params.start_city,
	                                        .resume_at = params.resume_at});
}

} // namespace periple
