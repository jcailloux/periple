#pragma once

// Nearest neighbor construction heuristic
//
// Rosenkrantz, Stearns, Lewis (1977), "An Analysis of Several Heuristics for the Traveling Salesman Problem"

#include <periple/algorithms/greedy_construct.hpp>
#include <periple/core/dispatch.hpp>

#include <limits>
#include <optional>
#include <span>

namespace periple {

// Strategy for greedy_construct: selects the nearest unvisited city.
//
// Without variant callbacks (3-arg): scores by distance.
// With variant callbacks (4-arg): uses move_filter and move_eval > dist.
struct NearestSelector {

	// --- Without variant callbacks ---

	template <DistanceSource Dist>
	auto select_next(const Dist& dist,
	                 std::span<const typename dist_traits<Dist>::city_type> tour,
	                 std::span<const uint8_t> visited) const
		-> std::optional<typename dist_traits<Dist>::city_type>
	{
		using city_type = typename dist_traits<Dist>::city_type;
		using cost_type = typename dist_traits<Dist>::cost_type;

		const auto n = visited.size();
		cost_type best_score = std::numeric_limits<cost_type>::max();
		city_type best_city{};
		bool found = false;

		for (std::size_t j = 0; j < n; ++j) {
			if (visited[j]) continue;
			auto candidate = static_cast<city_type>(j);
			auto s = dist(tour.back(), candidate);
			if (!found || s < best_score) {
				best_score = s;
				best_city = candidate;
				found = true;
			}
		}

		if (!found)
			return std::nullopt;
		return best_city;
	}

	// --- With variant callbacks ---

	template <DistanceSource Dist, typename Variant>
	auto select_next(const Dist& dist,
	                 std::span<const typename dist_traits<Dist>::city_type> tour,
	                 std::span<const uint8_t> visited,
	                 const Variant& variant) const
		-> std::optional<typename dist_traits<Dist>::city_type>
	{
		using city_type = typename dist_traits<Dist>::city_type;
		using score_type = decltype(score(dist, tour, city_type{}, variant));

		const auto n = visited.size();
		score_type best_score = std::numeric_limits<score_type>::max();
		city_type best_city{};
		bool found = false;

		for (std::size_t j = 0; j < n; ++j) {
			if (visited[j]) continue;
			auto candidate = static_cast<city_type>(j);

			if (!detail::dispatch_filter(variant, tour,
			                             AppendMove<city_type>{candidate}))
				continue;

			auto s = score(dist, tour, candidate, variant);
			if (!found || s < best_score) {
				best_score = s;
				best_city = candidate;
				found = true;
			}
		}

		if (!found)
			return std::nullopt;
		return best_city;
	}

private:
	template <DistanceSource Dist, typename Variant>
	static auto score(const Dist& dist,
	                  std::span<const typename dist_traits<Dist>::city_type> tour,
	                  typename dist_traits<Dist>::city_type candidate,
	                  const Variant& variant)
	{
		using city_type = typename dist_traits<Dist>::city_type;
		using cost_type = typename dist_traits<Dist>::cost_type;

		if constexpr (requires {
			{ variant.move_eval(tour, AppendMove<city_type>{candidate}) }
				-> std::convertible_to<cost_type>;
		}) {
			return variant.move_eval(tour, AppendMove<city_type>{candidate});
		} else {
			return dist(tour.back(), candidate);
		}
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
