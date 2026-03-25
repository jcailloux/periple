#pragma once

// Nearest neighbor construction heuristic
//
// Rosenkrantz, Stearns, Lewis (1977), "An Analysis of Several Heuristics for the Traveling Salesman Problem"

#include <periple/algorithms/greedy_construct.hpp>

#include <limits>
#include <optional>
#include <span>

namespace periple {

// Strategy for greedy_construct: selects the nearest unvisited city.
//
// Without variant callbacks (3-arg): scores by distance.
// With variant callbacks (4-arg): uses move_filter, move_score > move_eval > dist,
// and delegates on_commit via on_placed.
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

			if constexpr (requires {
				{ variant.move_filter(tour, AppendMove<city_type>{candidate}) }
					-> std::convertible_to<bool>;
			}) {
				if (!variant.move_filter(tour, AppendMove<city_type>{candidate}))
					continue;
			}

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

	template <DistanceSource Dist, typename Variant>
	void on_placed(const Dist&,
	               std::span<const typename dist_traits<Dist>::city_type> tour,
	               std::span<const uint8_t>,
	               const Variant& variant) const
	{
		using city_type = typename dist_traits<Dist>::city_type;
		if constexpr (requires {
			variant.on_commit(tour, AppendMove<city_type>{tour.back()});
		}) {
			variant.on_commit(tour, AppendMove<city_type>{tour.back()});
		}
	}

private:
	template <DistanceSource Dist, typename Variant>
	static auto score(const Dist& dist,
	                  std::span<const typename dist_traits<Dist>::city_type> tour,
	                  typename dist_traits<Dist>::city_type candidate,
	                  const Variant& variant)
	{
		using city_type = typename dist_traits<Dist>::city_type;

		if constexpr (requires {
			variant.move_score(tour, AppendMove<city_type>{candidate});
		}) {
			return variant.move_score(tour, AppendMove<city_type>{candidate});
		} else if constexpr (requires {
			variant.move_eval(tour, AppendMove<city_type>{candidate});
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

// With variant callbacks (primary implementation).
template <DistanceSource Dist, typename TourCost>
template <typename Variant>
auto Solver<Dist, TourCost>::nearest_neighbor(
	const Variant& variant, NearestNeighborParams params)
	-> Solver&
{
	return greedy_construct(NearestSelector{}, variant,
	                        ConstructParams{.start_city = params.start_city});
}

// Without variant callbacks (forwards to primary).
template <DistanceSource Dist, typename TourCost>
auto Solver<Dist, TourCost>::nearest_neighbor(
	NearestNeighborParams params)
	-> Solver&
{
	return nearest_neighbor(NoCallbacks{}, params);
}

} // namespace periple