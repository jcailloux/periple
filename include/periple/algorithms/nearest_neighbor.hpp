#pragma once

#include <periple/core/solver.hpp>

#include <algorithm>
#include <limits>
#include <span>

namespace periple {
namespace detail {

template <DistanceSource Dist>
auto nearest_neighbor_build(
	const Dist& dist,
	const std::size_t n,
	const typename dist_traits<Dist>::city_type start,
	std::span<typename dist_traits<Dist>::city_type> tour,
	std::span<uint8_t> visited
) -> typename dist_traits<Dist>::cost_type
{
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	if (n < 2) {
		if (n == 1) tour[0] = start;
		return cost_type{};
	}

	std::ranges::fill(visited.first(n), uint8_t{0});

	tour[0] = start;
	visited[static_cast<std::size_t>(start)] = 1;
	cost_type total_cost{};

	for (std::size_t step = 1; step < n; ++step) {
		city_type current = tour[step - 1];
		cost_type best_cost = std::numeric_limits<cost_type>::max();
		city_type best_city{};

		for (std::size_t j = 0; j < n; ++j) {
			if (visited[j]) continue;
			cost_type c = dist(current, static_cast<city_type>(j));
			if (c < best_cost) {
				best_cost = c;
				best_city = static_cast<city_type>(j);
			}
		}

		tour[step] = best_city;
		visited[static_cast<std::size_t>(best_city)] = 1;
		total_cost += best_cost;
	}

	total_cost += dist(tour[n - 1], start);
	return total_cost;
}

} // namespace detail

template <DistanceSource Dist>
inline auto Solver<Dist>::nearest_neighbor(NearestNeighborParams params)
	-> Solver&
{
	n_ = dist_->size();
	ensure_shared(n_);
	cost_ = detail::nearest_neighbor_build(
		*dist_, n_,
		static_cast<city_type>(params.start_city),
		std::span<city_type>(tour_.data(), n_),
		std::span<uint8_t>(visited_.data(), n_));
	status_ = SolutionStatus::feasible;
	rebuild_position();
	return *this;
}

} // namespace periple
