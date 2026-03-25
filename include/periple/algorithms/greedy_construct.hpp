#pragma once

// Greedy append-based construction
//
// Generic loop that builds a tour by appending one city at a time.
// The Strategy decides which city to append next; the framework only
// manages the tour buffer, visited flags, and termination.
//
// Variant callbacks are forwarded to the strategy's methods when they
// accept a callbacks parameter (4-arg). Otherwise the 3-arg form is called.

#include <periple/core/solver.hpp>

#include <optional>
#include <span>

namespace periple {
namespace detail {

// Calls strategy.select_next with or without variant callbacks,
// depending on what the strategy accepts.
template <DistanceSource Dist, typename Strategy, typename Variant>
auto call_select_next(
	Strategy& strategy, const Dist& dist,
	std::span<const typename dist_traits<Dist>::city_type> tour,
	std::span<const uint8_t> visited,
	const Variant& variant)
{
	if constexpr (requires {
		strategy.select_next(dist, tour, visited, variant);
	}) {
		return strategy.select_next(dist, tour, visited, variant);
	} else {
		return strategy.select_next(dist, tour, visited);
	}
}

// Calls strategy.on_placed with or without variant callbacks,
// if the method exists at all.
template <DistanceSource Dist, typename Strategy, typename Variant>
void call_on_placed(
	Strategy& strategy, const Dist& dist,
	std::span<const typename dist_traits<Dist>::city_type> tour,
	std::span<const uint8_t> visited,
	const Variant& variant)
{
	if constexpr (requires {
		strategy.on_placed(dist, tour, visited, variant);
	}) {
		strategy.on_placed(dist, tour, visited, variant);
	} else if constexpr (requires {
		strategy.on_placed(dist, tour, visited);
	}) {
		strategy.on_placed(dist, tour, visited);
	}
}

template <DistanceSource Dist, typename Strategy, typename Variant>
auto greedy_append_build(
	const Dist& dist, std::size_t n, std::size_t start_step,
	std::span<typename dist_traits<Dist>::city_type> tour,
	std::span<uint8_t> visited,
	Strategy& strategy, const Variant& variant)
	-> std::size_t
{
	using city_type = typename dist_traits<Dist>::city_type;

	for (std::size_t step = start_step; step < n; ++step) {
		auto partial = std::span<const city_type>(tour.data(), step);
		auto vis = std::span<const uint8_t>(visited.data(), n);
		auto next = call_select_next(strategy, dist, partial, vis, variant);
		if (!next)
			return step;

		tour[step] = *next;
		visited[static_cast<std::size_t>(*next)] = 1;

		call_on_placed(strategy, dist,
			std::span<const city_type>(tour.data(), step + 1), vis, variant);
	}
	return n;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Solver::greedy_construct
// ---------------------------------------------------------------------------

// With variant callbacks (primary implementation).
template <DistanceSource Dist, typename TourCost>
template <typename Strategy, typename Variant>
auto Solver<Dist, TourCost>::greedy_construct(
	Strategy strategy, const Variant& variant, ConstructParams params)
	-> Solver&
{
	assert(dist_);
	const auto n = dist_->size();
	ensure_shared(n);

	std::size_t start_step;

	if (status_ == SolutionStatus::partial) {
		start_step = n_;
	} else {
		if (n == 0) {
			n_ = 0;
			cost_ = {};
			status_ = SolutionStatus::feasible;
			return *this;
		}
		std::fill_n(visited_.data(), n, uint8_t{0});
		auto start = static_cast<city_type>(params.start_city);
		tour_[0] = start;
		visited_[params.start_city] = 1;
		start_step = 1;

		detail::call_on_placed(strategy, *dist_,
			std::span<const city_type>(tour_.data(), 1),
			std::span<const uint8_t>(visited_.data(), n), variant);
	}

	std::size_t placed = detail::greedy_append_build(
		*dist_, n, start_step,
		std::span<city_type>(tour_.data(), n),
		std::span<uint8_t>(visited_.data(), n),
		strategy, variant);

	n_ = placed;
	if (placed == n) {
		cost_ = compute_tour_cost(std::span<const city_type>(tour_.data(), n));
		status_ = SolutionStatus::feasible;
		rebuild_position();
	} else {
		cost_ = {};
		status_ = SolutionStatus::partial;
	}
	return *this;
}

// Without variant callbacks (forwards to primary).
template <DistanceSource Dist, typename TourCost>
template <typename Strategy>
auto Solver<Dist, TourCost>::greedy_construct(
	Strategy strategy, ConstructParams params)
	-> Solver&
{
	return greedy_construct(std::move(strategy), NoCallbacks{}, params);
}

} // namespace periple
