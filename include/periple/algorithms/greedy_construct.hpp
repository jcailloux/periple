#pragma once

// Greedy append-based construction
//
// Generic loop that builds a tour by appending one city at a time.
// The selection strategy is injected via a Selector (e.g. NearestSelector).

#include <periple/core/solver.hpp>

#include <algorithm>
#include <limits>
#include <span>

namespace periple {
namespace detail {

// Generic append-based construction loop.
// Expects tour[0..start_step) and visited to be set by the caller.
// Does not compute cost; the Solver wrapper handles that.
template <DistanceSource Dist, typename Selector, typename Callbacks>
void greedy_append_build(
	const Dist& dist, std::size_t n, std::size_t start_step,
	std::span<typename dist_traits<Dist>::city_type> tour,
	std::span<uint8_t> visited,
	const Selector& select, const Callbacks& cb)
{
	using city_type = typename dist_traits<Dist>::city_type;

	// Score type is deduced from the selector (callback return type or cost_type).
	using score_type = decltype(select.evaluate(dist, std::span<const city_type>{}, city_type{}, cb));

	for (std::size_t step = start_step; step < n; ++step) {
		auto partial = std::span<const city_type>(tour.data(), step);
		score_type best_score = std::numeric_limits<score_type>::max();
		city_type best_city{};
		bool found = false;

		for (std::size_t j = 0; j < n; ++j) {
			if (visited[j]) continue;
			auto candidate = static_cast<city_type>(j);

			if constexpr (requires(const Callbacks& c, std::span<const city_type> t, const AppendMove<city_type>& m) {
				{ c.move_filter(t, m) } -> std::convertible_to<bool>;
			}) {
				if (!cb.move_filter(partial, AppendMove<city_type>{candidate}))
					continue;
			}

			score_type score = select.evaluate(dist, partial, candidate, cb);
			if (!found || score < best_score) {
				best_score = score;
				best_city = candidate;
				found = true;
			}
		}

		// Fallback: if all candidates were filtered, pick first unvisited.
		if (!found) {
			for (std::size_t j = 0; j < n; ++j) {
				if (!visited[j]) {
					best_city = static_cast<city_type>(j);
					break;
				}
			}
		}

		tour[step] = best_city;
		visited[static_cast<std::size_t>(best_city)] = 1;

		if constexpr (requires(const Callbacks& c, std::span<const city_type> t, const AppendMove<city_type>& m) {
			c.on_commit(t, m);
		}) {
			cb.on_commit(std::span<const city_type>(tour.data(), step + 1), AppendMove<city_type>{best_city});
		}
	}
}

} // namespace detail

// ---------------------------------------------------------------------------
// Solver::greedy_construct -- generic append-based construction
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename TourCost>
template <typename Selector, typename Callbacks>
auto Solver<Dist, TourCost>::greedy_construct(
	const Selector& sel, const Callbacks& cb, ConstructParams params)
	-> Solver&
{
	assert(dist_);
	const auto n = dist_->size();
	ensure_shared(n);

	std::size_t start_step;

	if (status_ == SolutionStatus::partial) {
		// Continue from partial tour.  visited_ and tour_[0..n_) are set.
		start_step = n_;
	} else {
		if (n == 0) {
			n_ = 0;
			cost_ = {};
			status_ = SolutionStatus::feasible;
			return *this;
		}
		// Fresh build: place start city.
		std::fill_n(visited_.data(), n, uint8_t{0});
		auto start = static_cast<city_type>(params.start_city);
		tour_[0] = start;
		visited_[params.start_city] = 1;
		start_step = 1;

		// Notify on_commit for the start city.
		if constexpr (requires(const Callbacks& c, std::span<const city_type> t,
		                       const AppendMove<city_type>& m) {
			c.on_commit(t, m);
		}) {
			cb.on_commit(std::span<const city_type>(tour_.data(), 1),
			             AppendMove<city_type>{start});
		}
	}

	detail::greedy_append_build(
		*dist_, n, start_step,
		std::span<city_type>(tour_.data(), n),
		std::span<uint8_t>(visited_.data(), n),
		sel, cb);

	n_ = n;
	cost_ = compute_tour_cost(std::span<const city_type>(tour_.data(), n));
	status_ = SolutionStatus::feasible;
	rebuild_position();
	return *this;
}

} // namespace periple
