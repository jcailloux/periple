#pragma once

// Greedy append-based construction
//
// Generic loop that builds a tour by appending one city at a time.
// The Strategy decides which city to append next; the framework only
// manages the tour buffer, visited flags, and termination.

#include <periple/core/solver.hpp>

#include <optional>

namespace periple {

namespace detail {

// A strategy that runs long takes stop as second argument.
template <typename Strategy, typename S, typename Stop>
auto select_next(Strategy& strategy, S& solver, Stop& stop) {
	if constexpr (requires { strategy.select_next(solver, stop); })
		return strategy.select_next(solver, stop);
	else
		return strategy.select_next(solver);
}

} // namespace detail

// ---------------------------------------------------------------------------
// Solver::greedy_construct
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
template <typename Strategy>
auto Solver<Dist, Variant>::greedy_construct(
	Strategy strategy, ConstructParams params)
	-> Solver&
{
	return detail::with_deadline(params.time_limit, [&](auto& deadline) -> Solver& {
		return greedy_construct_run(strategy, params, deadline);
	});
}

template <DistanceSource Dist, typename Variant>
template <typename Strategy, typename D>
auto Solver<Dist, Variant>::greedy_construct_run(
	Strategy& strategy, const ConstructParams& params, D& deadline)
	-> Solver&
{
	assert(dist_ && "greedy_construct: no distance source set");
	const auto total = dist_->size();

	if (params.resume_at > 0) {
		assert(params.resume_at <= n_
			&& "greedy_construct: resume_at exceeds current tour length");
		resume_prefix(params.resume_at);
	} else {
		if (try_trivial()) return *this;
		clear();
		append(static_cast<city_type>(params.start_city));
	}

	detail::Poller stop(deadline);
	detail::Poller stop_strategy(deadline);  // not the loop's: its units cost differently
	while (n_ < total && !stop()) {
		auto next = detail::select_next(strategy, *this, stop_strategy);
		if (!next) break;
		append(*next);
	}

	return *this;
}

} // namespace periple