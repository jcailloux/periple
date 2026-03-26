#pragma once

// Greedy append-based construction
//
// Generic loop that builds a tour by appending one city at a time.
// The Strategy decides which city to append next; the framework only
// manages the tour buffer, visited flags, and termination.

#include <periple/core/solver.hpp>

#include <optional>

namespace periple {

// ---------------------------------------------------------------------------
// Solver::greedy_construct
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
template <typename Strategy>
auto Solver<Dist, Variant>::greedy_construct(
	Strategy strategy, ConstructParams params)
	-> Solver&
{
	assert(dist_ && "greedy_construct: no distance source set");
	const auto total = dist_->size();

	if (params.resume_at > 0) {
		assert(params.resume_at <= n_
			&& "greedy_construct: resume_at exceeds current tour length");
		const auto k = params.resume_at;
		ensure_capacity(total);
		std::fill_n(visited_.data(), total, uint8_t{0});
		for (std::size_t i = 0; i < k; ++i)
			visited_[static_cast<std::size_t>(tour_[i])] = 1;
		n_ = k;
		auto prefix = std::span<const city_type>(tour_.data(), k);
		const auto& variant = variant_ref();
		if constexpr (requires {
			{ variant.on_truncate(*dist_, prefix) } -> std::convertible_to<cost_type>;
		}) {
			cost_ = static_cast<cost_type>(variant.on_truncate(*dist_, prefix));
		} else {
			cost_ = compute_tour_cost(prefix);
		}
		status_ = SolutionStatus::partial;
	} else {
		if (try_trivial()) return *this;
		clear();
		append(static_cast<city_type>(params.start_city));
	}

	while (n_ < total) {
		auto next = strategy.select_next(*this);
		if (!next) break;
		append(*next);
	}

	return *this;
}

} // namespace periple
