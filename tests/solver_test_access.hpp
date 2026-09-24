#pragma once

#include <periple/core/solver.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <span>
#include <vector>

namespace periple {

template <DistanceSource Dist, typename Variant = NoCallbacks>
class SolverTestAccess {
public:
	explicit SolverTestAccess(const Solver<Dist, Variant>& solver) : s_(solver) {}

	// Returns true if position_[tour_[i]] == i for all i.
	[[nodiscard]] bool position_consistent() const {
		for (std::size_t i = 0; i < s_.n_; ++i)
			if (static_cast<std::size_t>(
					s_.position_[static_cast<std::size_t>(s_.tour_[i])]) != i)
				return false;
		return true;
	}

	// Returns true if visited_[c] == (c is in tour_[0..n_)) for every city.
	[[nodiscard]] bool visited_consistent() const {
		if (s_.n_ == 0) return true;
		const auto total = s_.dist_->size();
		for (std::size_t c = 0; c < total; ++c) {
			const auto p = static_cast<std::size_t>(s_.position_[c]);
			const bool in_tour = p < s_.n_ && static_cast<std::size_t>(s_.tour_[p]) == c;
			if ((s_.visited_[c] != 0) != in_tour) return false;
		}
		return true;
	}

	[[nodiscard]] std::size_t tour_version() const { return s_.tour_version_; }

	// Returns true if no dimension was left mid-evaluation.
	[[nodiscard]] bool staging_inactive() const { return !s_.ctx_.staging_active(); }

	// Returns true if the cached path costs are marked valid for the current tour.
	[[nodiscard]] bool path_costs_current() const {
		return s_.path_costs_version_ == s_.tour_version_;
	}

	// Returns true unless the path costs are marked valid and disagree with a
	// fresh recomputation from tour_.
	[[nodiscard]] bool path_costs_consistent() const {
		if (s_.n_ == 0 || !path_costs_current()) return true;
		typename Solver<Dist, Variant>::cost_type fwd{}, bwd{};
		for (std::size_t q = 0; q + 1 < s_.n_; ++q) {
			if (s_.path_fwd_[q] != fwd || s_.path_bwd_[q] != bwd) return false;
			fwd += (*s_.dist_)(s_.tour_[q], s_.tour_[q + 1]);
			bwd += (*s_.dist_)(s_.tour_[q + 1], s_.tour_[q]);
		}
		return s_.path_fwd_[s_.n_ - 1] == fwd && s_.path_bwd_[s_.n_ - 1] == bwd;
	}

private:
	const Solver<Dist, Variant>& s_;
};

template <DistanceSource Dist, typename Variant>
SolverTestAccess(const Solver<Dist, Variant>&) -> SolverTestAccess<Dist, Variant>;

// Invariants every method must leave in place, whatever the final status.
template <DistanceSource Dist, typename Variant>
void assert_tour_invariants(const Solver<Dist, Variant>& solver) {
	SolverTestAccess access(solver);
	assert(access.position_consistent() && "position_ must match tour_");
	assert(access.visited_consistent() && "visited_ must match tour_");
	assert(access.staging_inactive() && "no evaluation may be left staged");
	assert(access.path_costs_consistent() && "path costs must be valid or stale");
}

// Runs a solver method and checks its output invariants, plus the contract every
// tour-derived cache relies on: changing the tour changes tour_version_. Without
// the bump such a cache stays silently marked valid for a tour it no longer
// describes, which no observable behaviour would reveal.
template <DistanceSource Dist, typename Variant, typename Fn>
void run_checked(Solver<Dist, Variant>& solver, Fn&& call) {
	using city_type = typename Solver<Dist, Variant>::city_type;
	const std::vector<city_type> before(solver.tour().begin(), solver.tour().end());
	const auto version = SolverTestAccess(solver).tour_version();

	call();

	const auto after = solver.tour();
	assert((std::equal(before.begin(), before.end(), after.begin(), after.end())
	        || SolverTestAccess(solver).tour_version() != version)
		&& "a method that changes the tour must change tour_version_");
	assert_tour_invariants(solver);
}

} // namespace periple
