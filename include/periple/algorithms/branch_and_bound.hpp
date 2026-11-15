#pragma once

// Branch and bound with the default bound: SpanningTreeBound when symmetric(),
// SpanningArborescenceBound otherwise, inside time_windows::DeadlineBound when
// its test is exact for the variant. The search and the other strategies have
// their own headers in branch_and_bound/.

#include <periple/algorithms/branch_and_bound/deadline_bound.hpp>
#include <periple/algorithms/branch_and_bound/spanning_arborescence_bound.hpp>
#include <periple/algorithms/branch_and_bound/spanning_tree_bound.hpp>

#include <utility>

namespace periple {

namespace detail {

// bound inside the deadline test when the test is exact for Variant, bound
// alone otherwise: decided at compile time.
template <typename Variant, typename Bound>
auto default_bound(Bound bound) {
	if constexpr (time_windows::detail::deadline_test_exact<Variant>) return time_windows::DeadlineBound(std::move(bound));
	else return bound;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Solver::branch_and_bound -- facade with the default bound
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::branch_and_bound(BranchAndBoundParams params) -> Solver& {
	if (symmetric_) return branch_and_bound(detail::default_bound<Variant>(SpanningTreeBound{}), params);
	return branch_and_bound(detail::default_bound<Variant>(SpanningArborescenceBound{}), params);
}

} // namespace periple
