#pragma once

// Branch and bound with the default bound: SpanningTreeBound under
// set_symmetric(true), SpanningArborescenceBound otherwise. The search and the
// other strategies have their own headers in branch_and_bound/.

#include <periple/algorithms/branch_and_bound/spanning_arborescence_bound.hpp>
#include <periple/algorithms/branch_and_bound/spanning_tree_bound.hpp>

namespace periple {

// ---------------------------------------------------------------------------
// Solver::branch_and_bound -- facade with the default bound
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::branch_and_bound(BranchAndBoundParams params) -> Solver& {
	if (symmetric_) return branch_and_bound(SpanningTreeBound{}, params);
	return branch_and_bound(SpanningArborescenceBound{}, params);
}

} // namespace periple
