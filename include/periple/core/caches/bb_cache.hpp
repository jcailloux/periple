#pragma once

#include <cstddef>
#include <vector>

namespace periple {

// A child of a branch and bound node: a city the variant accepts after the last
// placed one, and its score through the pipeline (evaluate_append).
template <typename CityT>
struct BranchChild {
	double score;
	CityT city;
};

// Branch and bound workspace, refreshed on entry. One slot per depth d, the
// length of the tour at the node.
template <typename CostT, typename CityT>
struct BBCache {
	std::vector<std::vector<BranchChild<CityT>>> children;  // children of the node at depth d, allocated on first visit
	std::vector<std::size_t> next;                          // next child to explore
	std::vector<CostT> cost;                                // tour cost at depth d, restored on backtrack
	std::vector<CityT> best;                                // best complete tour found
	std::vector<BranchChild<CityT>> scored, ordered;        // a node's children around a user order, compared by debug builds

	void reset() {
		children.clear();
		next.clear();
		cost.clear();
		best.clear();
		scored.clear();
		ordered.clear();
	}
};

} // namespace periple
