#pragma once

#include <cstddef>
#include <vector>

namespace periple {

// Branch and bound workspace, refreshed on entry. One slot per depth d, the
// length of the tour at the node.
template <typename CostT, typename CityT>
struct BBCache {
	struct Child { double score; CityT city; };

	std::vector<std::vector<Child>> children;  // children of the node at depth d, allocated on first visit
	std::vector<std::size_t> next;             // next child to explore
	std::vector<CostT> cost;                   // tour cost at depth d, restored on backtrack
	std::vector<CityT> best;                   // best complete tour found

	void reset() {
		children.clear();
		next.clear();
		cost.clear();
		best.clear();
	}
};

} // namespace periple
