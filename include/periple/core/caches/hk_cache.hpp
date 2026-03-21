#pragma once

#include <vector>

namespace periple {

template <typename CostT, typename CityT>
struct HKCache {
	std::vector<CostT> dp;
	std::vector<CityT> parent;

	void reset() {
		dp.clear();
		parent.clear();
	}
};

} // namespace periple