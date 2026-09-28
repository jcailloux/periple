#pragma once

#include <cstddef>
#include <memory>

namespace periple {

// Allocated without initialization: held_karp fills dp itself and reads parent
// only where dp was reached, so zeroing would be a wasted pass over the table.
template <typename CostT, typename CityT>
struct HKCache {
	std::unique_ptr<CostT[]> dp;
	std::unique_ptr<CityT[]> parent;
	std::size_t capacity = 0;

	void reserve(std::size_t size) {
		if (size <= capacity) return;
		dp.reset();  // free before allocating the larger tables
		parent.reset();
		dp     = std::make_unique_for_overwrite<CostT[]>(size);
		parent = std::make_unique_for_overwrite<CityT[]>(size);
		capacity = size;
	}

	void reset() {
		dp.reset();
		parent.reset();
		capacity = 0;
	}
};

} // namespace periple
