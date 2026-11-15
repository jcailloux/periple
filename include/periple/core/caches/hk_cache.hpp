#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace periple {

namespace detail {

// Open-addressing map from a visited-set bitmask to its slot. Key 0 marks an
// empty cell: every mask holds the start city, so none is 0.
class MaskIndex {
public:
	// Empties the map in O(entries), keeping its capacity.
	void clear() {
		for (auto cell : used_) keys_[cell] = 0;
		used_.clear();
	}

	void reset() {
		keys_.clear();
		vals_.clear();
		used_.clear();
	}

	template <typename Make>
	std::uint32_t find_or_insert(std::size_t mask, Make&& make) {
		if (2 * (used_.size() + 1) > keys_.size()) grow();
		std::size_t cell = home(mask, shift_);
		while (keys_[cell] != 0) {
			if (keys_[cell] == mask) return vals_[cell];
			cell = (cell + 1) & (keys_.size() - 1);
		}
		keys_[cell] = mask;
		vals_[cell] = make();
		used_.push_back(cell);
		return vals_[cell];
	}

private:
	static std::size_t home(std::size_t mask, int shift) {
		return static_cast<std::size_t>((static_cast<std::uint64_t>(mask) * 0x9E3779B97F4A7C15ull) >> shift);
	}

	void grow() {
		const std::size_t cap = keys_.empty() ? 64 : 2 * keys_.size();
		const int shift = 64 - std::countr_zero(cap);
		std::vector<std::size_t> keys(cap, 0);
		std::vector<std::uint32_t> vals(cap);
		for (auto& cell : used_) {
			std::size_t to = home(keys_[cell], shift);
			while (keys[to] != 0) to = (to + 1) & (cap - 1);
			keys[to] = keys_[cell];
			vals[to] = vals_[cell];
			cell = to;
		}
		keys_.swap(keys);
		vals_.swap(vals);
		shift_ = shift;
	}

	std::vector<std::size_t> keys_;
	std::vector<std::uint32_t> vals_;
	std::vector<std::size_t> used_;  // occupied cells, for an O(entries) clear
	int shift_ = 64;
};

} // namespace detail

// AllSets
// Allocated without initialization: held_karp fills dp itself and reads parent
// only where dp was reached, so zeroing would be a wasted pass over the table.
// A parent is a city below 25, stored on one byte.
template <typename CostT>
struct HKCache {
	std::unique_ptr<CostT[]> dp;
	std::unique_ptr<std::uint8_t[]> parent;
	std::size_t capacity = 0;

	void reserve(std::size_t size) {
		if (size <= capacity) return;
		dp.reset();  // free before allocating the larger tables
		parent.reset();
		dp     = std::make_unique_for_overwrite<CostT[]>(size);
		parent = std::make_unique_for_overwrite<std::uint8_t[]>(size);
		capacity = size;
	}

	void reset() {
		dp.reset();
		parent.reset();
		capacity = 0;
	}
};

// ReachableSets
template <typename CostT>
struct HKStatesCache {
	std::vector<std::size_t> masks;          // visited set of each slot, in layer order
	std::vector<CostT> cost;                 // per state: slot * n + city
	std::vector<std::uint32_t> pred;         // per state: predecessor state
	std::vector<std::uint32_t> target_slot;  // per target city, for the slot being expanded
	detail::MaskIndex index;                 // masks of the layer being discovered

	void reset() {
		masks.clear();
		cost.clear();
		pred.clear();
		target_slot.clear();
		index.reset();
	}
};

} // namespace periple
