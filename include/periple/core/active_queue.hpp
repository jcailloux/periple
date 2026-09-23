#pragma once

// ActiveQueue -- bounded work queue with don't-look bits
//
// Bentley (1992), "Fast Algorithms for Geometric Traveling Salesman Problems"
//
// FIFO over the n cities, each present at most once, flags[c] == 0 marking
// membership. Local search shows in the pop/deactivate split rather than in
// the buffer: a popped city keeps its flag clear while it is being scanned, so
// moves found from it do not re-enqueue it, and only deactivate() makes it
// queueable again. A plain worklist would clear the flag on pop.
//
// Non-owning view over two Solver workspace buffers of size n.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

namespace periple::detail {

template <typename CityT>
class ActiveQueue {
public:
	ActiveQueue(std::span<CityT> slots, std::span<std::uint8_t> flags)
		: slots_(slots), flags_(flags) {}

	// Activates every city of tour, in tour order.
	void reset(std::span<const CityT> tour) {
		std::fill(flags_.begin(), flags_.end(), std::uint8_t{0});
		std::copy(tour.begin(), tour.end(), slots_.begin());
		head_ = 0;
		count_ = tour.size();
	}

	[[nodiscard]] bool empty() const { return count_ == 0; }

	// The flag of the returned city stays clear until deactivate().
	CityT pop() {
		const CityT c = slots_[head_];
		head_ = (head_ + 1 == slots_.size()) ? 0 : head_ + 1;
		--count_;
		return c;
	}

	void push(CityT c) {
		const auto ci = static_cast<std::size_t>(c);
		if (!flags_[ci]) return;
		flags_[ci] = 0;
		const std::size_t tail = head_ + count_;
		slots_[tail >= slots_.size() ? tail - slots_.size() : tail] = c;
		++count_;
	}

	void deactivate(CityT c) { flags_[static_cast<std::size_t>(c)] = 1; }

private:
	std::span<CityT> slots_;
	std::span<std::uint8_t> flags_;
	std::size_t head_ = 0;
	std::size_t count_ = 0;
};

} // namespace periple::detail
