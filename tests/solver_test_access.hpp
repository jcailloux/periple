#pragma once

#include <periple/core/solver.hpp>

#include <cstddef>
#include <span>

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

private:
	const Solver<Dist, Variant>& s_;
};

template <DistanceSource Dist, typename Variant>
SolverTestAccess(const Solver<Dist, Variant>&) -> SolverTestAccess<Dist, Variant>;

} // namespace periple
