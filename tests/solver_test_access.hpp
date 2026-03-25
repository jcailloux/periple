#pragma once

#include <periple/core/solver.hpp>

#include <cstddef>

namespace periple {

template <DistanceSource Dist, typename Variant = NoCallbacks>
class SolverTestAccess {
public:
	explicit SolverTestAccess(const Solver<Dist, Variant>& solver) : s_(solver) {}

	// Returns true if position_[tour_[i]] == i for all i.
	bool position_consistent() const {
		for (std::size_t i = 0; i < s_.n_; ++i)
			if (static_cast<std::size_t>(
					s_.position_[static_cast<std::size_t>(s_.tour_[i])]) != i)
				return false;
		return true;
	}

private:
	const Solver<Dist, Variant>& s_;
};

template <DistanceSource Dist, typename Variant>
SolverTestAccess(const Solver<Dist, Variant>&) -> SolverTestAccess<Dist, Variant>;

} // namespace periple
