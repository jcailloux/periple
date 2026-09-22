#pragma once

// CumulativeCost -- built-in dimension for prefix cost tracking
//
// Tracks the accumulated tour cost (edge distances + variant penalties)
// at each position. Implicit in every variant's context (see context_for);
// backs Solver::prefix_cost.
//
// Captures cost_delta automatically via ctx-aware commit dispatch:
// variants that set ctx.cost_delta (e.g. Relaxed time windows) feed
// into this dimension without any explicit coupling.

#include <periple/core/staged_vector.hpp>
#include <periple/core/traits.hpp>
#include <periple/core/moves/append_move.hpp>

#include <cstddef>

namespace periple {

struct CumulativeCost {
	// Tentative state (written by init, consumed by commit)
	double prefix = 0;      // committed cost at pos-1
	double edge = 0;        // raw distance of the appended edge
	std::size_t pos_ = 0;

	// Committed (per-position, staged for non-destructive eval)
	StagedVector<double> costs;

	void resize(std::size_t n) { costs.resize(n); }
	void reset() { costs.fill(0.0); }

	// --- Staging lifecycle ---------------------------------------------------

	void begin_staging(std::size_t from) { costs.begin_staging(from); }
	void discard_staging() { costs.discard_staging(); }
	void save_staging(std::size_t to) { costs.save_staging(to); }
	void commit_staging(std::size_t to) { costs.commit(to); }

	// --- AppendMove ----------------------------------------------------------

	template <DistanceSource Dist, typename CityT>
	void init(const AppendMove<CityT>& m, const Dist& dist) {
		pos_ = m.pos;
		if (m.pos == 0) {
			prefix = 0.0;
			edge = 0.0;
		} else {
			prefix = costs[m.pos - 1];
			edge = static_cast<double>(dist(m.prev_city, m.city));
		}
	}

	// Mirrors Solver::append: per-edge cast to cost_type, nothing at pos 0,
	// so prefix costs agree with cost_ even for int costs with fractional penalties.
	template <typename CityT, typename Ctx>
	void commit(const AppendMove<CityT>&, const Ctx& ctx) {
		using cost_type = decltype(ctx.cost());
		costs[pos_] = (pos_ == 0) ? 0.0
			: prefix + static_cast<double>(static_cast<cost_type>(edge + ctx.cost_delta));
	}

	struct AppendSnapshot { double prefix, edge; std::size_t pos; };

	template <typename CityT>
	auto snapshot(const AppendMove<CityT>&) const -> AppendSnapshot {
		return {prefix, edge, pos_};
	}
	void restore(const AppendSnapshot& s) {
		prefix = s.prefix; edge = s.edge; pos_ = s.pos;
	}
};

} // namespace periple
