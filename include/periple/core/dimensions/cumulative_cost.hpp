#pragma once

// CumulativeCost -- opt-in dimension for prefix cost tracking
//
// Tracks the accumulated tour cost (edge distances + variant penalties)
// at each position. Used by evaluate_replay for O(1) prefix cost lookup.
//
// Captures cost_delta automatically via ctx-aware commit dispatch:
// variants that set ctx.cost_delta (e.g. Relaxed time windows) feed
// into this dimension without any explicit coupling.

#include <periple/core/staged_vector.hpp>
#include <periple/core/traits.hpp>

#include <cstddef>

namespace periple {

struct CumulativeCost {
	// Tentative state
	double pending = 0;
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
			pending = 0.0;
		} else {
			pending = costs[m.pos - 1] + static_cast<double>(dist(m.prev_city, m.city));
		}
	}

	template <typename CityT, typename Ctx>
	void commit(const AppendMove<CityT>&, const Ctx& ctx) {
		costs[pos_] = pending + ctx.cost_delta;
	}

	struct AppendSnapshot { double pending; std::size_t pos; };

	template <typename CityT>
	auto snapshot(const AppendMove<CityT>&) const -> AppendSnapshot {
		return {pending, pos_};
	}
	void restore(const AppendSnapshot& s) {
		pending = s.pending; pos_ = s.pos;
	}
};

} // namespace periple