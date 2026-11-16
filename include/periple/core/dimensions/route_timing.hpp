#pragma once

// RouteTiming -- built-in dimension for arrival/departure tracking
//
// Tracks cumulative arrival and departure times along the tour.
// Used by time_windows variants to check feasibility and compute penalties.
//
// Supports both constructive algorithms (AppendMove, per-position committed
// state) and exact DP algorithms (DPMove, per-state committed state, indexed
// by the move).

#include <periple/core/staged_vector.hpp>
#include <periple/core/traits.hpp>
#include <periple/core/moves/append_move.hpp>
#include <periple/core/moves/dp_move.hpp>

#include <compare>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace periple {

namespace detail {

// A variant that changes RouteTiming only by adding nonnegative amounts to its
// departure. When every component of a variant does, a departure is at least
// the arrival plus the amount of any one component, which DeadlineBound needs.
// The shipped variants that qualify specialize it.
template <typename V>
struct only_delays : std::false_type {};

} // namespace detail

struct RouteTiming {
	// Tentative state (written by init, read/adjusted by move_prepare)
	double arrival = 0;
	double departure = 0;
	std::size_t pos_ = 0;

	// Committed constructive (per-position, staged for non-destructive eval)
	// Only departures are stored: arrival is always derived in init from
	// departures[pos-1] + travel. This is the sole propagating state.
	StagedVector<double> departures;

	// Committed DP (per state, at the indices carried by DPMove)
	std::vector<double> dp_departures;

	void resize(std::size_t n) { departures.resize(n); }
	void reset() { departures.fill(0.0); }

	// --- Staging lifecycle ---------------------------------------------------

	void begin_staging(std::size_t from) { departures.begin_staging(from); }
	void discard_staging() { departures.discard_staging(); }
	void save_staging(std::size_t to) { departures.save_staging(to); }
	void commit_staging(std::size_t to) { departures.commit(to); }
	[[nodiscard]] bool staging_active() const { return departures.staging_active(); }

	// --- AppendMove (constructive) ----------------------------------------

	template <DistanceSource Dist, typename CityT>
	void init(const AppendMove<CityT>& m, const Dist& dist) {
		pos_ = m.pos;
		if (pos_ == 0) {
			arrival = 0.0;
			departure = 0.0;
		} else {
			arrival = departures[pos_ - 1]
				+ static_cast<double>(dist(m.prev_city, m.city));
			departure = arrival;
		}
	}

	template <typename CityT>
	void commit(const AppendMove<CityT>&) {
		departures[pos_] = departure;
	}

	struct AppendSnapshot { double arrival, departure; std::size_t pos; };

	template <typename CityT>
	auto snapshot(const AppendMove<CityT>&) const -> AppendSnapshot {
		return {arrival, departure, pos_};
	}
	void restore(const AppendSnapshot& s) {
		arrival = s.arrival; departure = s.departure; pos_ = s.pos;
	}

	// --- DPMove (exact, inline commit, no snapshot) -----------------------

	void resize_dp(std::size_t states) {
		if (dp_departures.size() < states) dp_departures.resize(states);
	}

	void begin_dp(std::size_t state) { dp_departures[state] = departure; }

	template <DistanceSource Dist, typename CityT>
	void init(const DPMove<CityT>& m, const Dist& dist) {
		arrival = dp_departures[m.from_state] + static_cast<double>(dist(m.from, m.to));
		departure = arrival;
	}

	template <typename CityT>
	void commit(const DPMove<CityT>& m) {
		dp_departures[m.to_state] = departure;
	}

	// Earlier is never worse: under the shipped variants, a later departure
	// meets no window an earlier one misses, and pays no less.
	[[nodiscard]] auto compare_dp(std::size_t state) const -> std::partial_ordering {
		return departure <=> dp_departures[state];
	}
};

} // namespace periple