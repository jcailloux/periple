#pragma once

// RouteTiming -- built-in dimension for arrival/departure tracking
//
// Tracks cumulative arrival and departure times along the tour.
// Used by time_windows variants to check feasibility and compute penalties.
//
// Supports both constructive algorithms (AppendMove, per-position committed
// state) and exact DP algorithms (DPMove, per-(set,city) committed state).

#include <periple/core/dimensions.hpp>
#include <periple/core/moves/dp_move.hpp>

#include <cstddef>
#include <vector>

namespace periple {

struct RouteTiming {
	// Tentative state (written by init, read/adjusted by move_prepare)
	double arrival = 0;
	double departure = 0;
	std::size_t pos_ = 0;

	// Committed constructive (per-position)
	std::vector<double> arrivals;
	std::vector<double> departures;

	// Committed DP (per-(set,city), allocated lazily on first DPMove init)
	std::vector<double> dp_arrivals;
	std::vector<double> dp_departures;
	std::size_t dp_n_ = 0;

	void resize(std::size_t n) { arrivals.resize(n); departures.resize(n); }
	void reset() {
		std::fill(arrivals.begin(), arrivals.end(), 0.0);
		std::fill(departures.begin(), departures.end(), 0.0);
	}

	// --- AppendMove (constructive) ----------------------------------------

	template <DistanceSource Dist, typename CityT, typename Ctx>
	void init(const AppendMove<CityT>& m, const Dist& dist, const Ctx& ctx) {
		pos_ = ctx.tour().size();
		if (pos_ == 0) {
			arrival = 0.0;
			departure = 0.0;
		} else {
			arrival = departures[pos_ - 1]
				+ static_cast<double>(dist(ctx.tour().back(), m.city));
			departure = arrival;
		}
	}

	template <typename CityT>
	void commit(const AppendMove<CityT>&) {
		arrivals[pos_] = arrival;
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

	template <DistanceSource Dist, typename CityT>
	void init(const DPMove<CityT>& m, const Dist& dist) {
		if (dp_arrivals.empty()) {
			dp_n_ = dist.size();
			dp_arrivals.resize(dp_n_ * (std::size_t{1} << dp_n_));
			dp_departures.resize(dp_n_ * (std::size_t{1} << dp_n_));
		}
		std::size_t idx = m.set * dp_n_ + static_cast<std::size_t>(m.from);
		arrival = dp_departures[idx] + static_cast<double>(dist(m.from, m.to));
		departure = arrival;
	}

	template <typename CityT>
	void commit(const DPMove<CityT>& m) {
		std::size_t new_set = m.set | (std::size_t{1} << static_cast<std::size_t>(m.to));
		std::size_t idx = new_set * dp_n_ + static_cast<std::size_t>(m.to);
		dp_arrivals[idx] = arrival;
		dp_departures[idx] = departure;
	}
};

} // namespace periple