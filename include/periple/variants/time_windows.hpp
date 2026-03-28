#pragma once

// TSP with Time Windows (TSPTW)
//
// Savelsbergh (1985), "Local Search in Routing Problems with Time Windows"

#include <periple/core/moves.hpp>
#include <periple/core/traits.hpp>

#include <algorithm>
#include <cassert>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace periple::time_windows {

struct TimeWindow {
	double earliest;
	double latest;
};

namespace detail {

// ---------------------------------------------------------------------------
// WindowStore -- flat storage for per-city time windows
// ---------------------------------------------------------------------------

class WindowStore {
public:
	// Single window per city.
	explicit WindowStore(std::span<const TimeWindow> windows)
		: windows_(windows.begin(), windows.end())
	{
		offsets_.resize(windows.size() + 1);
		for (std::size_t i = 0; i <= windows.size(); ++i)
			offsets_[i] = i;
	}

	// Optional window per city (nullopt = unconstrained).
	explicit WindowStore(std::span<const std::optional<TimeWindow>> windows) {
		offsets_.resize(windows.size() + 1);
		std::size_t pos = 0;
		for (std::size_t i = 0; i < windows.size(); ++i) {
			offsets_[i] = pos;
			if (windows[i]) {
				windows_.push_back(*windows[i]);
				++pos;
			}
		}
		offsets_[windows.size()] = pos;
	}

	// Multiple windows per city (sorted by earliest internally).
	explicit WindowStore(std::span<const std::vector<TimeWindow>> windows) {
		offsets_.reserve(windows.size() + 1);
		offsets_.push_back(0);
		for (const auto& ws : windows) {
			windows_.insert(windows_.end(), ws.begin(), ws.end());
			offsets_.push_back(windows_.size());
		}
		sort_windows();
	}

	[[nodiscard]] std::span<const TimeWindow> operator[](std::size_t city) const {
		return {windows_.data() + offsets_[city],
		        offsets_[city + 1] - offsets_[city]};
	}

private:
	void sort_windows() {
		for (std::size_t i = 0; i + 1 < offsets_.size(); ++i) {
			auto begin = windows_.begin() + static_cast<std::ptrdiff_t>(offsets_[i]);
			auto end   = windows_.begin() + static_cast<std::ptrdiff_t>(offsets_[i + 1]);
			std::sort(begin, end, [](const TimeWindow& a, const TimeWindow& b) {
				return a.earliest < b.earliest;
			});
		}
	}

	std::vector<TimeWindow> windows_;
	std::vector<std::size_t> offsets_;
};

// ---------------------------------------------------------------------------
// Helpers -- shared feasibility and timing logic
// ---------------------------------------------------------------------------

// Returns true if arriving at this time fits in at least one window
// (possibly after waiting for a later window to open).
// Windows must be sorted by earliest.
inline bool is_feasible(std::span<const TimeWindow> ws, double arrival) {
	if (ws.empty()) return true;
	for (const auto& w : ws) {
		if (arrival <= w.latest)
			return true;
	}
	return false;
}

// Computes departure time: waits for window if early, skips closed windows.
// Windows must be sorted by earliest.
inline double departure_time(std::span<const TimeWindow> ws, double arrival) {
	if (ws.empty()) return arrival;
	for (const auto& w : ws) {
		if (arrival <= w.latest)
			return std::max(arrival, w.earliest);
	}
	return arrival;
}

// Minimum lateness across all windows.  Returns 0 if any window is satisfied.
inline double violation_amount(std::span<const TimeWindow> ws, double arrival) {
	if (ws.empty()) return 0.0;
	double min_violation = std::numeric_limits<double>::max();
	for (const auto& w : ws) {
		if (arrival <= w.latest) return 0.0;
		min_violation = std::min(min_violation, arrival - w.latest);
	}
	return min_violation;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Strict: hard constraint via move_filter (reject infeasible moves)
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
struct Strict {
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	// Single window per city.
	Strict(const Dist& dist, std::span<const TimeWindow> windows)
		: dist_(&dist)
		, n_(dist.size())
		, store_(windows)
		, arrival_times_(dist.size(), cost_type{})
	{
		assert(windows.size() == dist.size() && "time_windows::Strict: must provide one window per city");
	}

	// Optional window per city (nullopt = unconstrained).
	Strict(const Dist& dist, std::span<const std::optional<TimeWindow>> windows)
		: dist_(&dist)
		, n_(dist.size())
		, store_(windows)
		, arrival_times_(dist.size(), cost_type{})
	{
		assert(windows.size() == dist.size() && "time_windows::Strict: must provide one optional window per city");
	}

	// Multiple windows per city.
	Strict(const Dist& dist, std::span<const std::vector<TimeWindow>> windows)
		: dist_(&dist)
		, n_(dist.size())
		, store_(windows)
		, arrival_times_(dist.size(), cost_type{})
	{
		assert(windows.size() == dist.size() && "time_windows::Strict: must provide one window vector per city");
	}

	// --- Constructive callbacks (AppendMove) ---

	bool move_filter(const AppendMove<city_type, cost_type>& m) const {
		assert(!m.tour.empty() && "move_filter: AppendMove tour must not be empty");
		auto arrival = next_arrival(m.tour, m.tour.size() - 1, m.city);
		return detail::is_feasible(windows_of(m.city), arrival);
	}

	void on_move(const AppendMove<city_type, cost_type>& m) const {
		auto pos = m.tour.size() - 1;
		if (pos == 0) {
			arrival_times_[0] = cost_type{};
		} else {
			arrival_times_[pos] = static_cast<cost_type>(
				next_arrival(m.tour, pos - 1, m.city));
		}
	}

	// --- DP callbacks (DPMove) ---

	bool move_filter(const DPMove<city_type, cost_type>& m) const {
		ensure_dp_arrival();
		return detail::is_feasible(windows_of(m.to), dp_arrival_at(m));
	}

	void on_move(const DPMove<city_type, cost_type>& m) const {
		ensure_dp_arrival();
		std::size_t set_next = m.set |
			(std::size_t{1} << static_cast<std::size_t>(m.to));
		dp_arrival_[set_next * n_ + static_cast<std::size_t>(m.to)] =
			dp_arrival_at(m);
	}

private:
	std::span<const TimeWindow> windows_of(city_type city) const {
		return store_[static_cast<std::size_t>(city)];
	}

	// Arrival time for constructive (sequential) path.
	double next_arrival(std::span<const city_type> tour,
	                    std::size_t from_pos, city_type to) const
	{
		auto from = tour[from_pos];
		auto depart = detail::departure_time(
			windows_of(from),
			static_cast<double>(arrival_times_[from_pos]));
		return depart + static_cast<double>((*dist_)(from, to));
	}

	// Arrival time at m.to for a DP transition.
	double dp_arrival_at(const DPMove<city_type, cost_type>& m) const {
		double arr = dp_arrival_[m.set * n_ + static_cast<std::size_t>(m.from)];
		double depart = detail::departure_time(windows_of(m.from), arr);
		return depart + static_cast<double>((*dist_)(m.from, m.to));
	}

	void ensure_dp_arrival() const {
		if (!dp_arrival_.empty()) return;
		dp_arrival_.resize(n_ * (std::size_t{1} << n_), 0.0);
		// Base state: arrival at city 0 in set {0} is time 0.
		dp_arrival_[1 * n_ + 0] = 0.0;
	}

	const Dist* dist_;
	std::size_t n_;
	detail::WindowStore store_;
	mutable std::vector<cost_type> arrival_times_;  // constructive: indexed by position
	mutable std::vector<double> dp_arrival_;         // DP: indexed by set*n + city
};

// ---------------------------------------------------------------------------
// Relaxed: soft constraint via penalties
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
struct Relaxed {
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	// Single window per city.
	Relaxed(const Dist& dist, std::span<const TimeWindow> windows,
	        cost_type penalty_weight = 1000)
		: dist_(&dist)
		, n_(dist.size())
		, store_(windows)
		, penalty_weight_(penalty_weight)
	{
		assert(windows.size() == dist.size() && "time_windows::Relaxed: must provide one window per city");
	}

	// Optional window per city (nullopt = unconstrained).
	Relaxed(const Dist& dist, std::span<const std::optional<TimeWindow>> windows,
	        cost_type penalty_weight = 1000)
		: dist_(&dist)
		, n_(dist.size())
		, store_(windows)
		, penalty_weight_(penalty_weight)
	{
		assert(windows.size() == dist.size() && "time_windows::Relaxed: must provide one optional window per city");
	}

	// Multiple windows per city.
	Relaxed(const Dist& dist, std::span<const std::vector<TimeWindow>> windows,
	        cost_type penalty_weight = 1000)
		: dist_(&dist)
		, n_(dist.size())
		, store_(windows)
		, penalty_weight_(penalty_weight)
	{
		assert(windows.size() == dist.size() && "time_windows::Relaxed: must provide one window vector per city");
	}

	// --- tour_cost: distance + penalty for late arrivals ---

	auto tour_cost(const Dist& dist,
	               std::span<const city_type> tour) const -> cost_type
	{
		const bool closed = (tour.size() == dist.size());
		const auto edges = closed ? tour.size() : tour.size() - 1;
		cost_type distance_cost{};
		cost_type penalty_cost{};
		double time = 0.0;

		for (std::size_t i = 0; i < edges; ++i) {
			auto ws = windows_of(tour[i]);
			auto violation = detail::violation_amount(ws, time);
			if (violation > 0.0)
				penalty_cost += static_cast<cost_type>(
					static_cast<double>(penalty_weight_) * violation);
			time = detail::departure_time(ws, time);

			auto travel = dist(tour[i], tour[(i + 1) % tour.size()]);
			distance_cost += travel;
			time += static_cast<double>(travel);
		}

		return distance_cost + penalty_cost;
	}

	// --- DP callbacks (DPMove) ---

	auto move_eval(const DPMove<city_type, cost_type>& m) const -> cost_type {
		ensure_dp_arrival();
		double arr_to = dp_arrival_at(m);
		double violation = detail::violation_amount(windows_of(m.to), arr_to);
		auto raw = (*dist_)(m.from, m.to);
		if (violation > 0.0)
			return raw + static_cast<cost_type>(
				static_cast<double>(penalty_weight_) * violation);
		return raw;
	}

	void on_move(const DPMove<city_type, cost_type>& m) const {
		ensure_dp_arrival();
		std::size_t set_next = m.set |
			(std::size_t{1} << static_cast<std::size_t>(m.to));
		dp_arrival_[set_next * n_ + static_cast<std::size_t>(m.to)] =
			dp_arrival_at(m);
	}

private:
	std::span<const TimeWindow> windows_of(city_type city) const {
		return store_[static_cast<std::size_t>(city)];
	}

	double dp_arrival_at(const DPMove<city_type, cost_type>& m) const {
		double arr = dp_arrival_[m.set * n_ + static_cast<std::size_t>(m.from)];
		double depart = detail::departure_time(windows_of(m.from), arr);
		return depart + static_cast<double>((*dist_)(m.from, m.to));
	}

	void ensure_dp_arrival() const {
		if (!dp_arrival_.empty()) return;
		dp_arrival_.resize(n_ * (std::size_t{1} << n_), 0.0);
		dp_arrival_[1 * n_ + 0] = 0.0;
	}

	const Dist* dist_;
	std::size_t n_;
	detail::WindowStore store_;
	cost_type penalty_weight_;
	mutable std::vector<double> dp_arrival_;  // DP: indexed by set*n + city
};

} // namespace periple::time_windows
