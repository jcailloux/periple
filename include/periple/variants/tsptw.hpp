#pragma once

// TSP with Time Windows (TSPTW)
//
// Savelsbergh (1985), "Local Search in Routing Problems with Time Windows"

#include <periple/core/callbacks.hpp>
#include <periple/core/traits.hpp>

#include <algorithm>
#include <cassert>
#include <limits>
#include <span>
#include <vector>

namespace periple::tsptw {

struct TimeWindow {
	double earliest;
	double latest;
};

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

	std::span<const TimeWindow> operator[](std::size_t city) const {
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

namespace detail {

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
		, store_(windows)
		, arrival_times_(dist.size(), cost_type{})
	{}

	// Multiple windows per city.
	Strict(const Dist& dist, std::span<const std::vector<TimeWindow>> windows)
		: dist_(&dist)
		, store_(windows)
		, arrival_times_(dist.size(), cost_type{})
	{}

	bool move_filter(std::span<const city_type> tour,
	                 const AppendMove<city_type>& m) const
	{
		assert(!tour.empty());
		auto arrival = next_arrival(tour, tour.size() - 1, m.city);
		return detail::is_feasible(windows_of(m.city), arrival);
	}

	void on_commit(std::span<const city_type> tour,
	               const AppendMove<city_type>& m) const
	{
		auto pos = tour.size() - 1;
		if (pos == 0) {
			arrival_times_[0] = cost_type{};
		} else {
			arrival_times_[pos] = static_cast<cost_type>(
				next_arrival(tour, pos - 1, m.city));
		}
	}

private:
	std::span<const TimeWindow> windows_of(city_type city) const {
		return store_[static_cast<std::size_t>(city)];
	}

	double next_arrival(std::span<const city_type> tour,
	                    std::size_t from_pos, city_type to) const
	{
		auto from = tour[from_pos];
		auto depart = detail::departure_time(
			windows_of(from),
			static_cast<double>(arrival_times_[from_pos]));
		return depart + static_cast<double>((*dist_)(from, to));
	}

	const Dist* dist_;
	WindowStore store_;
	mutable std::vector<cost_type> arrival_times_;
};

// ---------------------------------------------------------------------------
// Relaxed: soft constraint via tour_cost penalties (no rejection)
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
struct Relaxed {
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	// Single window per city.  dist is used only for template deduction.
	Relaxed(const Dist&, std::span<const TimeWindow> windows,
	        cost_type penalty_weight = 1000)
		: store_(windows)
		, penalty_weight_(penalty_weight)
	{}

	// Multiple windows per city.  dist is used only for template deduction.
	Relaxed(const Dist&, std::span<const std::vector<TimeWindow>> windows,
	        cost_type penalty_weight = 1000)
		: store_(windows)
		, penalty_weight_(penalty_weight)
	{}

	// tour_cost callable: distance + penalty for late arrivals.
	auto operator()(const Dist& dist,
	                std::span<const city_type> tour) const -> cost_type
	{
		cost_type distance_cost{};
		cost_type penalty_cost{};
		double time = 0.0;

		for (std::size_t i = 0; i < tour.size(); ++i) {
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

private:
	std::span<const TimeWindow> windows_of(city_type city) const {
		return store_[static_cast<std::size_t>(city)];
	}

	WindowStore store_;
	cost_type penalty_weight_;
};

} // namespace periple::tsptw