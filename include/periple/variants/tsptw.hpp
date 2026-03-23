#pragma once

#include <periple/core/callbacks.hpp>
#include <periple/core/traits.hpp>

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

namespace periple::tsptw {

struct TimeWindow {
	double earliest;
	double latest;
};

// ---------------------------------------------------------------------------
// Strict: hard constraint via move_filter (reject infeasible moves)
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
struct Strict {
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	Strict(const Dist& dist, std::span<const TimeWindow> windows)
		: dist_(&dist)
		, windows_(windows.begin(), windows.end())
		, arrival_times_(dist.size(), cost_type{})
	{}

	bool move_filter(std::span<const city_type> tour,
	                 const AppendMove<city_type>& m) const
	{
		assert(!tour.empty());
		auto last = tour.back();
		auto last_arrival = arrival_times_[tour.size() - 1];
		auto depart = std::max(last_arrival,
			static_cast<cost_type>(windows_[static_cast<std::size_t>(last)].earliest));
		auto arrival = depart + (*dist_)(last, m.city);
		return arrival <=
			static_cast<cost_type>(windows_[static_cast<std::size_t>(m.city)].latest);
	}

	void on_commit(std::span<const city_type> tour,
	               const AppendMove<city_type>& m) const
	{
		auto pos = tour.size() - 1;
		if (pos == 0) {
			arrival_times_[0] = cost_type{};
		} else {
			auto prev = tour[pos - 1];
			auto prev_arrival = arrival_times_[pos - 1];
			auto depart = std::max(prev_arrival,
				static_cast<cost_type>(
					windows_[static_cast<std::size_t>(prev)].earliest));
			arrival_times_[pos] = depart + (*dist_)(prev, m.city);
		}
	}

private:
	const Dist* dist_;
	std::vector<TimeWindow> windows_;
	mutable std::vector<cost_type> arrival_times_;
};

// ---------------------------------------------------------------------------
// Relaxed: soft constraint via tour_cost penalties (no rejection)
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
struct Relaxed {
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	Relaxed(const Dist& dist, std::span<const TimeWindow> windows,
	         cost_type penalty_weight = 1000)
		: dist_(&dist)
		, windows_(windows.begin(), windows.end())
		, penalty_weight_(penalty_weight)
	{}

	// tour_cost callable: distance + penalty for late arrivals.
	auto operator()(const Dist& dist,
	                std::span<const city_type> tour) const -> cost_type
	{
		cost_type distance_cost{};
		cost_type penalty_cost{};
		cost_type time{};
		const auto n = tour.size();

		for (std::size_t i = 0; i < n; ++i) {
			auto ci = static_cast<std::size_t>(tour[i]);

			auto latest = static_cast<cost_type>(windows_[ci].latest);
			if (time > latest)
				penalty_cost += penalty_weight_ * (time - latest);

			auto earliest = static_cast<cost_type>(windows_[ci].earliest);
			time = std::max(time, earliest);

			auto next = tour[(i + 1) % n];
			auto travel = dist(tour[i], next);
			distance_cost += travel;
			time += travel;
		}

		return distance_cost + penalty_cost;
	}

private:
	const Dist* dist_;
	std::vector<TimeWindow> windows_;
	cost_type penalty_weight_;
};

} // namespace periple::tsptw
