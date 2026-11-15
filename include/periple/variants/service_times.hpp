#pragma once

// Service Times variant
//
// Adds per-city service durations to the departure time.
// Uses the RouteTiming dimension to adjust departure after arrival.

#include <periple/core/dimensions/route_timing.hpp>
#include <periple/core/moves/dp_move.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <span>
#include <type_traits>
#include <vector>

namespace periple::service_times {

struct ServiceTimes {
	using dimension = RouteTiming;

	explicit ServiceTimes(std::span<const double> times) : times_(times.begin(), times.end()) {
		assert(std::all_of(times_.begin(), times_.end(), [](double t) { return t >= 0; })
			&& "ServiceTimes: every service time must be a nonnegative duration");
	}

	// --- AppendMove ---

	template <typename CityT, typename Ctx>
	void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const {
		auto i = static_cast<std::size_t>(m.city);
		assert(i < times_.size() && "ServiceTimes: city index out of bounds");
		ctx.template dim<RouteTiming>().departure += times_[i];
	}

	// --- DPMove ---

	template <typename CityT, typename Ctx>
	void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const {
		auto i = static_cast<std::size_t>(m.to);
		assert(i < times_.size() && "ServiceTimes: city index out of bounds");
		ctx.template dim<RouteTiming>().departure += times_[i];
	}

private:
	std::vector<double> times_;
};

} // namespace periple::service_times

namespace periple::detail {

// Adds each city's service duration, nonnegative as the constructor asserts.
template <> struct only_delays<service_times::ServiceTimes> : std::true_type {};

} // namespace periple::detail