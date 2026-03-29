#pragma once

// Service Times variant
//
// Adds per-city service durations to the departure time.
// Uses the RouteTiming dimension to adjust departure after arrival.

#include <periple/core/dimensions/route_timing.hpp>

#include <cassert>
#include <cstddef>
#include <span>

namespace periple::service_times {

struct ServiceTimes {
	using dimension = RouteTiming;

	explicit ServiceTimes(std::span<const double> times) : times_(times.begin(), times.end()) {}

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