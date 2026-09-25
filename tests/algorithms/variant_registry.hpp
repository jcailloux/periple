#pragma once

// Shipped variants, one entry per configuration. test_variants runs every
// entry under every registered algorithm and checks the results against the
// AppendMove pipeline, so a new variant is tested by adding its entry here.

#include <periple/core/composed.hpp>
#include <periple/distance/matrix.hpp>
#include <periple/variants/service_times.hpp>
#include <periple/variants/time_windows.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <tuple>
#include <type_traits>
#include <vector>

namespace periple::test {

struct Rng {
	std::uint64_t x;
	std::uint64_t operator()() { x ^= x << 13; x ^= x >> 7; x ^= x << 17; return x; }
	int below(int hi) { return static_cast<int>((*this)() % static_cast<std::uint64_t>(hi)); }
};

// A random asymmetric instance and a random witness tour, whose arrival times
// anchor the windows: instances mix feasible and infeasible ones.
struct Instance {
	DistanceMatrix<int> dist;
	std::vector<std::size_t> witness;
	Rng& rng;
};

// Arrival time at each city along the witness, with no waiting.
inline std::vector<double> witness_arrivals(const Instance& in, std::span<const double> service) {
	std::vector<double> arrival(in.dist.size(), 0.0);
	double t = 0.0;
	for (std::size_t k = 1; k < in.witness.size(); ++k) {
		const auto a = in.witness[k - 1], b = in.witness[k];
		t += (service.empty() ? 0.0 : service[a]) + static_cast<double>(in.dist(a, b));
		arrival[b] = t;
	}
	return arrival;
}

// Deadlines from 8 before to 11 after the witness arrivals; with waiting, each
// window opens up to 19 before.
inline std::vector<time_windows::TimeWindow> make_windows(Instance& in, std::span<const double> service, bool waiting) {
	const auto arrival = witness_arrivals(in, service);
	std::vector<time_windows::TimeWindow> windows(in.dist.size());
	windows[0] = {0.0, 1e9};  // the depot never closes
	for (std::size_t c = 1; c < windows.size(); ++c) {
		const double earliest = waiting ? std::max(0.0, arrival[c] - in.rng.below(20)) : 0.0;
		windows[c] = {earliest, arrival[c] - 8 + in.rng.below(20)};
	}
	return windows;
}

inline std::vector<double> make_service(Instance& in) {
	std::vector<double> service(in.dist.size(), 0.0);
	for (std::size_t c = 1; c < service.size(); ++c) service[c] = in.rng.below(5);
	return service;
}

// exact_optimal: an exact algorithm must reach the brute-force optimum. False
// only for a known issue listed in CHANGELOG.md.

struct StrictDeadlines {
	static constexpr const char* name = "strict_deadlines";
	static constexpr bool exact_optimal = true;
	time_windows::Strict variant;
	explicit StrictDeadlines(Instance& in) : variant(make_windows(in, {}, false)) {}
};

struct StrictWindows {
	static constexpr const char* name = "strict_windows";
	static constexpr bool exact_optimal = false;  // known issue, see CHANGELOG.md
	time_windows::Strict variant;
	explicit StrictWindows(Instance& in) : variant(make_windows(in, {}, true)) {}
};

struct RelaxedWindows {
	static constexpr const char* name = "relaxed_windows";
	static constexpr bool exact_optimal = false;  // known issue, see CHANGELOG.md
	time_windows::Relaxed variant;
	explicit RelaxedWindows(Instance& in) : variant(make_windows(in, {}, true), 10) {}
};

struct ServiceOnly {
	static constexpr const char* name = "service_times";
	static constexpr bool exact_optimal = true;
	service_times::ServiceTimes variant;
	explicit ServiceOnly(Instance& in) : variant(make_service(in)) {}
};

struct ServiceStrictDeadlines {
	static constexpr const char* name = "service_times+strict_deadlines";
	static constexpr bool exact_optimal = true;
	std::vector<double> service;
	service_times::ServiceTimes times;
	time_windows::Strict windows;
	Composed<service_times::ServiceTimes, time_windows::Strict> variant;
	explicit ServiceStrictDeadlines(Instance& in)
		: service(make_service(in)), times(service), windows(make_windows(in, service, false)),
		  variant(times, windows) {}
};

// Master list -- add new shipped variants here.
using ShippedVariants = std::tuple<StrictDeadlines, StrictWindows, RelaxedWindows, ServiceOnly, ServiceStrictDeadlines>;

template <typename F>
void for_each_shipped_variant(F&& fn) {
	[&]<typename... Es>(std::tuple<Es...>*) {
		(fn(std::type_identity<Es>{}), ...);
	}(static_cast<ShippedVariants*>(nullptr));
}

} // namespace periple::test
