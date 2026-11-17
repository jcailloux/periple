#pragma once

// Latest departures toward due times, by shortest paths: the test that drops a
// partial tour which can no longer reach a city in time
//
// Dumas, Desrosiers, Gelinas, Solomon (1995), "An Optimal Algorithm for the Traveling
//   Salesman Problem with Time Windows"
//   -- a partial tour leaving its last city at the earliest is dropped once, by the
//      shortest path without windows, an unvisited city or the end of the tour can no
//      longer be reached by its deadline, test 1

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace periple::detail {

// Per city j, the latest departure from j that still reaches each city c by
// its due time through the shortest path.
struct LatestDepartures {
	std::vector<double> slack;  // slack[j * n + c]: due(c) - sp(j, c)
	std::vector<double> least;  // by j: the least slack of row j, j itself left out; from build_least_slacks
};

// Fills ld.slack from distance(a, b) and due (one per city, margin included):
// Floyd-Warshall, O(n^3), then the slacks. False if stop fires first; polled
// once per pass of Floyd-Warshall.
template <typename Distance, typename Stop>
bool build_latest_departures(LatestDepartures& ld, Distance distance, std::span<const double> due, Stop& stop) {
	const std::size_t n = due.size();
	auto& sp = ld.slack;
	sp.resize(n * n);
	for (std::size_t a = 0; a < n; ++a)
		for (std::size_t b = 0; b < n; ++b) sp[a * n + b] = static_cast<double>(distance(a, b));
	for (std::size_t k = 0; k < n; ++k) {
		if (stop()) return false;
		const double* via = sp.data() + k * n;
		for (std::size_t a = 0; a < n; ++a) {
			double* from = sp.data() + a * n;
			const double to_k = from[k];
			for (std::size_t b = 0; b < n; ++b) from[b] = std::min(from[b], to_k + via[b]);
		}
	}
	// A city without a due time keeps +infinity, whatever the path.
	for (std::size_t a = 0; a < n; ++a)
		for (std::size_t c = 0; c < n; ++c) sp[a * n + c] = std::isinf(due[c]) ? due[c] : due[c] - sp[a * n + c];
	return true;
}

// Fills ld.least from ld.slack, for may_miss_a_due_time. O(n^2).
inline void build_least_slacks(LatestDepartures& ld, std::size_t n) {
	ld.least.resize(n);
	for (std::size_t j = 0; j < n; ++j) {
		const double* row = ld.slack.data() + j * n;
		double least = std::numeric_limits<double>::infinity();
		for (std::size_t c = 0; c < n; ++c)
			if (c != j) least = std::min(least, row[c]);
		ld.least[j] = least;
	}
}

// Whether a tour leaving j at departure can no longer reach in time the first
// city or a city c with open(c), even by the shortest path. O(n).
template <typename Open>
bool misses_a_due_time(const LatestDepartures& ld, std::size_t n, std::size_t j,
                       double departure, std::size_t first, Open open) {
	const double* row = ld.slack.data() + j * n;
	bool late = departure > row[first];
	// Eight cities without a branch, then the next eight: a late city ends the scan.
	std::size_t c = 0;
	for (; c + 8 <= n && !late; c += 8)
		for (std::size_t k = c; k < c + 8; ++k) late |= open(k) & (departure > row[k]);
	for (; c < n && !late; ++c) late |= open(c) & (departure > row[c]);
	return late;
}

// Whether misses_a_due_time can hold at all: departure beyond the least slack
// of row j. O(1), but not from the papers: the least slack counts the visited
// cities too, so it concludes only while every city could still be reached in
// time.
inline bool may_miss_a_due_time(const LatestDepartures& ld, std::size_t j, double departure) {
	return departure > ld.least[j];
}

} // namespace periple::detail
