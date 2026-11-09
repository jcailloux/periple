#pragma once

// Deadline bound for branch_and_bound, under strict time windows
//
// Desrosiers, Dumas, Soumis (1986), "A Dynamic Programming Solution of the Large-Scale
//   Single-Vehicle Dial-a-Ride Problem with Time Windows"
//   -- a partial tour is dropped once, leaving its last city at the earliest, it cannot
//      reach an unvisited city before that city's window closes, criterion 6
// Dumas, Desrosiers, Gelinas, Solomon (1995), "An Optimal Algorithm for the Traveling
//   Salesman Problem with Time Windows"
//   -- the same test by shortest paths without windows, over every unvisited city and
//      the end of the tour, test 1

#include <periple/algorithms/branch_and_bound/search.hpp>
#include <periple/variants/time_windows.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace periple::time_windows {

// Lower bound for branch_and_bound under Strict windows, around an inner bound:
// a node from which some unvisited city, or the return to the first, can no
// longer be reached before the last of its windows closes is a dead_end; any
// other node gets the inner bound's value. Passed as
// branch_and_bound(time_windows::DeadlineBound(tw, SpanningArborescenceBound{})),
// tw the Strict of the solver's variant. Leaving the last city at its earliest
// departure, the tour reaches each city no sooner than by the shortest path,
// since waiting only delays (Desrosiers, Dumas, Soumis; Dumas et al.). The test
// times the tour under tw alone, without services: a variant that adds them, or
// any other delay, only makes a tour later, so every tour it accepts passes.
// Shortest paths by Floyd-Warshall in begin, O(n^3) once, n^2 doubles. O(n)
// per node.
template <typename Bound>
class DeadlineBound {
public:
	static constexpr bool assumes_nonnegative_penalties = periple::detail::assumes_nonnegative_penalties<Bound>;

	DeadlineBound(const Strict& windows, Bound bound) : windows_(&windows), bound_(std::move(bound)) {}

	template <DistanceSource Dist, typename Variant, typename Stop>
	void begin(const Solver<Dist, Variant>& solver, Stop& stop) {
		n_ = solver.size();
		// Stopped, the search ends before the first lower_bound.
		if (!shortest_paths(solver, stop)) return;
		const auto tour = solver.tour();
		first_ = static_cast<std::size_t>(tour.front());
		limit_.resize(n_);
		for (std::size_t c = 0; c < n_; ++c) {
			const auto ws = windows_->windows(c);
			double latest = ws.empty() ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
			for (const auto& w : ws) latest = std::max(latest, w.latest);
			// The test adds times in another order than RouteTiming, so it rounds
			// differently: the margin keeps a tour that arrives just on time.
			limit_[c] = latest + 1e-9 * (1.0 + std::abs(latest));
		}
		open_ = limit_;
		placed_.resize(n_);
		departure_.resize(n_);
		masked_ = 0;
		for (std::size_t i = 0; i < tour.size(); ++i) {
			const auto c = static_cast<std::size_t>(tour[i]);
			const double arrival = i == 0 ? 0.0 : departure_[i - 1] + static_cast<double>(solver.distance(tour[i - 1], tour[i]));
			departure_[i] = detail::departure_time(windows_->windows(c), arrival);
			open_[c] = std::numeric_limits<double>::infinity();
			placed_[masked_++] = c;
		}
		if constexpr (requires { bound_.begin(solver, stop); }) bound_.begin(solver, stop);
		else if constexpr (requires { bound_.begin(solver); }) bound_.begin(solver);
	}

	template <DistanceSource Dist, typename Variant, typename Stop>
	auto lower_bound(const Solver<Dist, Variant>& solver, double cutoff, Stop& stop) -> double {
		if (misses_a_deadline(solver)) return dead_end;
		if constexpr (requires { bound_.lower_bound(solver, cutoff, stop); }) return bound_.lower_bound(solver, cutoff, stop);
		else return bound_.lower_bound(solver, cutoff);
	}

private:
	// Floyd-Warshall on travel times; false if stop fires first.
	template <DistanceSource Dist, typename Variant, typename Stop>
	bool shortest_paths(const Solver<Dist, Variant>& solver, Stop& stop) {
		using city_type = typename dist_traits<Dist>::city_type;
		paths_.resize(n_ * n_);
		for (std::size_t a = 0; a < n_; ++a)
			for (std::size_t b = 0; b < n_; ++b)
				paths_[a * n_ + b] = static_cast<double>(solver.distance(static_cast<city_type>(a), static_cast<city_type>(b)));
		for (std::size_t k = 0; k < n_; ++k) {
			if (stop()) return false;
			const double* via = paths_.data() + k * n_;
			for (std::size_t a = 0; a < n_; ++a) {
				double* from = paths_.data() + a * n_;
				const double to_k = from[k];
				for (std::size_t b = 0; b < n_; ++b) from[b] = std::min(from[b], to_k + via[b]);
			}
		}
		return true;
	}

	// Departs the last city at the earliest, then checks every deadline.
	template <DistanceSource Dist, typename Variant>
	bool misses_a_deadline(const Solver<Dist, Variant>& solver) {
		const auto tour = solver.tour();
		const std::size_t depth = tour.size();
		assert(masked_ + 1 >= depth && "DeadlineBound: lower_bound must run after begin, at every node, depth first");
		const auto last = static_cast<std::size_t>(tour[depth - 1]);
		const double arrival = departure_[depth - 2] + static_cast<double>(solver.distance(tour[depth - 2], tour[depth - 1]));
		const double departure = detail::departure_time(windows_->windows(last), arrival);
		departure_[depth - 1] = departure;
		// Depth first, the parent's prefix is masked already: cities placed
		// beyond it since get their limit back, then the last city masks.
		while (masked_ > depth - 1) {
			const std::size_t c = placed_[--masked_];
			open_[c] = limit_[c];
		}
		open_[last] = std::numeric_limits<double>::infinity();
		placed_[masked_++] = last;
		const double* to = paths_.data() + last * n_;
		bool late = departure + to[first_] > limit_[first_];
		// Eight cities without a branch, then the next eight: a late city ends the scan.
		std::size_t c = 0;
		for (; c + 8 <= n_ && !late; c += 8)
			for (std::size_t k = c; k < c + 8; ++k) late |= departure + to[k] > open_[k];
		for (; c < n_ && !late; ++c) late |= departure + to[c] > open_[c];
		return late;
	}

	const Strict* windows_;
	Bound bound_;
	std::size_t n_ = 0, first_ = 0;
	std::vector<double> paths_;        // paths_[a * n_ + b]: shortest travel time from a to b
	std::vector<double> limit_;        // by city: the end of its last window, plus the margin
	std::vector<double> open_;         // limit_, +infinity for the cities of the prefix
	std::vector<std::size_t> placed_;  // by position: the cities open_ masks
	std::size_t masked_ = 0;           // how many
	std::vector<double> departure_;    // by position: the earliest departure, under the windows alone
};

} // namespace periple::time_windows
