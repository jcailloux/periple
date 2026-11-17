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
#include <periple/core/latest_departures.hpp>
#include <periple/variants/time_windows.hpp>

#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

namespace periple::time_windows {

// Lower bound for branch_and_bound under Strict windows, around an inner bound:
// a node from which some unvisited city, or the return to the first, can no
// longer be reached before the last of its windows closes is a dead_end; any
// other node gets the inner bound's value. Leaving the last city at its
// earliest departure, the tour reaches each city no sooner than by the
// shortest path, since waiting only delays (Desrosiers, Dumas, Soumis; Dumas
// et al.). The windows are those of the one Strict in the solver's variant.
// The test times the tour under them alone: it stays exact while nothing else
// in the variant makes a time earlier, as service times only delay.
// branch_and_bound() checks that before applying it; passed explicitly, as
// branch_and_bound(time_windows::DeadlineBound(AssignmentBound{})), it is left
// to the caller. Shortest paths by Floyd-Warshall in begin, O(n^3) once, n^2
// doubles. O(n) per node.
template <typename Bound>
class DeadlineBound {
public:
	static constexpr bool assumes_nonnegative_penalties = periple::detail::assumes_nonnegative_penalties<Bound>;

	explicit DeadlineBound(Bound bound) : bound_(std::move(bound)) {}

	template <DistanceSource Dist, typename Variant, typename Stop>
	void begin(const Solver<Dist, Variant>& solver, Stop& stop) {
		using city_type = typename dist_traits<Dist>::city_type;
		static_assert(detail::timing<Variant>::stricts == 1,
			"DeadlineBound: the solver's variant must hold exactly one time_windows::Strict, whose windows the test reads");
		windows_ = &detail::strict_of(solver.variant());
		n_ = solver.size();
		due_.resize(n_);
		detail::due_times(*windows_, due_);
		const auto distance = [&solver](std::size_t a, std::size_t b) {
			return solver.distance(static_cast<city_type>(a), static_cast<city_type>(b));
		};
		// Stopped, the search ends before the first lower_bound.
		if (!periple::detail::build_latest_departures(table_, distance, due_, stop)) return;
		const auto tour = solver.tour();
		first_ = static_cast<std::size_t>(tour.front());
		in_prefix_.assign(n_, 0);
		placed_.resize(n_);
		departure_.resize(n_);
		masked_ = 0;
		for (std::size_t i = 0; i < tour.size(); ++i) {
			const auto c = static_cast<std::size_t>(tour[i]);
			const double arrival = i == 0 ? 0.0 : departure_[i - 1] + static_cast<double>(solver.distance(tour[i - 1], tour[i]));
			departure_[i] = detail::departure_time(windows_->windows(c), arrival);
			in_prefix_[c] = 1;
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
		// Depth first, the parent's prefix is marked already: cities placed
		// beyond it since leave the prefix, then the last city enters it.
		while (masked_ > depth - 1) in_prefix_[placed_[--masked_]] = 0;
		in_prefix_[last] = 1;
		placed_[masked_++] = last;
		// No O(1) pre-test: here it rarely concludes, as the least slack of a
		// row also counts the visited cities.
		return periple::detail::misses_a_due_time(table_, n_, last, departure, first_,
			[this](std::size_t c) { return in_prefix_[c] == 0; });
	}

	const Strict* windows_ = nullptr;          // the Strict of the solver's variant, set by begin
	Bound bound_;
	std::size_t n_ = 0, first_ = 0;
	std::vector<double> due_;                  // by city: the end of its last window, plus the margin
	periple::detail::LatestDepartures table_;  // from due_ and the shortest travel times
	std::vector<unsigned char> in_prefix_;     // by city: 1 for the cities of the prefix
	std::vector<std::size_t> placed_;          // by position: the cities in_prefix_ marks
	std::size_t masked_ = 0;                   // how many
	std::vector<double> departure_;            // by position: the earliest departure, under the windows alone
};

} // namespace periple::time_windows
