#include <periple/algorithms/registry.hpp>
#include <periple/distance/matrix.hpp>
#include <periple/variants/time_windows.hpp>
#include <solver_test_access.hpp>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <optional>
#include <vector>

using namespace periple;

// ---------------------------------------------------------------------------
// Test matrices (small enough for HK)
// ---------------------------------------------------------------------------

auto make_sym4() {
	return SymmetricDistanceMatrix<int>({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
}

auto make_sym5() {
	return SymmetricDistanceMatrix<int>({
		{ 0,  3,  4,  2,  7},
		{ 3,  0,  4,  6,  3},
		{ 4,  4,  0,  5,  8},
		{ 2,  6,  5,  0,  6},
		{ 7,  3,  8,  6,  0}
	});
}

auto make_asym4() {
	return DistanceMatrix<int>(4, {
		 0,  5,  8, 12,
		15,  0, 10,  7,
		 6, 14,  0,  9,
		11,  3, 13,  0
	});
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
void assert_valid_tour(const Dist& dist,
                       std::span<const typename dist_traits<Dist>::city_type> tour)
{
	const auto n = dist.size();
	assert(tour.size() == n);
	std::vector<bool> seen(n, false);
	for (auto c : tour) {
		auto i = static_cast<std::size_t>(c);
		assert(i < n && !seen[i]);
		seen[i] = true;
	}
}

// Compute pure distance cost (no penalties) for a complete tour.
template <DistanceSource Dist>
auto pure_distance_cost(const Dist& dist, std::span<const typename dist_traits<Dist>::city_type> tour) -> typename dist_traits<Dist>::cost_type {
	using cost_type = typename dist_traits<Dist>::cost_type;
	cost_type total{};
	for (std::size_t i = 0; i < tour.size(); ++i)
		total += dist(tour[i], tour[(i + 1) % tour.size()]);
	return total;
}

// Verify that arrival times respect all time windows.
template <DistanceSource Dist>
bool tour_respects_windows(
	const Dist& dist,
	std::span<const typename dist_traits<Dist>::city_type> tour,
	std::span<const time_windows::TimeWindow> windows)
{
	double time = 0.0;
	for (std::size_t i = 0; i < tour.size(); ++i) {
		auto ci = static_cast<std::size_t>(tour[i]);
		if (!time_windows::detail::is_feasible(std::span<const time_windows::TimeWindow>(&windows[ci], 1), time))
			return false;
		time = std::max(time, windows[ci].earliest);
		auto next = tour[(i + 1) % tour.size()];
		time += static_cast<double>(dist(tour[i], next));
	}
	return true;
}

// ---------------------------------------------------------------------------
// TSPTW Strict x all algorithms
// ---------------------------------------------------------------------------

template <typename Algo, DistanceSource Dist>
void test_strict(const Algo& algo, const Dist& dist, std::span<const time_windows::TimeWindow> windows) {
	time_windows::Strict tw(windows);
	Solver solver(dist, tw);
	run_checked(solver, [&] { algo(solver); });

	if (solver.status() == SolutionStatus::partial ||
	    solver.status() == SolutionStatus::infeasible) {
		// Some cities couldn't be placed (strict filter rejected everything).
		// Valid partial tour: all placed cities are unique.
		std::vector<bool> seen(dist.size(), false);
		for (auto c : solver.tour()) {
			auto i = static_cast<std::size_t>(c);
			assert(!seen[i]);
			seen[i] = true;
		}
		return;
	}

	assert_valid_tour(dist, solver.tour());
	assert(tour_respects_windows(dist, solver.tour(), windows));
}

template <DistanceSource Dist>
void run_strict(const Dist& dist) {
	const auto n = dist.size();
	// Generous windows: city i has window [0, 200 + 50*i].
	// All tours should be feasible.
	std::vector<time_windows::TimeWindow> windows(n);
	for (std::size_t i = 0; i < n; ++i)
		windows[i] = {0.0, 200.0 + 50.0 * static_cast<double>(i)};

	for_each_algorithm(nullptr, [&](const auto& algo) {
		test_strict(algo, dist, windows);
	});
}

// Zero-width windows: nothing but the start city can be reached in time, so the
// construction stops after one city. An algorithm that improves a tour must
// check that it got one, two_opt requiring a complete feasible tour.
template <DistanceSource Dist>
void run_strict_partial(const Dist& dist) {
	std::vector<time_windows::TimeWindow> windows(dist.size(), {0.0, 0.0});

	for_each_algorithm(nullptr, [&](const auto& algo) {
		test_strict(algo, dist, windows);
	});
}

// ---------------------------------------------------------------------------
// TSPTW Relaxed x all algorithms
// ---------------------------------------------------------------------------

template <typename Algo, DistanceSource Dist>
void test_relaxed(const Algo& algo, const Dist& dist, std::span<const time_windows::TimeWindow> windows, int penalty_weight) {
	time_windows::Relaxed relaxed(windows, penalty_weight);
	Solver solver(dist, relaxed);
	run_checked(solver, [&] { algo(solver); });

	assert_valid_tour(dist, solver.tour());

	// Cost must be >= pure distance cost (penalties are non-negative).
	auto dist_cost = pure_distance_cost(dist, solver.tour());
	assert(solver.cost() >= dist_cost && "relaxed cost must be >= pure distance cost");

	// Self-consistency: set_tour should recompute the same cost.
	using city_type = typename dist_traits<Dist>::city_type;
	std::vector<city_type> tour_copy(solver.tour().begin(), solver.tour().end());
	Solver verifier(dist, relaxed);
	verifier.set_tour(tour_copy);
	assert(verifier.cost() == solver.cost() && "relaxed cost must be self-consistent");
}

template <DistanceSource Dist>
void run_relaxed(const Dist& dist) {
	const auto n = dist.size();
	// Tight window on city 1 to trigger penalties.
	std::vector<time_windows::TimeWindow> windows(n);
	for (std::size_t i = 0; i < n; ++i)
		windows[i] = {0.0, 100.0};
	if (n > 1)
		windows[1] = {0.0, 5.0};  // tight

	for_each_algorithm(nullptr, [&](const auto& algo) {
		test_relaxed(algo, dist, windows, 1000);
	});
}

// ---------------------------------------------------------------------------
// TSPTW Strict with optional windows x all algorithms
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
void run_strict_optional(const Dist& dist) {
	const auto n = dist.size();
	// City 0 has a generous window, others are unconstrained.
	std::vector<std::optional<time_windows::TimeWindow>> windows(n, std::nullopt);
	if (n > 0) windows[0] = time_windows::TimeWindow{0.0, 500.0};

	for_each_algorithm(nullptr, [&](const auto& algo) {
		auto tw_span = std::span<const std::optional<time_windows::TimeWindow>>(windows);
		time_windows::Strict tw(tw_span);
		Solver solver(dist, tw);
		run_checked(solver, [&] { algo(solver); });

		// All cities unconstrained except city 0 with generous window.
		// Should always find a full tour.
		assert_valid_tour(dist, solver.tour());
	});
}

template <DistanceSource Dist>
void run_relaxed_optional(const Dist& dist) {
	const auto n = dist.size();
	// City 1 has a tight window, others unconstrained.
	std::vector<std::optional<time_windows::TimeWindow>> windows(n, std::nullopt);
	if (n > 1) windows[1] = time_windows::TimeWindow{0.0, 5.0};

	for_each_algorithm(nullptr, [&](const auto& algo) {
		time_windows::Relaxed relaxed(std::span<const std::optional<time_windows::TimeWindow>>(windows), 1000);
		Solver solver(dist, relaxed);
		run_checked(solver, [&] { algo(solver); });

		assert_valid_tour(dist, solver.tour());

		// Self-consistency.
		using city_type = typename dist_traits<Dist>::city_type;
		std::vector<city_type> tour_copy(solver.tour().begin(), solver.tour().end());
		Solver verifier(dist, relaxed);
		verifier.set_tour(tour_copy);
		assert(verifier.cost() == solver.cost() && "relaxed optional cost must be self-consistent");
	});
}

// ---------------------------------------------------------------------------
// An infeasible exact solve drops the tour, which must bump tour_version_
// ---------------------------------------------------------------------------

void run_infeasible_held_karp() {
	auto dist = make_sym4();
	// Zero-width windows: nothing but the depot is reachable in time.
	std::vector<time_windows::TimeWindow> windows(dist.size(), {0.0, 0.0});
	time_windows::Strict tw{std::span<const time_windows::TimeWindow>(windows)};
	Solver solver(dist, tw);

	std::vector<std::size_t> prefix = {0, 2};
	solver.set_tour(prefix);
	run_checked(solver, [&] { solver.held_karp(); });

	assert(solver.status() == SolutionStatus::infeasible && "setup: no complete tour is feasible");
	assert(solver.tour().empty() && "an infeasible exact solve drops the tour");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	std::printf("tsptw_strict_sym4 ... ");
	std::fflush(stdout);
	run_strict(make_sym4());
	std::printf("OK\n");

	std::printf("tsptw_strict_sym5 ... ");
	std::fflush(stdout);
	run_strict(make_sym5());
	std::printf("OK\n");

	std::printf("tsptw_strict_asym4 ... ");
	std::fflush(stdout);
	run_strict(make_asym4());
	std::printf("OK\n");

	std::printf("tsptw_strict_partial_sym5 ... ");
	std::fflush(stdout);
	run_strict_partial(make_sym5());
	std::printf("OK\n");

	std::printf("tsptw_relaxed_sym4 ... ");
	std::fflush(stdout);
	run_relaxed(make_sym4());
	std::printf("OK\n");

	std::printf("tsptw_relaxed_sym5 ... ");
	std::fflush(stdout);
	run_relaxed(make_sym5());
	std::printf("OK\n");

	std::printf("tsptw_relaxed_asym4 ... ");
	std::fflush(stdout);
	run_relaxed(make_asym4());
	std::printf("OK\n");

	std::printf("tsptw_strict_optional_sym4 ... ");
	std::fflush(stdout);
	run_strict_optional(make_sym4());
	std::printf("OK\n");

	std::printf("tsptw_relaxed_optional_sym4 ... ");
	std::fflush(stdout);
	run_relaxed_optional(make_sym4());
	std::printf("OK\n");

	std::printf("tsptw_infeasible_held_karp ... ");
	std::fflush(stdout);
	run_infeasible_held_karp();
	std::printf("OK\n");
}