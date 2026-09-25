#include <periple/algorithms/registry.hpp>
#include <periple/distance/matrix.hpp>
#include <periple/variants/time_windows.hpp>
#include <solver_test_access.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
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
// Oracle variants x all algorithms: constrained optimum known by brute force
// ---------------------------------------------------------------------------

// Adds a cost per directed edge.
struct EdgeSurcharge {
	const DistanceMatrix<int>* extra;

	template <typename CityT, typename Ctx>
	void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const {
		if (m.pos > 0) ctx.cost_delta += (*extra)(m.prev_city, m.city);
	}

	template <typename CityT, typename Ctx>
	void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const {
		ctx.cost_delta += (*extra)(m.from, m.to);
	}
};

// Rejects some directed edges.
struct ForbiddenEdges {
	const std::vector<std::uint8_t>* banned;  // banned[a * n + b]
	std::size_t n;

	bool allows(std::size_t a, std::size_t b) const { return !(*banned)[a * n + b]; }

	template <typename CityT>
	bool move_filter(const AppendMove<CityT>& m) const {
		return m.pos == 0 || allows(static_cast<std::size_t>(m.prev_city), static_cast<std::size_t>(m.city));
	}

	template <typename CityT>
	bool move_filter(const DPMove<CityT>& m) const {
		return allows(static_cast<std::size_t>(m.from), static_cast<std::size_t>(m.to));
	}
};

// Cheapest tour from city 0 under edge_cost, among those whose every edge is
// allowed; nullopt if there is none.
template <typename EdgeCost, typename Allows>
std::optional<int> brute_force_constrained(std::size_t n, EdgeCost edge_cost, Allows allows) {
	std::vector<std::size_t> tour(n);
	std::iota(tour.begin(), tour.end(), std::size_t{0});
	std::optional<int> best;
	do {
		int cost = 0;
		bool admitted = true;
		for (std::size_t i = 0; i < n && admitted; ++i) {
			const auto a = tour[i], b = tour[(i + 1) % n];
			admitted = allows(a, b);
			cost += edge_cost(a, b);
		}
		if (admitted && (!best || cost < *best)) best = cost;
	} while (std::next_permutation(tour.begin() + 1, tour.end()));
	return best;
}

template <typename Algo>
void test_oracles(const Algo& algo, const DistanceMatrix<int>& dist,
                  const DistanceMatrix<int>& extra, const std::vector<std::uint8_t>& banned) {
	const auto n = dist.size();
	auto surcharged = [&](std::size_t a, std::size_t b) { return dist(a, b) + extra(a, b); };
	auto plain = [&](std::size_t a, std::size_t b) { return dist(a, b); };

	EdgeSurcharge surcharge{&extra};
	Solver s1(dist, surcharge);
	run_checked(s1, [&] { algo(s1); });
	assert_valid_tour(dist, s1.tour());
	int tour_cost = 0;
	for (std::size_t i = 0; i < n; ++i)
		tour_cost += surcharged(s1.tour()[i], s1.tour()[(i + 1) % n]);
	assert(s1.cost() == tour_cost && "the reported cost must include every edge's move_prepare surcharge");
	if constexpr (Algo::is_exact)
		assert(s1.cost() == *brute_force_constrained(n, surcharged, [](std::size_t, std::size_t) { return true; })
			&& "an exact algorithm must minimize the cost move_prepare defines");

	ForbiddenEdges forbidden{&banned, n};
	Solver s2(dist, forbidden);
	run_checked(s2, [&] { algo(s2); });
	if (s2.status() == SolutionStatus::feasible || s2.status() == SolutionStatus::optimal) {
		for (std::size_t i = 0; i < n; ++i)
			assert(forbidden.allows(s2.tour()[i], s2.tour()[(i + 1) % n])
				&& "a complete tour must not use an edge move_filter rejects");
	}
	if constexpr (Algo::is_exact) {
		const auto best = brute_force_constrained(n, plain,
			[&](std::size_t a, std::size_t b) { return forbidden.allows(a, b); });
		assert((best ? s2.status() == SolutionStatus::optimal && s2.cost() == *best
		             : s2.status() == SolutionStatus::infeasible)
			&& "an exact algorithm must reach the optimum over the tours move_filter admits, or report infeasible");
	}
}

// Random asymmetric instances, n = 3..6, each directed edge banned with probability 1/3.
void run_oracles() {
	std::uint64_t x = 0x9E3779B97F4A7C15ull;
	auto next = [&x] { x ^= x << 13; x ^= x >> 7; x ^= x << 17; return x; };
	auto random_matrix = [&](std::size_t n, int hi) {
		std::vector<int> w(n * n, 0);
		for (std::size_t i = 0; i < n; ++i)
			for (std::size_t j = 0; j < n; ++j)
				if (i != j) w[i * n + j] = static_cast<int>(next() % static_cast<std::uint64_t>(hi)) + 1;
		return DistanceMatrix<int>(n, w);
	};

	for (std::size_t n = 3; n <= 6; ++n) {
		for (int seed = 0; seed < 5; ++seed) {
			auto dist = random_matrix(n, 50);
			auto extra = random_matrix(n, 100);
			std::vector<std::uint8_t> banned(n * n, 0);
			for (std::size_t i = 0; i < n; ++i)
				for (std::size_t j = 0; j < n; ++j)
					if (i != j && next() % 3 == 0) banned[i * n + j] = 1;

			for_each_algorithm(nullptr, [&](const auto& algo) {
				test_oracles(algo, dist, extra, banned);
			});
		}
	}
}

// Deadlines only (earliest = 0): no waiting, so arrival equals the distance
// travelled and the cheapest state is also the earliest one. Exercises a
// dimension (RouteTiming) whose state must propagate from move to move.
template <typename Algo>
void test_deadline_oracle(const Algo& algo, const DistanceMatrix<int>& dist,
                          std::span<const time_windows::TimeWindow> windows) {
	const auto n = dist.size();
	std::vector<std::size_t> tour(n);
	std::iota(tour.begin(), tour.end(), std::size_t{0});
	std::optional<int> best;
	do {
		int arrival = 0;
		bool on_time = true;
		for (std::size_t i = 1; i <= n && on_time; ++i) {
			arrival += dist(tour[i - 1], tour[i % n]);
			on_time = arrival <= windows[tour[i % n]].latest;
		}
		if (on_time && (!best || arrival < *best)) best = arrival;
	} while (std::next_permutation(tour.begin() + 1, tour.end()));

	time_windows::Strict tw(windows);
	Solver solver(dist, tw);
	run_checked(solver, [&] { algo(solver); });
	if (solver.status() == SolutionStatus::feasible || solver.status() == SolutionStatus::optimal)
		assert(tour_respects_windows(dist, solver.tour(), windows)
			&& "a complete tour must meet every deadline the dimension tracks");
	if constexpr (Algo::is_exact)
		assert((best ? solver.status() == SolutionStatus::optimal && solver.cost() == *best
		             : solver.status() == SolutionStatus::infeasible)
			&& "an exact algorithm must reach the optimum over the tours that meet their deadlines, or report infeasible");
}

// Deadlines set just after the arrival times of a random tour: that tour stays
// feasible, and the unconstrained optimum usually is not.
void run_deadline_oracles() {
	std::uint64_t x = 0xD1B54A32D192ED03ull;
	auto next = [&x] { x ^= x << 13; x ^= x >> 7; x ^= x << 17; return x; };
	for (std::size_t n = 3; n <= 6; ++n) {
		for (int seed = 0; seed < 5; ++seed) {
			std::vector<int> w(n * n, 0);
			for (std::size_t i = 0; i < n; ++i)
				for (std::size_t j = 0; j < n; ++j)
					if (i != j) w[i * n + j] = static_cast<int>(next() % 50) + 1;
			DistanceMatrix<int> dist(n, w);

			std::vector<std::size_t> witness(n);
			std::iota(witness.begin(), witness.end(), std::size_t{0});
			for (std::size_t i = n - 1; i > 1; --i)
				std::swap(witness[i], witness[1 + next() % i]);

			std::vector<time_windows::TimeWindow> windows(n);
			windows[0] = {0.0, 1e9};  // the depot never closes
			int arrival = 0;
			for (std::size_t i = 1; i < n; ++i) {
				arrival += dist(witness[i - 1], witness[i]);
				windows[witness[i]] = {0.0, static_cast<double>(arrival + static_cast<int>(next() % 10))};
			}

			for_each_algorithm(nullptr, [&](const auto& algo) {
				test_deadline_oracle(algo, dist, windows);
			});
		}
	}
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

	std::printf("variant_oracles ... ");
	std::fflush(stdout);
	run_oracles();
	std::printf("OK\n");

	std::printf("deadline_oracles ... ");
	std::fflush(stdout);
	run_deadline_oracles();
	std::printf("OK\n");
}