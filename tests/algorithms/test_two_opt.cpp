#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <solver_test_access.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <span>
#include <vector>

using namespace periple;

// ---------------------------------------------------------------------------
// Instances
// ---------------------------------------------------------------------------

struct Lcg {
	std::uint64_t state;
	std::uint32_t next() {
		state = state * 6364136223846793005ull + 1442695040888963407ull;
		return static_cast<std::uint32_t>(state >> 33);
	}
	int in(int lo, int hi) {
		return lo + static_cast<int>(next() % static_cast<std::uint32_t>(hi - lo + 1));
	}
};

auto random_sym(std::size_t n, std::uint64_t seed) {
	SymmetricDistanceMatrix<int> m(n);
	Lcg rng{seed};
	for (std::size_t i = 1; i < n; ++i)
		for (std::size_t j = 0; j < i; ++j)
			m.set(i, j, rng.in(1, 100));
	return m;
}

auto random_asym(std::size_t n, std::uint64_t seed) {
	DistanceMatrix<int> m(n);
	Lcg rng{seed};
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = 0; j < n; ++j)
			if (i != j) m(i, j) = rng.in(1, 100);
	return m;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
auto recompute_cost(const Dist& dist, std::span<const std::size_t> tour) {
	typename dist_traits<Dist>::cost_type total{};
	for (std::size_t i = 0; i < tour.size(); ++i)
		total += dist(tour[i], tour[(i + 1) % tour.size()]);
	return total;
}

template <DistanceSource Dist, typename Variant>
void check_tour(const Solver<Dist, Variant>& solver) {
	const auto n = solver.size();
	assert(solver.tour().size() == n && "two_opt must leave a complete tour");
	std::vector<bool> seen(n, false);
	for (auto c : solver.tour()) {
		assert(!seen[c] && "two_opt must leave a permutation");
		seen[c] = true;
	}
}

// Every two_opt call in this file goes through here, so run_checked applies the
// output invariants and the tour-version contract to each of them.
template <DistanceSource Dist, typename Variant>
void run_two_opt(Solver<Dist, Variant>& solver, TwoOptParams params = {}) {
	run_checked(solver, [&] { solver.two_opt(params); });
}

// Don't-look bits only reactivate the four endpoints of an applied move, while
// a directed reversal also flips the edges inside the segment: one call can
// stop short of the 2-opt fixed point. Iterating makes the oracles below exact,
// since a call that applies no move has scanned every pair with full lists.
template <DistanceSource Dist, typename Variant>
void two_opt_to_fixed_point(Solver<Dist, Variant>& solver, std::size_t neighbors = 0) {
	for (;;) {
		const auto before = solver.cost();
		run_two_opt(solver, {.neighbors = neighbors});
		if (solver.cost() == before) return;
	}
}

// ---------------------------------------------------------------------------
// Oracles
// ---------------------------------------------------------------------------

// Pairs 0 <= i < j < n cover every pair of cycle edges; a wrapping reversal is
// the complement of one of them, hence the same cycle.
template <DistanceSource Dist>
bool is_two_optimal_sym(const Dist& d, std::span<const std::size_t> t) {
	const auto n = t.size();
	for (std::size_t i = 0; i + 1 < n; ++i)
		for (std::size_t j = i + 1; j < n; ++j) {
			const auto a = t[i], b = t[i + 1], c = t[j], e = t[(j + 1) % n];
			if (d(a, c) + d(b, e) < d(a, b) + d(c, e)) return false;
		}
	return true;
}

// No non-wrapping reversal t[i+1..j] improves the tour.
template <DistanceSource Dist>
bool is_two_optimal_directed(const Dist& d, std::span<const std::size_t> t) {
	std::vector<std::size_t> v(t.begin(), t.end());
	const std::span<const std::size_t> view(v);
	const auto base = recompute_cost(d, view);
	for (std::size_t i = 0; i + 1 < v.size(); ++i)
		for (std::size_t j = i + 1; j < v.size(); ++j) {
			auto lo = v.begin() + static_cast<std::ptrdiff_t>(i) + 1;
			auto hi = v.begin() + static_cast<std::ptrdiff_t>(j) + 1;
			std::reverse(lo, hi);
			const bool better = recompute_cost(d, view) < base;
			std::reverse(lo, hi);
			if (better) return false;
		}
	return true;
}

// Restricted to the candidate lists: no reversal creating an edge a -> c with
// c among a's k nearest improves the tour. That is what the directed scan
// guarantees at its fixed point, and unlike the oracles above it distinguishes
// the two positions the candidate edge can take in a reversal.
template <DistanceSource Dist>
bool is_candidate_optimal_directed(Solver<Dist>& solver, const Dist& dist, std::size_t k) {
	std::vector<std::size_t> v(solver.tour().begin(), solver.tour().end());
	const std::span<const std::size_t> view(v);
	const auto base = recompute_cost(dist, view);
	auto improves = [&](std::size_t i, std::size_t j) {
		auto lo = v.begin() + static_cast<std::ptrdiff_t>(i) + 1;
		auto hi = v.begin() + static_cast<std::ptrdiff_t>(j) + 1;
		std::reverse(lo, hi);
		const bool better = recompute_cost(dist, view) < base;
		std::reverse(lo, hi);
		return better;
	};
	for (std::size_t a = 0; a < v.size(); ++a)
		for (auto c : solver.neighbors(a, k)) {
			const std::size_t pa = solver.position()[a];
			const std::size_t pc = solver.position()[c];
			if (pa >= pc) continue;  // edge a -> c: scanned from c
			if (improves(pa, pc)) return false;
			if (pa >= 1 && improves(pa - 1, pc - 1)) return false;
		}
	return true;
}

// Same neighborhood, scored through the variant pipeline.
template <DistanceSource Dist, typename Variant>
bool is_two_optimal_replay(const Solver<Dist, Variant>& solver) {
	std::vector<std::size_t> v(solver.tour().begin(), solver.tour().end());
	const std::span<const std::size_t> view(v);
	bool optimal = true;
	for (std::size_t i = 0; optimal && i + 1 < v.size(); ++i)
		for (std::size_t j = i + 1; j < v.size(); ++j) {
			auto lo = v.begin() + static_cast<std::ptrdiff_t>(i) + 1;
			auto hi = v.begin() + static_cast<std::ptrdiff_t>(j) + 1;
			std::reverse(lo, hi);
			const auto candidate = solver.evaluate_replay(view, 1, 0);
			std::reverse(lo, hi);
			if (candidate && *candidate < solver.cost()) { optimal = false; break; }
		}
	solver.discard_staging();
	return optimal;
}

template <DistanceSource Dist>
bool tour_respects_windows(const Dist& dist, std::span<const std::size_t> tour,
                           std::span<const time_windows::TimeWindow> windows) {
	double time = 0.0;
	for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
		if (time > windows[tour[i]].latest) return false;
		time = std::max(time, windows[tour[i]].earliest)
			+ static_cast<double>(dist(tour[i], tour[i + 1]));
	}
	return tour.empty() || time <= windows[tour.back()].latest;
}

// ---------------------------------------------------------------------------
// Symmetric mode
// ---------------------------------------------------------------------------

void test_symmetric_improves() {
	auto m = random_sym(30, 1);
	Solver solver(m);
	solver.set_symmetric(true);
	solver.nearest_neighbor();
	const auto nn_cost = solver.cost();

	run_two_opt(solver);
	check_tour(solver);
	assert(solver.status() == SolutionStatus::feasible && "two_opt keeps the tour feasible");
	assert(solver.cost() < nn_cost && "2-opt must improve a random NN tour");
	assert(solver.cost() == recompute_cost(m, solver.tour()) && "cost must stay exact");
}

void test_symmetric_two_optimal() {
	auto m = random_sym(30, 2);
	Solver solver(m);
	solver.set_symmetric(true);
	solver.nearest_neighbor();
	run_two_opt(solver, {.neighbors = 0});

	check_tour(solver);
	assert(solver.cost() == recompute_cost(m, solver.tour()) && "cost must stay exact");
	assert(is_two_optimal_sym(m, solver.tour())
		&& "full candidate lists must reach a 2-optimal tour in one call");
}

void test_repeated_call_stable() {
	auto m = random_sym(30, 3);
	Solver solver(m);
	solver.set_symmetric(true);
	solver.nearest_neighbor();
	run_two_opt(solver);

	std::vector<std::size_t> tour(solver.tour().begin(), solver.tour().end());
	const auto cost = solver.cost();
	run_two_opt(solver);

	check_tour(solver);
	assert(solver.cost() == cost && "a second call must find nothing");
	assert(std::equal(tour.begin(), tour.end(), solver.tour().begin(), solver.tour().end())
		&& "a second call must not touch the tour");
}

void test_max_moves_one() {
	auto m = random_sym(30, 4);
	Solver capped(m);
	capped.set_symmetric(true);
	capped.nearest_neighbor();
	const auto nn_cost = capped.cost();
	run_two_opt(capped, {.max_moves = 1});

	check_tour(capped);
	assert(capped.cost() < nn_cost && "max_moves = 1 must still apply one move");
	assert(capped.cost() == recompute_cost(m, capped.tour()) && "cost must stay exact");

	Solver full(m);
	full.set_symmetric(true);
	full.nearest_neighbor();
	run_two_opt(full);
	assert(capped.cost() >= full.cost() && "one move cannot beat the full descent");
}

void test_set_tour_then_two_opt() {
	auto m = random_sym(10, 5);
	std::vector<std::size_t> identity(m.size());
	std::iota(identity.begin(), identity.end(), std::size_t{0});

	Solver solver(m);
	solver.set_symmetric(true);
	solver.set_tour(identity);
	const auto start = solver.cost();
	run_two_opt(solver, {.neighbors = 0});

	check_tour(solver);
	assert(solver.cost() <= start && "two_opt must not degrade an imported tour");
	assert(solver.cost() == recompute_cost(m, solver.tour()) && "cost must stay exact");
	assert(is_two_optimal_sym(m, solver.tour()) && "imported tours reach the same fixed point");
}

void test_matrix_switch() {
	auto m1 = random_sym(8, 6);
	auto m2 = random_sym(12, 7);

	Solver solver(m1);
	solver.set_symmetric(true);
	solver.nearest_neighbor();
	run_two_opt(solver);
	solver.set_matrix(m2);
	solver.nearest_neighbor();
	run_two_opt(solver);

	check_tour(solver);
	assert(solver.cost() == recompute_cost(m2, solver.tour())
		&& "neighbor lists must be rebuilt for the new matrix");
}

// ---------------------------------------------------------------------------
// Directed modes (NoCallbacks without declared symmetry, and asymmetric)
// ---------------------------------------------------------------------------

void test_undeclared_symmetry() {
	auto m = random_sym(12, 8);
	Solver solver(m);  // symmetry not declared: directed mode on symmetric data
	solver.nearest_neighbor();
	two_opt_to_fixed_point(solver);

	check_tour(solver);
	assert(solver.cost() == recompute_cost(m, solver.tour()) && "cost must stay exact");
	assert(is_two_optimal_directed(m, solver.tour())
		&& "the directed scan must reach its fixed point");
}

void test_asymmetric_two_optimal() {
	auto m = random_asym(12, 9);
	Solver solver(m);
	solver.nearest_neighbor();
	const auto first = solver.tour()[0];
	two_opt_to_fixed_point(solver);

	check_tour(solver);
	assert(solver.tour()[0] == first && "a directed mode must keep position 0 fixed");
	assert(solver.cost() == recompute_cost(m, solver.tour())
		&& "path costs must score reversals exactly");
	assert(is_two_optimal_directed(m, solver.tour())
		&& "the directed scan must reach its fixed point");
}

// With full lists every pair is reachable from its first city alone, so only
// truncated lists tell whether both positions of a candidate edge are scanned.
// This sweep is the smallest tried that separates the two.
void test_directed_candidate_optimal() {
	constexpr std::size_t k = 5;
	for (std::uint64_t seed = 1; seed <= 40; ++seed) {
		auto m = random_asym(50, seed);
		Solver solver(m);
		solver.nearest_neighbor();
		two_opt_to_fixed_point(solver, k);

		check_tour(solver);
		assert(solver.cost() == recompute_cost(m, solver.tour()) && "cost must stay exact");
		assert(is_candidate_optimal_directed(solver, m, k)
			&& "both positions of a candidate edge must be scanned");
	}
}

void test_path_costs_versioning() {
	auto m = random_asym(12, 10);
	Solver solver(m);
	SolverTestAccess access(solver);

	solver.nearest_neighbor();
	run_two_opt(solver, {.neighbors = 0});
	assert(access.path_costs_current() && "asymmetric mode must leave the path costs valid");
	assert(access.path_costs_consistent() && "path costs must match the tour");

	std::vector<std::size_t> identity(m.size());
	std::iota(identity.begin(), identity.end(), std::size_t{0});
	solver.set_tour(identity);
	assert(!access.path_costs_current() && "set_tour must invalidate the path costs");

	run_two_opt(solver, {.neighbors = 0});
	check_tour(solver);
	assert(access.path_costs_current() && access.path_costs_consistent()
		&& "the second call must rebuild the path costs");
	assert(solver.cost() == recompute_cost(m, solver.tour()) && "cost must stay exact");
}

// ---------------------------------------------------------------------------
// Replay mode (variants)
// ---------------------------------------------------------------------------

void test_strict_keeps_feasibility() {
	auto m = random_sym(10, 11);
	const auto n = m.size();

	// Windows derived from a plain NN tour with 15% slack: NN under Strict
	// follows the same tour, while most reversals break a window, so the replay
	// filter is what keeps the tour feasible.
	Solver plain(m);
	plain.nearest_neighbor();
	std::vector<time_windows::TimeWindow> windows(n);
	windows[plain.tour()[0]] = {0.0, 1e9};  // depot, also checked on the closing edge
	double arrival = 0.0;
	for (std::size_t i = 1; i < n; ++i) {
		arrival += static_cast<double>(m(plain.tour()[i - 1], plain.tour()[i]));
		windows[plain.tour()[i]] = {0.0, arrival * 1.15};
	}

	time_windows::Strict tw{std::span<const time_windows::TimeWindow>(windows)};
	Solver solver(m, tw);
	solver.nearest_neighbor();
	assert(solver.status() == SolutionStatus::feasible && "setup: NN must stay feasible");
	const auto nn_cost = solver.cost();

	two_opt_to_fixed_point(solver);
	check_tour(solver);
	assert(solver.status() == SolutionStatus::feasible && "two_opt must keep the tour feasible");
	assert(solver.cost() <= nn_cost && "two_opt must not degrade the tour");
	assert(tour_respects_windows(m, solver.tour(), windows)
		&& "rejected candidates must never be applied");
	assert(is_two_optimal_replay(solver) && "the replay scan must reach its fixed point");
}

void test_relaxed_consistent() {
	auto m = random_sym(10, 12);
	std::vector<time_windows::TimeWindow> windows(m.size(), {0.0, 100.0});
	windows[1] = {0.0, 5.0};  // tight: penalties are unavoidable

	time_windows::Relaxed relaxed{std::span<const time_windows::TimeWindow>(windows), 1000};
	Solver solver(m, relaxed);
	solver.nearest_neighbor();
	const auto nn_cost = solver.cost();
	const auto first = solver.tour()[0];

	two_opt_to_fixed_point(solver);
	check_tour(solver);
	assert(solver.tour()[0] == first && "replay mode must keep position 0 fixed");
	assert(solver.cost() <= nn_cost && "two_opt must not degrade the penalized cost");

	std::vector<std::size_t> tour(solver.tour().begin(), solver.tour().end());
	Solver verifier(m, relaxed);
	verifier.set_tour(tour);
	assert(verifier.cost() == solver.cost() && "replay cost must match a full rebuild");
	assert(is_two_optimal_replay(solver) && "the replay scan must reach its fixed point");
}

// Fractional parts accumulate past 1: the prefix costs consumed by replay must
// truncate per edge, exactly like Solver::append.
struct FractionalPenalty {
	template <typename Ctx>
	void move_prepare(const AppendMove<std::size_t>& m, Ctx& ctx) const {
		ctx.cost_delta += 0.6 + 0.1 * static_cast<double>(m.city);
	}
};

void test_fractional_penalty_consistent() {
	auto m = random_sym(10, 13);
	FractionalPenalty variant;
	Solver solver(m, variant);
	solver.nearest_neighbor();
	two_opt_to_fixed_point(solver);

	check_tour(solver);
	std::vector<std::size_t> tour(solver.tour().begin(), solver.tour().end());
	Solver verifier(m, variant);
	verifier.set_tour(tour);
	assert(verifier.cost() == solver.cost() && "per-edge truncation must match a full rebuild");
}

void test_composed_service_strict() {
	auto m = random_sym(8, 14);
	std::vector<double> service(m.size(), 3.0);
	service_times::ServiceTimes svc{std::span<const double>(service)};
	std::vector<time_windows::TimeWindow> windows(m.size(), {0.0, 1e9});
	time_windows::Strict tw{std::span<const time_windows::TimeWindow>(windows)};
	auto variant = Composed(svc, tw);

	Solver solver(m, variant);
	solver.nearest_neighbor();
	two_opt_to_fixed_point(solver);

	check_tour(solver);
	assert(solver.status() == SolutionStatus::feasible && "composed variant must stay feasible");
	auto self = solver.evaluate_replay(solver.tour(), 1, 0);
	solver.discard_staging();
	assert(self && *self == solver.cost() && "composed cost must match a full replay");
}

// ---------------------------------------------------------------------------
// Degenerate cases
// ---------------------------------------------------------------------------

void test_trivial_sizes() {
	for (std::size_t n = 0; n <= 3; ++n) {
		auto m = random_sym(n, 15);
		Solver solver(m);
		solver.set_symmetric(true);
		solver.nearest_neighbor();

		std::vector<std::size_t> tour(solver.tour().begin(), solver.tour().end());
		const auto cost = solver.cost();
		run_two_opt(solver);

		assert(solver.status() == SolutionStatus::feasible && "small instances stay feasible");
		assert(solver.cost() == cost && "no 2-opt move exists below 4 cities");
		assert(std::equal(tour.begin(), tour.end(), solver.tour().begin(), solver.tour().end())
			&& "the tour must be untouched");
	}
}

void test_optimal_is_noop() {
	auto m = random_sym(6, 16);
	Solver solver(m);
	solver.set_symmetric(true);
	solver.held_karp();

	std::vector<std::size_t> tour(solver.tour().begin(), solver.tour().end());
	const auto cost = solver.cost();
	run_two_opt(solver);

	assert(solver.status() == SolutionStatus::optimal && "two_opt must not downgrade an optimal tour");
	assert(solver.cost() == cost && "an optimal tour cannot be improved");
	assert(std::equal(tour.begin(), tour.end(), solver.tour().begin(), solver.tour().end())
		&& "the tour must be untouched");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	struct Test { const char* name; void (*fn)(); };
	Test tests[] = {
		// Symmetric mode
		{"symmetric_improves",           test_symmetric_improves},
		{"symmetric_two_optimal",        test_symmetric_two_optimal},
		{"repeated_call_stable",         test_repeated_call_stable},
		{"max_moves_one",                test_max_moves_one},
		{"set_tour_then_two_opt",        test_set_tour_then_two_opt},
		{"matrix_switch",                test_matrix_switch},
		// Directed modes
		{"undeclared_symmetry",          test_undeclared_symmetry},
		{"asymmetric_two_optimal",       test_asymmetric_two_optimal},
		{"directed_candidate_optimal",   test_directed_candidate_optimal},
		{"path_costs_versioning",        test_path_costs_versioning},
		// Replay mode
		{"strict_keeps_feasibility",     test_strict_keeps_feasibility},
		{"relaxed_consistent",           test_relaxed_consistent},
		{"fractional_penalty_consistent", test_fractional_penalty_consistent},
		{"composed_service_strict",      test_composed_service_strict},
		// Degenerate cases
		{"trivial_sizes",                test_trivial_sizes},
		{"optimal_is_noop",              test_optimal_is_noop},
	};

	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}
