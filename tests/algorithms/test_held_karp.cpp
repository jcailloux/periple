#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <solver_test_access.hpp>
#include "variant_registry.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>

using namespace periple;

// ---------------------------------------------------------------------------
// Test variants
// ---------------------------------------------------------------------------

// Visited-set bitmask of the placed prefix.
template <typename Ctx>
std::size_t placed_mask(const Ctx& ctx) {
	std::size_t mask = 0;
	for (auto c : ctx.tour()) mask |= std::size_t{1} << static_cast<std::size_t>(c);
	return mask;
}

// Urgent cities before the others, rejected as early as the constraint allows.
struct StrongPrecedence {
	std::size_t urgent;  // bitmask of urgent cities

	bool admits(std::size_t to, std::size_t set) const {
		return ((urgent >> to) & 1) || (set & urgent) == urgent;
	}

	template <typename CityT, typename Ctx>
	bool move_filter(const AppendMove<CityT>& m, const Ctx& ctx) const {
		return admits(static_cast<std::size_t>(m.city), placed_mask(ctx));
	}

	template <typename CityT>
	bool move_filter(const DPMove<CityT>& m) const {
		return admits(static_cast<std::size_t>(m.to), m.set);
	}
};

// Same admissible tours, rejected only once an urgent city would come too late.
struct WeakPrecedence {
	std::size_t urgent;
	std::size_t others;  // bitmask of non-urgent cities, depot excluded

	bool admits(std::size_t to, std::size_t set) const {
		return !((urgent >> to) & 1) || (set & others) == 0;
	}

	template <typename CityT, typename Ctx>
	bool move_filter(const AppendMove<CityT>& m, const Ctx& ctx) const {
		return admits(static_cast<std::size_t>(m.city), placed_mask(ctx));
	}

	template <typename CityT>
	bool move_filter(const DPMove<CityT>& m) const {
		return admits(static_cast<std::size_t>(m.to), m.set);
	}
};

// City j only once 1..j-1 are placed; the depot closes the tour.
struct ChainPrecedence {
	bool admits(std::size_t to, std::size_t set) const {
		const std::size_t before = (std::size_t{1} << to) - 1;
		return to == 0 || (set & before) == before;
	}

	template <typename CityT, typename Ctx>
	bool move_filter(const AppendMove<CityT>& m, const Ctx& ctx) const {
		return admits(static_cast<std::size_t>(m.city), placed_mask(ctx));
	}

	template <typename CityT>
	bool move_filter(const DPMove<CityT>& m) const {
		return admits(static_cast<std::size_t>(m.to), m.set);
	}
};

// Rejects every move into city 1, so no complete tour exists.
struct Unreachable1 {
	template <typename CityT>
	bool move_filter(const AppendMove<CityT>& m) const { return m.city != 1; }

	template <typename CityT>
	bool move_filter(const DPMove<CityT>& m) const { return m.to != 1; }
};

// ---------------------------------------------------------------------------
// Default engine, per variant
// ---------------------------------------------------------------------------

template <typename Variant>
using engine_of = detail::default_hk_engine<Variant,
	typename Solver<DistanceMatrix<int>, Variant>::context_type, std::size_t>;

static_assert(std::is_same_v<engine_of<NoCallbacks>, AllSets>, "held_karp(): without a variant nothing prunes, so AllSets");
static_assert(std::is_same_v<engine_of<service_times::ServiceTimes>, AllSets>, "held_karp(): ServiceTimes filters nothing, so AllSets");
static_assert(std::is_same_v<engine_of<time_windows::Strict>, ReachableSets>, "held_karp(): Strict filters DPMove, so ReachableSets");
static_assert(std::is_same_v<engine_of<time_windows::Relaxed>, AllSets>, "held_karp(): Relaxed filters nothing, so AllSets");
static_assert(std::is_same_v<engine_of<Composed<service_times::ServiceTimes, time_windows::Strict>>, ReachableSets>,
	"held_karp(): a Composed filters through any of its components, so ReachableSets");
static_assert(std::is_same_v<engine_of<StrongPrecedence>, ReachableSets>, "held_karp(): a variant of one's own that filters DPMove gets ReachableSets");

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Cost of a tour through the AppendMove pipeline of a fresh solver, without a
// variant when none is passed; nullopt if a move_filter rejects one of its
// moves or its closing edge.
template <typename... Variant>
std::optional<int> pipeline_cost(const DistanceMatrix<int>& dist, std::span<const std::size_t> tour,
                                 const Variant&... variant) {
	std::vector<std::size_t> cities(tour.begin(), tour.end());
	std::sort(cities.begin(), cities.end());
	assert(cities.size() == dist.size() && tour[0] == 0
		&& std::adjacent_find(cities.begin(), cities.end()) == cities.end()
		&& "a complete tour must visit every city once, from city 0");
	Solver s(dist, variant...);
	s.append(tour[0]);
	for (std::size_t k = 1; k < tour.size(); ++k) {
		if (!s.can_append(tour[k])) return std::nullopt;
		s.append(tour[k]);
	}
	if (s.status() != SolutionStatus::feasible) return std::nullopt;
	return s.cost();
}

// A complete tour from either engine passes every filter of the AppendMove
// pipeline at the cost reported; both engines reach the same status and cost.
template <typename... Variant>
void check_engines_agree(const DistanceMatrix<int>& dist, const Variant&... variant) {
	Solver all(dist, variant...);
	Solver reach(dist, variant...);
	run_checked(all, [&] { all.held_karp(AllSets{}); });
	run_checked(reach, [&] { reach.held_karp(ReachableSets{}); });
	assert(all.status() == reach.status() && "AllSets and ReachableSets must reach the same status");
	assert(all.cost() == reach.cost() && "AllSets and ReachableSets must reach the same cost");
	for (const auto* s : {&all, &reach}) {
		if (s->status() != SolutionStatus::optimal) continue;
		const auto cost = pipeline_cost(dist, s->tour(), variant...);
		assert(cost && *cost == s->cost() && "an optimal tour must pass the AppendMove pipeline at the cost reported");
	}
}

// Cheapest tour from city 0 among those the precedence admits, by brute force.
template <typename Precedence>
int brute_force_precedence(const DistanceMatrix<int>& dist, const Precedence& precedence) {
	const auto n = dist.size();
	std::vector<std::size_t> tour(n);
	std::iota(tour.begin(), tour.end(), std::size_t{0});
	int best = std::numeric_limits<int>::max();
	do {
		std::size_t set = 1;
		int cost = 0;
		bool admitted = true;
		for (std::size_t i = 1; i < n && admitted; ++i) {
			admitted = precedence.admits(tour[i], set);
			set |= std::size_t{1} << tour[i];
			cost += dist(tour[i - 1], tour[i]);
		}
		if (admitted) best = std::min(best, cost + dist(tour[n - 1], tour[0]));
	} while (std::next_permutation(tour.begin() + 1, tour.end()));
	return best;
}

// Random asymmetric matrix, weights in [1, max_weight].
DistanceMatrix<int> random_matrix(test::Rng& rng, std::size_t n, int max_weight) {
	std::vector<int> w(n * n, 0);
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = 0; j < n; ++j)
			if (i != j) w[i * n + j] = rng.below(max_weight) + 1;
	return DistanceMatrix<int>(n, w);
}

// Cities 1..u as a bitmask.
std::size_t first_cities(std::size_t u) {
	return ((std::size_t{1} << u) - 1) << 1;
}

// Every city but the depot as a bitmask.
std::size_t all_but_depot(std::size_t n) {
	return ((std::size_t{1} << n) - 1) & ~std::size_t{1};
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// n = 4..10, 10 instances per size: without a variant, under every shipped
// variant, and under both precedences with cities 1..(n-1)/2 urgent.
void test_engines_agree() {
	test::Rng rng{0x2545F4914F6CDD1Dull};
	for (std::size_t n = 4; n <= 10; ++n) {
		for (int seed = 0; seed < 10; ++seed) {
			std::vector<std::size_t> witness(n);
			std::iota(witness.begin(), witness.end(), std::size_t{0});
			for (std::size_t i = n - 1; i > 1; --i)
				std::swap(witness[i], witness[1 + rng() % i]);
			test::Instance in{random_matrix(rng, n, 30), witness, rng};

			check_engines_agree(in.dist);
			test::for_each_shipped_variant([&]<typename Entry>(std::type_identity<Entry>) {
				const Entry entry(in);
				check_engines_agree(in.dist, entry.variant);
			});
			const std::size_t urgent = first_cities((n - 1) / 2);
			check_engines_agree(in.dist, StrongPrecedence{urgent});
			check_engines_agree(in.dist, WeakPrecedence{urgent, all_but_depot(n) & ~urgent});
		}
	}
}

// n = 4..8, 10 instances per size, a random set of urgent cities.
void test_precedence_brute_force() {
	test::Rng rng{0x9E3779B97F4A7C15ull};
	for (std::size_t n = 4; n <= 8; ++n) {
		for (int seed = 0; seed < 10; ++seed) {
			const auto dist = random_matrix(rng, n, 100);
			const std::size_t urgent = static_cast<std::size_t>(rng()) & all_but_depot(n);
			const StrongPrecedence strong{urgent};
			const WeakPrecedence weak{urgent, all_but_depot(n) & ~urgent};
			auto check = [&](const auto& precedence, auto engine) {
				Solver s(dist, precedence);
				run_checked(s, [&] { s.held_karp(engine); });
				assert(s.status() == SolutionStatus::optimal && "urgent cities first is always admitted");
				assert(s.cost() == brute_force_precedence(dist, precedence)
					&& "held_karp must reach the cheapest tour the precedence admits");
			};
			check(strong, AllSets{});
			check(strong, ReachableSets{});
			check(weak, AllSets{});
			check(weak, ReachableSets{});
		}
	}
}

// u = 4 urgent cities, o = 5 others. Strong: {0} with any subset of the urgent
// cities (2^u), then all of them with a nonempty subset of the others
// (2^o - 1). Weak: every set, the urgent cities of a set placed first.
void test_reachable_sets() {
	test::Rng rng{0xD1B54A32D192ED03ull};
	const std::size_t u = 4, o = 5, n = 1 + u + o;
	const auto dist = random_matrix(rng, n, 100);
	const std::size_t urgent = first_cities(u);

	StrongPrecedence strong{urgent};
	Solver s1(dist, strong);
	s1.held_karp(ReachableSets{});
	assert(s1.status() == SolutionStatus::optimal && "urgent cities first is always admitted");
	assert(SolverTestAccess(s1).hk_slots() == (std::size_t{1} << u) + (std::size_t{1} << o) - 1
		&& "ReachableSets must create only the sets the strong precedence admits");

	WeakPrecedence weak{urgent, all_but_depot(n) & ~urgent};
	Solver s2(dist, weak);
	s2.held_karp(ReachableSets{});
	assert(s2.status() == SolutionStatus::optimal && "urgent cities first is always admitted");
	assert(SolverTestAccess(s2).hk_slots() == std::size_t{1} << (n - 1)
		&& "under the weak precedence every set is reached");
}

// A chain admits one tour; held_karp() takes ReachableSets, which takes up to
// 64 cities and reaches one set per size.
void test_beyond_25_cities() {
	test::Rng rng{0xBF58476D1CE4E5B9ull};
	const std::size_t n = 40;
	const auto dist = random_matrix(rng, n, 1000);
	ChainPrecedence chain;
	Solver s(dist, chain);
	run_checked(s, [&] { s.held_karp(); });
	assert(s.status() == SolutionStatus::optimal && "the chain admits one tour");
	for (std::size_t k = 0; k < n; ++k)
		assert(s.tour()[k] == k && "the chain admits only 0, 1, ..., n-1");
	assert(SolverTestAccess(s).hk_slots() == n && "the chain reaches one set per size");
}

// No complete tour: both engines report infeasible and leave no tour, even
// after a partial one.
void test_infeasible() {
	test::Rng rng{0x94D049BB133111EBull};
	const auto dist = random_matrix(rng, 5, 30);
	Unreachable1 variant;
	auto check = [&](auto engine) {
		Solver s(dist, variant);
		const std::vector<std::size_t> partial{0, 2};
		s.set_tour(partial);
		run_checked(s, [&] { s.held_karp(engine); });
		assert(s.status() == SolutionStatus::infeasible && "no tour reaches city 1");
		assert(s.tour().empty() && "an infeasible run leaves no tour");
	};
	check(AllSets{});
	check(ReachableSets{});
}

// One solver under Strict alternates engines and sizes; each run matches a
// fresh solver.
void test_reuse() {
	test::Rng rng{0x2545F4914F6CDD1Dull};
	const auto small = random_matrix(rng, 5, 30);
	const auto large = random_matrix(rng, 8, 30);
	std::vector<time_windows::TimeWindow> windows(large.size(), {0.0, 1e9});
	time_windows::Strict tw{std::span<const time_windows::TimeWindow>(windows)};

	Solver solver(small, tw);
	auto run = [&](const DistanceMatrix<int>& dist, auto engine) {
		run_checked(solver, [&] { solver.held_karp(engine); });
		Solver fresh(dist, tw);
		fresh.held_karp(engine);
		assert(solver.status() == SolutionStatus::optimal && "wide windows admit every tour");
		assert(solver.cost() == fresh.cost() && "a reused solver must match a fresh one");
	};
	run(small, AllSets{});
	solver.set_matrix(large);
	run(large, ReachableSets{});
	solver.set_matrix(small);
	run(small, AllSets{});
	run(small, ReachableSets{});
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	std::printf("engines_agree ... ");
	std::fflush(stdout);
	test_engines_agree();
	std::printf("OK\n");

	std::printf("precedence_brute_force ... ");
	std::fflush(stdout);
	test_precedence_brute_force();
	std::printf("OK\n");

	std::printf("reachable_sets ... ");
	std::fflush(stdout);
	test_reachable_sets();
	std::printf("OK\n");

	std::printf("beyond_25_cities ... ");
	std::fflush(stdout);
	test_beyond_25_cities();
	std::printf("OK\n");

	std::printf("infeasible ... ");
	std::fflush(stdout);
	test_infeasible();
	std::printf("OK\n");

	std::printf("reuse ... ");
	std::fflush(stdout);
	test_reuse();
	std::printf("OK\n");
}
