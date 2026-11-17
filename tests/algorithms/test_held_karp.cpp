#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <solver_test_access.hpp>
#include "variant_registry.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
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

// A variant of the user's that makes times earlier: leaving city c takes cut[c]
// off the departure, as a faster road would.
struct Shortcut {
	using dimension = RouteTiming;
	std::vector<double> cut;

	template <typename CityT, typename Ctx>
	void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const {
		ctx.template dim<RouteTiming>().departure -= cut[static_cast<std::size_t>(m.city)];
	}

	template <typename CityT, typename Ctx>
	void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const {
		ctx.template dim<RouteTiming>().departure -= cut[static_cast<std::size_t>(m.to)];
	}
};

// A matrix that spins on every read, as a slow distance source would.
struct SlowDistance {
	using cost_type = int;
	using city_type = std::size_t;
	const DistanceMatrix<int>* dist;
	std::chrono::microseconds spin;

	int operator()(std::size_t a, std::size_t b) const {
		const auto end = std::chrono::steady_clock::now() + spin;
		while (std::chrono::steady_clock::now() < end) {}
		return (*dist)(a, b);
	}
	std::size_t size() const { return dist->size(); }
};

// ---------------------------------------------------------------------------
// Default engine, per variant
// ---------------------------------------------------------------------------

template <typename Variant>
using engine_of = detail::default_hk_engine<Variant,
	typename Solver<DistanceMatrix<int>, Variant>::context_type, std::size_t>;

static_assert(std::is_same_v<engine_of<NoCallbacks>, AllSets>, "held_karp(): without a variant nothing prunes, so AllSets");
static_assert(std::is_same_v<engine_of<service_times::ServiceTimes>, AllSets>,
	"held_karp(): ServiceTimes declares that the cheapest state suffices and filters nothing, so AllSets");
static_assert(std::is_same_v<engine_of<time_windows::Strict>, ReachableLabels>,
	"held_karp(): under Strict, a costlier state that departs earlier can complete the tour, so ReachableLabels");
static_assert(std::is_same_v<engine_of<time_windows::Relaxed>, ReachableLabels>,
	"held_karp(): under Relaxed, a costlier state that departs earlier can pay less, so ReachableLabels");
static_assert(std::is_same_v<engine_of<Composed<service_times::ServiceTimes, time_windows::Strict>>, ReachableLabels>,
	"held_karp(): a Composed needs every non-dominated state when any of its components does, so ReachableLabels");
static_assert(std::is_same_v<engine_of<StrongPrecedence>, ReachableSets>, "held_karp(): a variant of one's own that filters DPMove gets ReachableSets");
static_assert(std::is_same_v<engine_of<Composed<service_times::ServiceTimes, StrongPrecedence>>, ReachableSets>,
	"held_karp(): ServiceTimes and a filter of one's own, so ReachableSets");

// ---------------------------------------------------------------------------
// Deadline test, per variant
// ---------------------------------------------------------------------------

static_assert(detail::dp_deadlines<time_windows::Strict>::value
	&& detail::dp_deadlines<Composed<service_times::ServiceTimes, time_windows::Strict>>::value
	&& detail::dp_deadlines<Composed<time_windows::Strict, time_windows::Relaxed>>::value
	&& detail::dp_deadlines<Composed<Composed<service_times::ServiceTimes, time_windows::Strict>, time_windows::Relaxed>>::value,
	"held_karp drops the states that miss a due time under one Strict, alone or composed with shipped variants");
static_assert(!detail::dp_deadlines<NoCallbacks>::value && !detail::dp_deadlines<service_times::ServiceTimes>::value
	&& !detail::dp_deadlines<time_windows::Relaxed>::value,
	"without a Strict, nothing is due");
static_assert(!detail::dp_deadlines<Composed<time_windows::Strict, time_windows::Strict>>::value,
	"the test reads one Strict: under two, held_karp keeps every state the filters admit");
static_assert(!detail::dp_deadlines<Composed<Shortcut, time_windows::Strict>>::value,
	"a variant of the user's could make a time earlier: held_karp keeps every state the filters admit");

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

// A complete tour from any engine passes every filter of the AppendMove
// pipeline at the cost reported. AllSets and ReachableSets reach the same status
// and cost. ReachableLabels reaches the optimum or infeasibility they claim, and
// a tour no costlier than one they report as feasible.
template <typename... Variant>
void check_engines_agree(const DistanceMatrix<int>& dist, const Variant&... variant) {
	Solver all(dist, variant...);
	Solver reach(dist, variant...);
	Solver labels(dist, variant...);
	run_checked(all, [&] { all.held_karp(AllSets{}); });
	run_checked(reach, [&] { reach.held_karp(ReachableSets{}); });
	run_checked(labels, [&] { labels.held_karp(ReachableLabels{}); });
	assert(all.status() == reach.status() && "AllSets and ReachableSets must reach the same status");
	assert(all.cost() == reach.cost() && "AllSets and ReachableSets must reach the same cost");
	if (reach.status() == SolutionStatus::feasible)
		assert(labels.status() == SolutionStatus::optimal && labels.cost() <= reach.cost()
			&& "ReachableLabels must reach a tour no costlier than one ReachableSets reports as feasible");
	else if (reach.status() != SolutionStatus::partial)
		assert(labels.status() == reach.status() && labels.cost() == reach.cost()
			&& "ReachableLabels must reach the optimum or infeasibility that ReachableSets claims");
	for (const auto* s : {&all, &reach, &labels}) {
		if (s->status() != SolutionStatus::optimal && s->status() != SolutionStatus::feasible) continue;
		const auto cost = pipeline_cost(dist, s->tour(), variant...);
		assert(cost && *cost == s->cost() && "a complete tour must pass the AppendMove pipeline at the cost reported");
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

// Strict with waiting, Relaxed and ServiceTimes with Strict with waiting,
// n = 9 and 10, 3 instances per size: held_karp() reaches branch_and_bound()'s
// status and cost, beyond the reach of brute force.
void test_against_branch_and_bound() {
	test::Rng rng{0xA0761D6478BD642Full};
	for (std::size_t n = 9; n <= 10; ++n) {
		for (int seed = 0; seed < 3; ++seed) {
			std::vector<std::size_t> witness(n);
			std::iota(witness.begin(), witness.end(), std::size_t{0});
			for (std::size_t i = n - 1; i > 1; --i)
				std::swap(witness[i], witness[1 + rng() % i]);
			test::Instance in{random_matrix(rng, n, 30), witness, rng};
			const auto service = test::make_service(in);
			const service_times::ServiceTimes times(service);
			const time_windows::Strict strict(test::make_windows(in, {}, true));
			const time_windows::Relaxed relaxed(test::make_windows(in, {}, true), 10);
			const time_windows::Strict strict_after_service(test::make_windows(in, service, true));
			auto check = [&](const auto& variant) {
				Solver hk(in.dist, variant);
				Solver bb(in.dist, variant);
				run_checked(hk, [&] { hk.held_karp(); });
				run_checked(bb, [&] { bb.branch_and_bound(); });
				assert(hk.status() == bb.status() && (hk.status() != SolutionStatus::optimal || hk.cost() == bb.cost())
					&& "held_karp() must reach branch_and_bound()'s status and cost");
			};
			check(strict);
			check(relaxed);
			check(Composed(times, strict_after_service));
		}
	}
}

// No complete tour: every engine reports infeasible and leaves no tour, even
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
	check(ReachableLabels{});
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
	solver.set_matrix(large);
	run(large, ReachableLabels{});
}

// Every arc takes 1 and city 1 closes at 1: leaving city 0 for another city, a
// state reaches city 1 at 2 at the earliest. Only {0} and the sets that hold
// city 1 remain, 1 + 2^(n-2) of the 2^(n-1). Then every arc takes 10, city 1
// opens at 100 and city 0 closes at 105: leaving city 1 at 100, a tour returns
// at 110 at the earliest. Only the sets without city 1 remain, 2^(n-2), and no
// tour: a dropped state is no loss, so even one state per (set, city) proves
// there is none.
void test_deadline_pruning() {
	constexpr std::size_t n = 6;
	constexpr double open = 1e9;
	auto uniform = [](int weight) {
		std::vector<int> w(n * n, weight);
		for (std::size_t i = 0; i < n; ++i) w[i * n + i] = 0;
		return DistanceMatrix<int>(n, w);
	};

	const auto ones = uniform(1);
	std::vector<time_windows::TimeWindow> closing(n, {0, open});
	closing[1] = {0, 1};
	const time_windows::Strict first(closing);
	Solver sets(ones, first);
	Solver labels(ones, first);
	run_checked(sets, [&] { sets.held_karp(ReachableSets{}); });
	run_checked(labels, [&] { labels.held_karp(ReachableLabels{}); });
	assert(sets.status() == SolutionStatus::optimal && labels.status() == SolutionStatus::optimal
		&& sets.cost() == static_cast<int>(n) && labels.cost() == static_cast<int>(n)
		&& "city 1 first, every tour costs n");
	assert(SolverTestAccess(sets).hk_slots() == 1 + (std::size_t{1} << (n - 2))
		&& "ReachableSets keeps {0} and the sets that hold city 1 alone");
	assert(SolverTestAccess(labels).hk_label_slots() == 1 + (std::size_t{1} << (n - 2))
		&& "ReachableLabels keeps {0} and the sets that hold city 1 alone");

	const auto tens = uniform(10);
	std::vector<time_windows::TimeWindow> returning(n, {0, open});
	returning[0] = {0, 105};
	returning[1] = {100, 200};
	const time_windows::Strict back(returning);
	Solver all(tens, back);
	Solver sets_back(tens, back);
	Solver labels_back(tens, back);
	run_checked(all, [&] { all.held_karp(AllSets{}); });
	run_checked(sets_back, [&] { sets_back.held_karp(ReachableSets{}); });
	run_checked(labels_back, [&] { labels_back.held_karp(ReachableLabels{}); });
	for (const auto* s : {&all, &sets_back, &labels_back})
		assert(s->status() == SolutionStatus::infeasible && s->tour().empty()
			&& "no tour returns in time, and a dropped state is no loss");
	assert(SolverTestAccess(sets_back).hk_slots() == std::size_t{1} << (n - 2)
		&& "ReachableSets keeps the sets without city 1 alone");
	assert(SolverTestAccess(labels_back).hk_label_slots() == std::size_t{1} << (n - 2)
		&& "ReachableLabels keeps the sets without city 1 alone");
}

// Only the tour 0, 1, 2, 3 reaches city 3 in time, at (0.3 + 0.2) + 0.1 = 0.6;
// the test takes the shortest path from city 1, 0.2 + 0.1, off 0.6, which
// rounds below the departure from city 1, 0.3.
void test_deadline_rounding() {
	assert((0.3 + 0.2) + 0.1 <= 0.6 && 0.3 > 0.6 - (0.2 + 0.1) && "the case rests on this rounding");
	DistanceMatrix<double> rounding(4, std::vector<double>(16, 10.0));
	for (std::size_t i = 0; i < 4; ++i) rounding(i, i) = 0.0;
	rounding(0, 1) = 0.3;
	rounding(1, 2) = 0.2;
	rounding(2, 3) = 0.1;
	rounding(3, 0) = 1.0;
	const std::vector<time_windows::TimeWindow> on_time = {{0, 1e9}, {0, 1e9}, {0, 1e9}, {0, 0.6}};
	const time_windows::Strict margin(on_time);
	auto check = [&](auto engine) {
		Solver s(rounding, margin);
		run_checked(s, [&] { s.held_karp(engine); });
		assert(s.status() == SolutionStatus::optimal && std::abs(s.cost() - 1.6) < 1e-9
			&& "the margin keeps a tour that arrives just on time");
	};
	check(AllSets{});
	check(ReachableSets{});
	check(ReachableLabels{});
}

// Leaving city 2 at 10, the shortest path reaches city 3 at 30, past 15; the
// shortcut at city 1 brings the tour 0, 2, 1, 3, the only one, there at 10.
// Applied, the deadline test would drop the state at city 2 and lose that tour:
// under a variant of the user's, held_karp does not apply it.
void test_deadline_test_needs_delays_only() {
	std::vector<int> w(16, 50);
	for (std::size_t i = 0; i < 4; ++i) w[i * 4 + i] = 0;
	DistanceMatrix<int> dist(4, w);
	dist(0, 2) = 10;
	dist(2, 1) = 10;
	dist(1, 3) = 10;
	dist(3, 0) = 10;
	const std::vector<time_windows::TimeWindow> windows = {{0, 1e9}, {0, 1e9}, {0, 1e9}, {0, 15}};
	const time_windows::Strict strict(windows);
	const Shortcut shortcut{{0, 20, 0, 0}};
	const Composed variant(shortcut, strict);

	std::optional<int> best;
	std::vector<std::size_t> tour{0, 1, 2, 3};
	do {
		const auto cost = pipeline_cost(dist, tour, variant);
		if (cost && (!best || *cost < *best)) best = cost;
	} while (std::next_permutation(tour.begin() + 1, tour.end()));
	assert(best == 40 && "the shortcut admits the tour 0, 2, 1, 3 alone");

	auto check = [&](auto engine) {
		Solver s(dist, variant);
		run_checked(s, [&] { s.held_karp(engine); });
		assert(s.status() == SolutionStatus::optimal && s.cost() == *best && "held_karp keeps the tour the shortcut admits");
	};
	check(AllSets{});
	check(ReachableSets{});
	check(ReachableLabels{});
}

// A limit that expires while the latest departures are built stops the engine
// there: partial, the start city alone. Ten cities, 50 us per read of the
// distance: building them takes 5 ms, past the limit.
void test_deadline_stops_in_precompute() {
	using namespace std::chrono;
	test::Rng rng{0x6A09E667F3BCC908ull};
	const auto dist = random_matrix(rng, 10, 30);
	const SlowDistance slow{&dist, microseconds{50}};
	const std::vector<time_windows::TimeWindow> windows(dist.size(), {0.0, 1e9});
	const time_windows::Strict tw(windows);
	auto check = [&](auto engine) {
		Solver s(slow, tw);
		run_checked(s, [&] { s.held_karp(engine, {.time_limit = milliseconds{1}}); });
		assert(s.status() == SolutionStatus::partial && s.tour().size() == 1 && s.tour()[0] == 0
			&& "stopped before any tour, held_karp keeps the start city alone");
	};
	check(AllSets{});
	check(ReachableSets{});
	check(ReachableLabels{});
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

	std::printf("against_branch_and_bound ... ");
	std::fflush(stdout);
	test_against_branch_and_bound();
	std::printf("OK\n");

	std::printf("infeasible ... ");
	std::fflush(stdout);
	test_infeasible();
	std::printf("OK\n");

	std::printf("reuse ... ");
	std::fflush(stdout);
	test_reuse();
	std::printf("OK\n");

	std::printf("deadline_pruning ... ");
	std::fflush(stdout);
	test_deadline_pruning();
	std::printf("OK\n");

	std::printf("deadline_rounding ... ");
	std::fflush(stdout);
	test_deadline_rounding();
	std::printf("OK\n");

	std::printf("deadline_test_needs_delays_only ... ");
	std::fflush(stdout);
	test_deadline_test_needs_delays_only();
	std::printf("OK\n");

	std::printf("deadline_stops_in_precompute ... ");
	std::fflush(stdout);
	test_deadline_stops_in_precompute();
	std::printf("OK\n");
}
