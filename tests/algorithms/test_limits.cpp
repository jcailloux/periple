// Time limit contract, for every registered algorithm: RunOptions::time_limit
// reaches the algorithm, an expired limit does no work (preparation included),
// a short one stops the algorithm soon after, and a stopped algorithm leaves a
// consistent, reusable solver that proves nothing.

#include <periple/algorithms/registry.hpp>
#include <periple/core/deadline.hpp>
#include <periple/core/moves/append_move.hpp>
#include <periple/core/moves/dp_move.hpp>
#include <periple/distance/matrix.hpp>
#include <solver_test_access.hpp>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace periple;
using namespace std::chrono;

// Counts move_prepare calls, and spins in each while spin is set: an algorithm
// that ignores its limit then runs for seconds.
struct SlowPrepare {
	mutable std::size_t calls = 0;
	microseconds spin{0};

	template <typename CityT>
	void move_prepare(const AppendMove<CityT>&) const { burn(); }

	template <typename CityT>
	void move_prepare(const DPMove<CityT>&) const { burn(); }

	void burn() const {
		++calls;
		if (spin.count() == 0) return;
		const auto end = steady_clock::now() + spin;
		while (steady_clock::now() < end) {}
	}
};

DistanceMatrix<int> random_asym(std::size_t n, std::uint64_t x) {
	std::vector<int> w(n * n, 0);
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = 0; j < n; ++j) {
			x ^= x << 13; x ^= x >> 7; x ^= x << 17;
			if (i != j) w[i * n + j] = static_cast<int>(x % 100) + 1;
		}
	return DistanceMatrix<int>(n, w);
}

template <typename Algo, typename S>
void setup(const Algo& algo, S& s) {
	if constexpr (requires { algo.setup(s); }) algo.setup(s);
}

bool proves_nothing(SolutionStatus status) {
	return status == SolutionStatus::partial || status == SolutionStatus::feasible;
}

// ---------------------------------------------------------------------------
// Deadline
// ---------------------------------------------------------------------------

void test_deadline() {
	detail::NoDeadline none;
	detail::Poller unlimited(none);
	assert(!detail::expired(unlimited) && "without a limit, nothing expires");

	detail::Deadline past(nanoseconds{1});
	detail::Poller poll(past);
	assert(detail::expired(poll) && "the first poll reads the clock");
	assert(detail::expired(poll) && "an expired deadline stays expired");

	// Tight polling: never before the limit, and within the contract's 50 us
	// after it on the best of five runs, since one run can be preempted.
	const auto limit = milliseconds{5};
	auto best_lag = nanoseconds::max();
	for (int run = 0; run < 5; ++run) {
		const auto start = steady_clock::now();
		detail::Deadline soon(limit);
		detail::Poller soon_poll(soon);
		while (!detail::expired(soon_poll)) {}
		const auto elapsed = duration_cast<nanoseconds>(steady_clock::now() - start);
		assert(elapsed >= limit && "a deadline never expires early");
		best_lag = std::min(best_lag, elapsed - limit);
	}
	assert(best_lag < microseconds{50} && "a deadline is noticed within 50 us of tight polling");
}

// ---------------------------------------------------------------------------
// Every registered algorithm
// ---------------------------------------------------------------------------

// An expired limit does no work: at most the start city is placed.
template <typename Algo>
void test_expired_limit(const Algo& algo) {
	auto dist = random_asym(6, 0x9E3779B97F4A7C15ull);
	SlowPrepare variant;
	Solver s(dist, variant);
	setup(algo, s);
	variant.calls = 0;
	run_checked(s, [&] { algo.run(s, {.time_limit = nanoseconds{1}}); });
	assert(variant.calls <= 1 && "an expired limit does no work, placing the start city at most");
	assert(proves_nothing(s.status()) && "a stopped algorithm proves nothing: partial or feasible");
}

// Each move_prepare costs spin, far above the deadline's read interval, so
// the stride stays at one poll per unit. Within the limit, at most limit / spin
// calls complete and one is under way; after it, the contract allows one unit
// of work (2n calls) and the wind-down (n + 1 calls). Counting calls keeps the
// check independent of machine speed.
// A preparation calls no callback: its units stay cheap and grow the stride
// of a poller shared with the costly units after it. Sizes where it lasts long
// enough to show: a 12-city table, 64 cities of neighbor lists.
template <typename Algo>
void test_short_limit(const Algo& algo) {
	constexpr auto spin = microseconds{500};
	constexpr auto limit = milliseconds{20};
	auto dist = random_asym((Algo::max_tier == 1) ? 12 : 64, 0xD1B54A32D192ED03ull);
	const std::size_t n = dist.size();
	const auto bound = static_cast<std::size_t>(limit / spin) + 1 + 2 * n + (n + 1);
	SlowPrepare variant;

	Solver full(dist, variant);
	setup(algo, full);
	variant.calls = 0;
	algo.run(full, {});
	const std::size_t full_calls = variant.calls;
	assert(full_calls > bound && "setup: the full run must make more calls than a limited one may");

	Solver s(dist, variant);
	setup(algo, s);
	variant.calls = 0;
	variant.spin = spin;
	const auto start = steady_clock::now();
	run_checked(s, [&] { algo.run(s, {.time_limit = limit}); });
	const auto elapsed = steady_clock::now() - start;
	variant.spin = microseconds{0};

	assert(variant.calls <= bound && "the algorithm must stop within the tolerance: one unit and the wind-down after the limit");
	assert(elapsed < limit + seconds{1} && "the algorithm must stop soon after its limit");
	assert(proves_nothing(s.status()) && "a stopped algorithm proves nothing: partial or feasible");
}

// An expired limit returns at once, preparation included. Allocating a table
// or building neighbor lists before the first poll calls no callback, so only
// time shows it: instances where that preparation takes milliseconds (a 20-city
// table, 2000-city neighbor lists), best of five runs against preemption.
template <typename Algo>
void test_expired_limit_time(const Algo& algo) {
	const std::size_t n = (Algo::max_tier == 1) ? 20 : 2000;
	auto dist = random_asym(n, 0x2545F4914F6CDD1Dull);
	auto best = nanoseconds::max();
	for (int run = 0; run < 5; ++run) {
		Solver s(dist);  // fresh: no cache from a previous run
		setup(algo, s);
		const auto start = steady_clock::now();
		algo.run(s, {.time_limit = nanoseconds{1}});
		best = std::min(best, duration_cast<nanoseconds>(steady_clock::now() - start));
	}
	assert(best < milliseconds{1} && "an expired limit returns at once, preparation included");
}

// A stopped algorithm leaves the solver reusable: run again without a limit,
// on the same instance or after set_matrix on another one of the same size, it
// gives a valid tour, and an exact algorithm the optimal cost of a fresh run.
// Stale state left by the stop (a table, a half-built cache) would show here.
// Only an optimal cost is compared, which asks no determinism of the algorithm.
template <typename Algo>
void test_reuse_after_stop(const Algo& algo) {
	constexpr std::size_t n = 10;
	const auto first  = random_asym(n, 0x9E3779B97F4A7C15ull);
	const auto second = random_asym(n, 0xBF58476D1CE4E5B9ull);

	for (const auto* target : {&first, &second}) {
		SlowPrepare variant;
		Solver s(first, variant);
		setup(algo, s);
		variant.spin = microseconds{1000};
		algo.run(s, {.time_limit = milliseconds{3}});  // stops after a few calls
		variant.spin = microseconds{0};

		if (target != &first) s.set_matrix(*target);
		setup(algo, s);
		run_checked(s, [&] { algo.run(s, {}); });

		assert((s.status() == SolutionStatus::feasible || s.status() == SolutionStatus::optimal)
			&& "after a stop, an unlimited run completes the tour");
		const auto tour = s.tour();
		assert(tour.size() == n && "after a stop, an unlimited run visits every city");
		std::vector<bool> seen(n, false);
		int cost = 0;
		for (std::size_t i = 0; i < n; ++i) {
			const auto c = static_cast<std::size_t>(tour[i]);
			assert(!seen[c] && "after a stop, an unlimited run visits each city once");
			seen[c] = true;
			cost += (*target)(tour[i], tour[(i + 1) % n]);
		}
		assert(s.cost() == cost && "after a stop, the cost matches the tour on the current instance");

		if constexpr (Algo::is_exact) {
			SlowPrepare fresh_variant;
			Solver fresh(*target, fresh_variant);
			setup(algo, fresh);
			algo.run(fresh, {});
			assert(s.cost() == fresh.cost() && "after a stop, an exact algorithm still finds the optimum");
		}
	}
}

int main() {
	std::printf("deadline ... ");
	std::fflush(stdout);
	test_deadline();
	std::printf("OK\n");

	for_each_algorithm(nullptr, [](const auto& algo) {
		std::printf("%s ... ", algo.name);
		std::fflush(stdout);
		test_expired_limit(algo);
		test_expired_limit_time(algo);
		test_short_limit(algo);
		test_reuse_after_stop(algo);
		std::printf("OK\n");
	});
}
