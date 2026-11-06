#include <periple/periple.hpp>
#include <periple/io/tsplib_parser.hpp>
#include <periple/variants/time_windows.hpp>
#include <solver_test_access.hpp>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

using namespace periple;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct Rng {
	std::uint64_t x;
	std::uint64_t operator()() { x ^= x << 13; x ^= x >> 7; x ^= x << 17; return x; }
	int below(int hi) { return static_cast<int>((*this)() % static_cast<std::uint64_t>(hi)); }
};

DistanceMatrix<int> random_asym(Rng& rng, std::size_t n, int hi) {
	std::vector<int> w(n * n, 0);
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = 0; j < n; ++j)
			if (i != j) w[i * n + j] = rng.below(hi) + 1;
	return DistanceMatrix<int>(n, w);
}

SymmetricDistanceMatrix<int> random_sym(Rng& rng, std::size_t n, int hi) {
	SymmetricDistanceMatrix<int> m(n);
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = i + 1; j < n; ++j)
			m.set(i, j, rng.below(hi) + 1);
	return m;
}

// Cost of a complete tour through the AppendMove pipeline, nullopt if a
// move_filter rejects one of its moves or its closing edge.
template <typename Dist, typename Variant>
auto pipeline_cost(const Dist& dist, const Variant& variant, std::span<const std::size_t> tour)
	-> std::optional<typename dist_traits<Dist>::cost_type>
{
	Solver s(dist, variant);
	s.append(tour[0]);
	for (std::size_t k = 1; k < tour.size(); ++k) {
		if (!s.can_append(tour[k])) return std::nullopt;
		s.append(tour[k]);
	}
	if (s.status() != SolutionStatus::feasible) return std::nullopt;
	return s.cost();
}

// Cheapest tour extending prefix (at least the start city), by enumeration.
template <typename Dist, typename Variant>
auto pipeline_optimum(const Dist& dist, const Variant& variant, std::vector<std::size_t> prefix)
	-> std::optional<typename dist_traits<Dist>::cost_type>
{
	std::vector<std::size_t> tour = prefix;
	for (std::size_t c = 0; c < dist.size(); ++c)
		if (std::find(prefix.begin(), prefix.end(), c) == prefix.end()) tour.push_back(c);
	const auto rest = tour.begin() + static_cast<std::ptrdiff_t>(prefix.size());
	std::sort(rest, tour.end());
	std::optional<typename dist_traits<Dist>::cost_type> best;
	do {
		const auto cost = pipeline_cost(dist, variant, tour);
		if (cost && (!best || *cost < *best)) best = cost;
	} while (std::next_permutation(rest, tour.end()));
	return best;
}

// ---------------------------------------------------------------------------
// Default bounds against held_karp
// ---------------------------------------------------------------------------

// Both default bounds, with and without a tour to beat, n = 3..9.
void test_default_bounds_match_held_karp() {
	Rng rng{0x9E3779B97F4A7C15ull};
	for (std::size_t n = 3; n <= 9; ++n) {
		for (int seed = 0; seed < 10; ++seed) {
			auto sym = random_sym(rng, n, 100);
			Solver exact(sym);
			exact.held_karp();

			Solver cold(sym);
			cold.set_symmetric(true);
			run_checked(cold, [&] { cold.branch_and_bound(); });
			assert(cold.status() == SolutionStatus::optimal && cold.cost() == exact.cost()
				&& "SpanningTreeBound must keep the optimum");

			Solver warm(sym);
			warm.set_symmetric(true);
			warm.nearest_neighbor().two_opt();
			run_checked(warm, [&] { warm.branch_and_bound(); });
			assert(warm.status() == SolutionStatus::optimal && warm.cost() == exact.cost()
				&& "a held tour must only speed the search up");
			assert(warm.tour()[0] == 0 && "the result starts at start_city, even from a rotated held tour");

			auto asym = random_asym(rng, n, 100);
			Solver exact_asym(asym);
			exact_asym.held_karp();
			Solver bb(asym);
			run_checked(bb, [&] { bb.branch_and_bound(); });
			assert(bb.status() == SolutionStatus::optimal && bb.cost() == exact_asym.cost()
				&& "OutgoingEdgeBound must keep the optimum");
		}
	}
}

// Tier 1 instances, the benchmark's exact requirement, in milliseconds.
void test_tsplib_optima() {
	const std::string dir = PERIPLE_INSTANCES_DIR;
	const struct { const char* file; int optimum; } cases[] = {
		{"/tier1/burma14.tsp", 3323},
		{"/tier1/gr24.tsp", 1272},
	};
	for (const auto& c : cases) {
		auto mat = load_tsplib(dir + c.file);
		Solver solver(mat);
		solver.set_symmetric(true);
		run_checked(solver, [&] { solver.branch_and_bound(); });
		assert(solver.status() == SolutionStatus::optimal && solver.cost() == c.optimum
			&& "branch_and_bound must reach the known optimum");
	}
}

// Negative weights: the Jonker-Volgenant view of an asymmetric instance.
void test_jonker_volgenant_view() {
	Rng rng{0x2545F4914F6CDD1Dull};
	for (std::size_t n = 3; n <= 6; ++n) {
		auto asym = random_asym(rng, n, 50);
		Solver exact(asym);
		exact.held_karp();

		auto jv = jonker_volgenant(asym);
		Solver solver(jv);
		solver.set_symmetric(true);
		run_checked(solver, [&] { solver.branch_and_bound(); });
		assert(solver.status() == SolutionStatus::optimal && "the view must be solved");
		assert(jv.atsp_cost(solver.cost()) == exact.cost() && "SpanningTreeBound must hold with negative weights");
	}
}

// Floating-point costs: the bound's rounding margin keeps the optimum.
void test_double_costs() {
	Rng rng{0xD1B54A32D192ED03ull};
	// Costs below 1: a bound rounded up as for integers would cut the optimum.
	for (const double scale : {7.0, 7000.0}) {
		for (std::size_t n = 4; n <= 8; ++n) {
			SymmetricDistanceMatrix<double> m(n);
			for (std::size_t i = 0; i < n; ++i)
				for (std::size_t j = i + 1; j < n; ++j)
					m.set(i, j, static_cast<double>(rng.below(1000)) / scale);
			Solver exact(m);
			exact.held_karp();
			Solver solver(m);
			solver.set_symmetric(true);
			run_checked(solver, [&] { solver.branch_and_bound(); });
			assert(std::abs(solver.cost() - exact.cost()) < 1e-9 && "the rounding margin must not cut the optimum");
		}
	}
}

// ---------------------------------------------------------------------------
// Variants under SpanningTreeBound (test_variants runs OutgoingEdgeBound)
// ---------------------------------------------------------------------------

void test_symmetric_relaxed_windows() {
	Rng rng{0x6A09E667F3BCC909ull};
	for (std::size_t n = 3; n <= 7; ++n) {
		for (int seed = 0; seed < 10; ++seed) {
			auto sym = random_sym(rng, n, 30);
			std::vector<time_windows::TimeWindow> windows(n);
			windows[0] = {0.0, 1e9};
			for (std::size_t c = 1; c < n; ++c)
				windows[c] = {static_cast<double>(rng.below(40)), static_cast<double>(20 + rng.below(80))};
			time_windows::Relaxed variant(windows, 10);
			Solver solver(sym, variant);
			solver.set_symmetric(true);
			run_checked(solver, [&] { solver.branch_and_bound(); });
			const auto best = pipeline_optimum(sym, variant, {0});
			assert(solver.status() == SolutionStatus::optimal && solver.cost() == *best
				&& "branch_and_bound must minimize penalties, waiting included");
		}
	}
}

// A dimension that keeps plain per-position state, without staging: fine for
// branch_and_bound, which only appends and pops.
struct RouteLoad {
	double load = 0;
	std::size_t pos_ = 0;
	std::vector<double> loads;

	void resize(std::size_t n) { loads.resize(n); }
	void reset() { std::fill(loads.begin(), loads.end(), 0.0); }

	template <DistanceSource Dist, typename CityT>
	void init(const AppendMove<CityT>& m, const Dist&) {
		pos_ = m.pos;
		load = (m.pos == 0) ? 0.0 : loads[m.pos - 1];
	}

	template <typename CityT>
	void commit(const AppendMove<CityT>&) { loads[pos_] = load; }

	struct AppendSnapshot { double load; std::size_t pos; };
	template <typename CityT>
	auto snapshot(const AppendMove<CityT>&) const -> AppendSnapshot { return {load, pos_}; }
	void restore(const AppendSnapshot& s) { load = s.load; pos_ = s.pos; }
};

// Loads pick up and drop off: the load must stay within [0, capacity].
struct PickupDelivery {
	using dimension = RouteLoad;
	std::vector<double> delta;
	double capacity;

	template <typename CityT, typename Ctx>
	void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const {
		if (m.pos < delta.size()) ctx.template dim<RouteLoad>().load += delta[static_cast<std::size_t>(m.city)];
	}

	template <typename CityT, typename Ctx>
	bool move_filter(const AppendMove<CityT>&, const Ctx& ctx) const {
		const double load = ctx.template dim<RouteLoad>().load;
		return load >= 0 && load <= capacity;
	}
};

void test_unstaged_dimension() {
	Rng rng{0xBB67AE8584CAA73Bull};
	for (std::size_t n = 3; n <= 7; ++n) {
		for (int seed = 0; seed < 10; ++seed) {
			auto dist = random_asym(rng, n, 30);
			PickupDelivery variant{std::vector<double>(n, 0.0), 2.0};
			for (std::size_t c = 1; c < n; ++c) variant.delta[c] = rng.below(3) - 1;
			Solver solver(dist, variant);
			run_checked(solver, [&] { solver.branch_and_bound(); });
			const auto best = pipeline_optimum(dist, variant, {0});
			assert((best ? solver.status() == SolutionStatus::optimal && solver.cost() == *best
			             : solver.status() == SolutionStatus::infeasible)
				&& "the popped position's dimension state must be overwritten, not undone");
		}
	}
}

// ---------------------------------------------------------------------------
// Held tour, prefix, node limit
// ---------------------------------------------------------------------------

void test_held_tour_bounds_the_search() {
	Rng rng{0x3C6EF372FE94F82Bull};
	auto dist = random_asym(rng, 7, 100);
	Solver solver(dist);
	solver.nearest_neighbor();
	const auto held = solver.cost();
	run_checked(solver, [&] { solver.branch_and_bound({.max_nodes = 1}); });
	assert(solver.status() == SolutionStatus::feasible && solver.cost() <= held
		&& "stopped early, the search returns the held tour or a better one");

	Solver fresh(dist);
	run_checked(fresh, [&] { fresh.branch_and_bound({.max_nodes = 1}); });
	assert(fresh.status() == SolutionStatus::partial && fresh.tour().size() == 1 && fresh.tour()[0] == 0
		&& "stopped before any complete tour, the solver keeps the root prefix");
}

// Without a variant, a held tour starting elsewhere is rotated.
void test_held_tour_rotated() {
	auto dist = SymmetricDistanceMatrix<int>({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	Solver solver(dist);
	std::vector<std::size_t> optimal = {2, 3, 1, 0};  // 30 + 25 + 10 + 15 = 80
	solver.set_tour(optimal);
	run_checked(solver, [&] { solver.branch_and_bound({.max_nodes = 1}); });
	assert(solver.cost() == 80 && solver.tour()[0] == 0
		&& "the held tour, rotated to start_city, is kept");
}

void test_resume_prefix() {
	Rng rng{0xA54FF53A5F1D36F1ull};
	for (std::size_t n = 4; n <= 7; ++n) {
		auto dist = random_asym(rng, n, 100);
		Solver solver(dist);
		std::vector<std::size_t> prefix = {0, n - 1};
		solver.set_tour(prefix);
		run_checked(solver, [&] { solver.branch_and_bound({.resume_at = 2}); });
		assert(solver.status() == SolutionStatus::feasible
			&& "optimal among the completions of a longer prefix only, hence feasible");
		assert(solver.tour()[0] == 0 && solver.tour()[1] == n - 1 && "the prefix is kept");
		assert(solver.cost() == *pipeline_optimum(dist, NoCallbacks{}, prefix)
			&& "the best completion of the prefix");

		solver.set_tour(std::vector<std::size_t>{2});
		run_checked(solver, [&] { solver.branch_and_bound({.resume_at = 1}); });
		assert(solver.status() == SolutionStatus::optimal && solver.tour()[0] == 2
			&& "a one-city prefix fixes the start only");
	}
}

// A limit reached before any complete tour leaves the prefix it was given.
void test_limit_keeps_prefix() {
	Rng rng{0x1F83D9ABFB41BD6Bull};
	auto dist = random_asym(rng, 8, 100);
	Solver solver(dist);
	std::vector<std::size_t> prefix = {0, 5, 2};
	solver.set_tour(prefix);
	run_checked(solver, [&] {
		solver.branch_and_bound({.resume_at = 3, .time_limit = std::chrono::nanoseconds{1}});
	});
	assert(solver.status() == SolutionStatus::partial && "an expired limit proves nothing");
	assert(std::equal(prefix.begin(), prefix.end(), solver.tour().begin(), solver.tour().end())
		&& "the given prefix is kept");
}

// On 500 cities a bound that ran to its end would outlast the allowance, so
// coming back in time proves it read stop. Up to 20 runs, stopping at the first
// on time, since a preempted run lags by a whole time slice.
void test_limit_stops_the_bound() {
	using namespace std::chrono;
	Rng rng{0x9B05688C2B3E6C1Full};
	auto dist = random_sym(rng, 500, 1000);
	constexpr auto limit = milliseconds{1};
	constexpr auto allowed = limit + milliseconds{4};
	SpanningTreeBound long_ascent;
	long_ascent.iterations = 1000;  // a bound that ignores stop overruns by far
	for (const bool symmetric : {false, true}) {
		auto best = nanoseconds::max();
		for (int run = 0; run < 20 && best >= allowed; ++run) {
			Solver solver(dist);
			solver.set_symmetric(symmetric, unchecked);
			solver.nearest_neighbor();  // a finite cutoff: SpanningTreeBound ascends
			const auto start = steady_clock::now();
			if (symmetric) solver.branch_and_bound(long_ascent, {.time_limit = limit});
			else solver.branch_and_bound({.time_limit = limit});
			best = std::min(best, duration_cast<nanoseconds>(steady_clock::now() - start));
			assert(solver.status() == SolutionStatus::feasible && "stopped early, the search returns the held tour or a better one");
		}
		assert(best < allowed && "a limit stops a bound under way");
	}
}

// ---------------------------------------------------------------------------
// Strategy callbacks
// ---------------------------------------------------------------------------

// Counts its calls; the trivial bound is valid for non-negative weights.
struct CountingBound {
	int* begins;
	int* calls;

	template <typename Solver>
	void begin(const Solver&) { ++*begins; }

	template <typename Solver>
	double lower_bound(const Solver&, double) { ++*calls; return 0.0; }
};

// Stopped in begin, it stays unprepared: the search must not call it.
struct WaitsInBegin {
	int* calls;

	template <typename Solver, typename Stop>
	void begin(const Solver&, Stop& stop) { while (!stop()) {} }

	template <typename Solver>
	double lower_bound(const Solver&, double) { ++*calls; return 0.0; }
};

// Declares every node without a completion.
struct RejectAll {
	template <typename Solver>
	double lower_bound(const Solver&, double) const { return std::numeric_limits<double>::infinity(); }
};

void test_custom_bound() {
	Rng rng{0x510E527FADE682D1ull};
	auto dist = random_asym(rng, 6, 100);
	Solver exact(dist);
	exact.held_karp();

	int begins = 0, calls = 0;
	Solver solver(dist);
	run_checked(solver, [&] { solver.branch_and_bound(CountingBound{&begins, &calls}); });
	assert(solver.status() == SolutionStatus::optimal && solver.cost() == exact.cost()
		&& "a valid user bound keeps the optimum");
	assert(begins == 1 && calls > 0 && "begin runs once per search, before the first lower_bound");

	int late_begins = 0, late_calls = 0;
	Solver late(dist);
	run_checked(late, [&] {
		late.branch_and_bound(CountingBound{&late_begins, &late_calls}, {.time_limit = std::chrono::nanoseconds{1}});
	});
	assert(late_begins == 0 && late_calls == 0 && "an expired limit does not start the strategy");

	int unprepared_calls = 0;
	Solver unprepared(dist);
	run_checked(unprepared, [&] {
		unprepared.branch_and_bound(WaitsInBegin{&unprepared_calls}, {.time_limit = std::chrono::milliseconds{1}});
	});
	assert(unprepared_calls == 0 && unprepared.status() == SolutionStatus::partial
		&& "a limit reached in begin ends the search before the first lower_bound");

	Solver rejected(dist);
	run_checked(rejected, [&] { rejected.branch_and_bound(RejectAll{}); });
	assert(rejected.status() == SolutionStatus::infeasible && rejected.tour().empty()
		&& "the strategy decides: a bound that prunes everything leaves no tour");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	struct Test { const char* name; void (*fn)(); };
	Test tests[] = {
		{"default_bounds_match_held_karp", test_default_bounds_match_held_karp},
		{"tsplib_optima",                  test_tsplib_optima},
		{"jonker_volgenant_view",          test_jonker_volgenant_view},
		{"double_costs",                   test_double_costs},
		{"symmetric_relaxed_windows",      test_symmetric_relaxed_windows},
		{"unstaged_dimension",             test_unstaged_dimension},
		{"held_tour_bounds_the_search",    test_held_tour_bounds_the_search},
		{"held_tour_rotated",              test_held_tour_rotated},
		{"resume_prefix",                  test_resume_prefix},
		{"limit_keeps_prefix",             test_limit_keeps_prefix},
		{"limit_stops_the_bound",          test_limit_stops_the_bound},
		{"custom_bound",                   test_custom_bound},
	};
	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}
