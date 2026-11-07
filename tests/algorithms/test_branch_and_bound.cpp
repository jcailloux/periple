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
// Bounds against held_karp
// ---------------------------------------------------------------------------

// The default bounds and AssignmentBound, with and without a tour to beat,
// n = 3..9; the asymmetric default holds on symmetric instances too.
void test_bounds_match_held_karp() {
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

			Solver mirror(sym);
			mirror.nearest_neighbor().two_opt();
			run_checked(mirror, [&] { mirror.branch_and_bound(SpanningArborescenceBound{}); });
			assert(mirror.status() == SolutionStatus::optimal && mirror.cost() == exact.cost()
				&& "SpanningArborescenceBound must keep the optimum of a symmetric instance");

			auto asym = random_asym(rng, n, 100);
			Solver exact_asym(asym);
			exact_asym.held_karp();
			Solver bb(asym);
			run_checked(bb, [&] { bb.branch_and_bound(); });
			assert(bb.status() == SolutionStatus::optimal && bb.cost() == exact_asym.cost()
				&& "SpanningArborescenceBound must keep the optimum");

			Solver warm_asym(asym);
			warm_asym.nearest_neighbor().two_opt();
			run_checked(warm_asym, [&] { warm_asym.branch_and_bound(); });
			assert(warm_asym.status() == SolutionStatus::optimal && warm_asym.cost() == exact_asym.cost()
				&& "SpanningArborescenceBound must keep the optimum below a held tour");

			Solver assignment(asym);
			run_checked(assignment, [&] { assignment.branch_and_bound(AssignmentBound{}); });
			assert(assignment.status() == SolutionStatus::optimal && assignment.cost() == exact_asym.cost()
				&& "AssignmentBound must keep the optimum");

			Solver warm_assignment(asym);
			warm_assignment.nearest_neighbor().two_opt();
			run_checked(warm_assignment, [&] { warm_assignment.branch_and_bound(AssignmentBound{}); });
			assert(warm_assignment.status() == SolutionStatus::optimal && warm_assignment.cost() == exact_asym.cost()
				&& "AssignmentBound must keep the optimum below a held tour");
		}
	}
}

// ---------------------------------------------------------------------------
// Bounds at a node, against enumeration
// ---------------------------------------------------------------------------

// Cheapest arborescence over last, the cities off the prefix and first, rooted
// at last, by enumeration of each node's parent: first leaves nothing, and last
// never goes straight to first while a city remains between them.
int min_arborescence(const DistanceMatrix<int>& dist, const std::vector<std::size_t>& prefix) {
	std::vector<std::size_t> nodes{prefix.back()};
	for (std::size_t c = 0; c < dist.size(); ++c)
		if (std::find(prefix.begin(), prefix.end(), c) == prefix.end()) nodes.push_back(c);
	nodes.push_back(prefix.front());
	const std::size_t m = nodes.size(), end = m - 1;
	auto allowed = [&](std::size_t a, std::size_t b) { return a != b && a != end && !(a == 0 && b == end); };
	std::vector<std::size_t> parent(m, 0);
	int best = std::numeric_limits<int>::max();
	for (;;) {
		bool ok = true;
		for (std::size_t b = 1; b < m && ok; ++b) ok = allowed(parent[b], b);
		for (std::size_t b = 1; b < m && ok; ++b) {
			std::size_t u = b, steps = 0;
			while (u != 0 && steps < m) { u = parent[u]; ++steps; }
			ok = u == 0;
		}
		if (ok) {
			int cost = 0;
			for (std::size_t b = 1; b < m; ++b) cost += dist(nodes[parent[b]], nodes[b]);
			best = std::min(best, cost);
		}
		std::size_t b = 1;
		while (b < m && ++parent[b] == end) parent[b++] = 0;
		if (b == m) break;
	}
	return best;
}

// Without ascent, the arborescence is the minimum one. Small costs give ties and
// cycles, hence contractions; up to 7 nodes.
void test_arborescence_is_minimum() {
	Rng rng{0x428A2F98D728AE22ull};
	auto never = [] { return false; };
	for (const int hi : {5, 1000}) {
		for (std::size_t n = 3; n <= 7; ++n) {
			for (int seed = 0; seed < 40; ++seed) {
				auto dist = random_asym(rng, n, hi);
				std::vector<std::size_t> order(n);
				std::iota(order.begin(), order.end(), std::size_t{0});
				for (std::size_t i = n - 1; i > 0; --i)
					std::swap(order[i], order[static_cast<std::size_t>(rng.below(static_cast<int>(i) + 1))]);
				const std::size_t len = 2 + static_cast<std::size_t>(rng.below(static_cast<int>(n) - 2));
				const std::vector<std::size_t> prefix(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(len));

				Solver solver(dist);
				for (std::size_t i = 0; i + 1 < len; ++i) solver.append(prefix[i]);
				SpanningArborescenceBound bound;
				bound.iterations = 0;
				bound.begin(solver);
				solver.append(prefix.back());
				const double value = bound.lower_bound(solver, 1e18, never);
				assert(value == min_arborescence(dist, prefix)
					&& "SpanningArborescenceBound without ascent must return the minimum arborescence");
			}
		}
	}
}

// Cheapest assignment of last and the cities off the prefix to those cities and
// first, by enumeration: no city to itself, and last never goes straight to first
// while a city remains between them.
int min_assignment(const DistanceMatrix<int>& dist, const std::vector<std::size_t>& prefix) {
	std::vector<std::size_t> rest;
	for (std::size_t c = 0; c < dist.size(); ++c)
		if (std::find(prefix.begin(), prefix.end(), c) == prefix.end()) rest.push_back(c);
	std::vector<std::size_t> rows{prefix.back()}, cols = rest;
	rows.insert(rows.end(), rest.begin(), rest.end());
	cols.push_back(prefix.front());
	std::sort(cols.begin(), cols.end());
	int best = std::numeric_limits<int>::max();
	do {
		bool ok = cols[0] != prefix.front();
		for (std::size_t r = 1; r < rows.size() && ok; ++r) ok = cols[r] != rows[r];
		if (!ok) continue;
		int cost = 0;
		for (std::size_t r = 0; r < rows.size(); ++r) cost += dist(rows[r], cols[r]);
		best = std::min(best, cost);
	} while (std::next_permutation(cols.begin(), cols.end()));
	return best;
}

// Along a chain of depths, each node starting from its parent's: with a cutoff,
// the value never exceeds the minimum assignment, and reaches the cutoff or
// equals that minimum; without one, it is the minimum. Small costs give ties;
// up to 8 cities.
void test_assignment_is_minimum() {
	Rng rng{0xB5C0FBCFEC4D3B2Full};
	auto never = [] { return false; };
	for (const int hi : {5, 1000}) {
		for (std::size_t n = 3; n <= 8; ++n) {
			for (int seed = 0; seed < 20; ++seed) {
				auto dist = random_asym(rng, n, hi);
				std::vector<std::size_t> order(n);
				std::iota(order.begin(), order.end(), std::size_t{0});
				for (std::size_t i = n - 1; i > 0; --i)
					std::swap(order[i], order[static_cast<std::size_t>(rng.below(static_cast<int>(i) + 1))]);
				const std::size_t root = 1 + static_cast<std::size_t>(rng.below(static_cast<int>(n) - 2));

				Solver solver(dist);
				for (std::size_t d = 0; d < root; ++d) solver.append(order[d]);
				AssignmentBound bound;
				bound.begin(solver, never);
				for (std::size_t d = root; d + 1 < n; ++d) {
					solver.append(order[d]);
					const std::vector<std::size_t> prefix(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(d + 1));
					const int exact = min_assignment(dist, prefix);
					const double cutoff = exact * (0.5 + rng.below(101) / 100.0);
					const double cut = bound.lower_bound(solver, cutoff, never);
					assert(cut <= exact && (cut >= cutoff || cut == exact)
						&& "AssignmentBound with a cutoff must reach it or return the minimum assignment");
					assert(bound.lower_bound(solver, std::numeric_limits<double>::infinity(), never) == exact
						&& "AssignmentBound without a cutoff must return the minimum assignment");
				}
			}
		}
	}
}

// Along a chain of depths, each node starting from its parent's state, with a
// cutoff around the optimal completion, a bound never exceeds that completion.
template <typename Dist, typename Bound>
void check_below_completion(const Dist& dist, Bound bound, bool symmetric, Rng& rng) {
	auto never = [] { return false; };
	const std::size_t n = dist.size();
	std::vector<std::size_t> order(n);
	std::iota(order.begin(), order.end(), std::size_t{0});
	for (std::size_t i = n - 1; i > 1; --i)
		std::swap(order[i], order[1 + static_cast<std::size_t>(rng.below(static_cast<int>(i)))]);
	Solver solver(dist);
	if (symmetric) solver.set_symmetric(true);
	solver.append(order[0]);
	if constexpr (requires { bound.begin(solver, never); }) bound.begin(solver, never);
	else bound.begin(solver);
	for (std::size_t d = 1; d + 1 < n; ++d) {
		solver.append(order[d]);
		const std::vector<std::size_t> prefix(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(d + 1));
		const double completion = static_cast<double>(*pipeline_optimum(dist, NoCallbacks{}, prefix) - solver.cost());
		const double cutoff = completion * (0.5 + rng.below(101) / 100.0);
		assert(bound.lower_bound(solver, cutoff, never) <= completion
			&& "a lower bound must not exceed the cheapest completion");
	}
}

void test_bounds_below_completion() {
	Rng rng{0x7137449123EF65CDull};
	for (const int hi : {5, 1000}) {
		for (std::size_t n = 4; n <= 8; ++n) {
			for (int seed = 0; seed < 10; ++seed) {
				auto asym = random_asym(rng, n, hi);
				check_below_completion(asym, SpanningArborescenceBound{}, false, rng);
				check_below_completion(asym, AssignmentBound{}, false, rng);
				auto sym = random_sym(rng, n, hi);
				check_below_completion(sym, SpanningArborescenceBound{}, false, rng);
				check_below_completion(sym, AssignmentBound{}, false, rng);
				check_below_completion(sym, SpanningTreeBound{}, true, rng);
			}
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
	// Forbidden edges must leave the cost usable: an overflow stops the pruning.
	for (std::size_t n = 7; n <= 14; ++n) {
		auto asym = random_asym(rng, n, 1000);
		Solver exact(asym);
		exact.held_karp();

		auto jv = jonker_volgenant(asym);
		Solver solver(jv);
		solver.set_symmetric(true);
		solver.branch_and_bound({.max_nodes = 1000000});
		assert(solver.status() == SolutionStatus::optimal && "a forbidden edge must prune its branch");
		assert(jv.atsp_cost(solver.cost()) == exact.cost() && "the view must reach the optimum");
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
			Solver assignment(m);
			run_checked(assignment, [&] { assignment.branch_and_bound(AssignmentBound{}); });
			assert(std::abs(assignment.cost() - exact.cost()) < 1e-9
				&& "the rounding margin must not cut the optimum under AssignmentBound");
		}
	}
}

// ---------------------------------------------------------------------------
// Variants under the bounds (test_variants runs SpanningArborescenceBound)
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

// Strict windows leave some instances without a tour.
void test_asymmetric_windows() {
	Rng rng{0x5BE0CD19137E2179ull};
	for (std::size_t n = 3; n <= 7; ++n) {
		for (int seed = 0; seed < 10; ++seed) {
			auto asym = random_asym(rng, n, 30);
			std::vector<time_windows::TimeWindow> windows(n);
			windows[0] = {0.0, 1e9};
			for (std::size_t c = 1; c < n; ++c)
				windows[c] = {static_cast<double>(rng.below(40)), static_cast<double>(20 + rng.below(80))};

			time_windows::Relaxed relaxed(windows, 10);
			Solver soft(asym, relaxed);
			run_checked(soft, [&] { soft.branch_and_bound(); });
			const auto soft_best = pipeline_optimum(asym, relaxed, {0});
			assert(soft.status() == SolutionStatus::optimal && soft.cost() == *soft_best
				&& "SpanningArborescenceBound must keep the optimum under penalties");

			Solver soft_assignment(asym, relaxed);
			run_checked(soft_assignment, [&] { soft_assignment.branch_and_bound(AssignmentBound{}); });
			assert(soft_assignment.status() == SolutionStatus::optimal && soft_assignment.cost() == *soft_best
				&& "AssignmentBound must keep the optimum under penalties");

			time_windows::Strict strict(windows);
			Solver hard(asym, strict);
			run_checked(hard, [&] { hard.branch_and_bound(); });
			const auto hard_best = pipeline_optimum(asym, strict, {0});
			assert((hard_best ? hard.status() == SolutionStatus::optimal && hard.cost() == *hard_best
			                  : hard.status() == SolutionStatus::infeasible)
				&& "SpanningArborescenceBound must keep the optimum under deadlines, or prove there is none");

			Solver hard_assignment(asym, strict);
			run_checked(hard_assignment, [&] { hard_assignment.branch_and_bound(AssignmentBound{}); });
			assert((hard_best ? hard_assignment.status() == SolutionStatus::optimal && hard_assignment.cost() == *hard_best
			                  : hard_assignment.status() == SolutionStatus::infeasible)
				&& "AssignmentBound must keep the optimum under deadlines, or prove there is none");
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

// A bound that ran to its end would outlast the allowance, so coming back in
// time proves it read stop. Up to 20 runs, stopping at the first on time, since
// a preempted run lags by a whole time slice.
void test_limit_stops_the_bound() {
	using namespace std::chrono;
	Rng rng{0x9B05688C2B3E6C1Full};
	constexpr auto limit = milliseconds{1};
	constexpr auto allowed = limit + milliseconds{4};
	auto stops_in_time = [&](const auto& dist, bool symmetric, auto bound) {
		auto best = nanoseconds::max();
		for (int run = 0; run < 20 && best >= allowed; ++run) {
			Solver solver(dist);
			solver.set_symmetric(symmetric, unchecked);
			solver.nearest_neighbor();  // a finite cutoff: the spanning bounds ascend
			const auto start = steady_clock::now();
			solver.branch_and_bound(bound, {.time_limit = limit});
			best = std::min(best, duration_cast<nanoseconds>(steady_clock::now() - start));
			assert(solver.status() == SolutionStatus::feasible && "stopped early, the search returns the held tour or a better one");
		}
		assert(best < allowed && "a limit stops a bound under way");
	};
	// Bounds that ignored stop would overrun by far: the spanning bounds ascend
	// 1000 steps per node on 500 cities, and on costs i * j the assignment of 400
	// cities takes long at the root.
	const auto dist = random_sym(rng, 500, 1000);
	SpanningTreeBound long_tree;
	long_tree.iterations = 1000;
	stops_in_time(dist, true, long_tree);
	SpanningArborescenceBound long_arborescence;
	long_arborescence.iterations = 1000;
	stops_in_time(dist, false, long_arborescence);
	constexpr std::size_t n = 400;
	std::vector<int> products(n * n, 0);
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = 0; j < n; ++j)
			if (i != j) products[i * n + j] = static_cast<int>(i * j) + 1;
	stops_in_time(DistanceMatrix<int>(n, products), false, AssignmentBound{});
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
		{"bounds_match_held_karp",         test_bounds_match_held_karp},
		{"arborescence_is_minimum",        test_arborescence_is_minimum},
		{"assignment_is_minimum",          test_assignment_is_minimum},
		{"bounds_below_completion",        test_bounds_below_completion},
		{"tsplib_optima",                  test_tsplib_optima},
		{"jonker_volgenant_view",          test_jonker_volgenant_view},
		{"double_costs",                   test_double_costs},
		{"symmetric_relaxed_windows",      test_symmetric_relaxed_windows},
		{"asymmetric_windows",             test_asymmetric_windows},
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
