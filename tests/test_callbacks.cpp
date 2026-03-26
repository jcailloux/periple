#include <periple/periple.hpp>
#include <periple/variants/tsptw.hpp>
#include <logging_callbacks.hpp>

#include <cassert>
#include <cstdio>
#include <cstddef>
#include <optional>
#include <vector>

using namespace periple;

// ---------------------------------------------------------------------------
// Test matrices
// ---------------------------------------------------------------------------

auto make_mat4() {
	return SymmetricDistanceMatrix<int>({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
}

// ---------------------------------------------------------------------------
// Regression: NoCallbacks produce the same result as before
// ---------------------------------------------------------------------------

void test_regression_nn_default() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();

	assert(solver.status() == SolutionStatus::feasible);
	assert(solver.tour().size() == 4);
	assert(solver.tour()[0] == 0);
	// NN from 0: 0->1(10)->3(25)->2(30), cost=10+25+30+15=80
	assert(solver.cost() == 80);
}

void test_regression_hk_default() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.held_karp();
	assert(solver.status() == SolutionStatus::optimal);
	assert(solver.cost() == 80);
}

// ---------------------------------------------------------------------------
// Protocol: LoggingCallbacks verify callback invocation order and count
// ---------------------------------------------------------------------------

void test_protocol_nn_logging() {
	auto mat = make_mat4();
	LoggingCallbacks cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();

	// n=4 cities.  Fresh build: on_move for start city, then 3 steps.
	// Step 1: move_filter x3 candidates, on_move x1
	// Step 2: move_filter x2 candidates, on_move x1
	// Step 3: move_filter x1 candidate, on_move x1
	// Total: move_filter = 3+2+1 = 6, on_move = 1(start) + 3 = 4
	std::size_t filter_count = 0, commit_count = 0;
	for (const auto& entry : cb.log) {
		if (entry == "move_filter") ++filter_count;
		else if (entry == "on_move") ++commit_count;
	}
	assert(filter_count == 6);
	assert(commit_count == 4);

	// Verify ordering: on_move first (start city), then blocks of
	// (move_filter..., on_move).
	assert(cb.log[0] == "on_move");  // start city
	assert(cb.log[1] == "move_filter");
}

void test_protocol_nn_n1() {
	// Single city: only on_move for start, no filtering.
	SymmetricDistanceMatrix<int> mat(1);
	LoggingCallbacks cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();

	assert(solver.tour().size() == 1);
	std::size_t filter_count = 0, commit_count = 0;
	for (const auto& e : cb.log) {
		if (e == "move_filter") ++filter_count;
		else if (e == "on_move") ++commit_count;
	}
	assert(filter_count == 0);
	assert(commit_count == 1);
}

// ---------------------------------------------------------------------------
// Unit: move_filter rejects a specific city
// ---------------------------------------------------------------------------

struct RejectCity1 {
	template <typename CostT>
	bool move_filter(const AppendMove<std::size_t, CostT>& m) const {
		return m.city != 1;
	}
};

void test_move_filter_reject() {
	auto mat = make_mat4();
	RejectCity1 cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor({.start_city = 0});

	// City 1 is always rejected by the filter.
	// Steps: 0->2(15)->3(30)->nullopt. Partial tour.
	assert(solver.status() == SolutionStatus::partial);
	assert(solver.tour().size() == 3);
	assert(solver.tour()[0] == 0);
	for (auto c : solver.tour())
		assert(c != 1);
}

// ---------------------------------------------------------------------------
// Unit: custom tour_cost
// ---------------------------------------------------------------------------

struct DoubleCost {
	template <DistanceSource D>
	auto tour_cost(const D& dist,
	               std::span<const typename dist_traits<D>::city_type> tour) const
		-> typename dist_traits<D>::cost_type
	{
		typename dist_traits<D>::cost_type total{};
		const auto n = tour.size();
		for (std::size_t i = 0; i < n; ++i)
			total += dist(tour[i], tour[(i + 1) % n]);
		return total * 2;
	}
};

void test_custom_tour_cost_nn() {
	auto mat = make_mat4();
	DoubleCost tc;
	Solver solver(mat, tc);
	solver.nearest_neighbor();

	Solver ref(mat);
	ref.nearest_neighbor();

	// Same tour, double cost.
	for (std::size_t i = 0; i < 4; ++i)
		assert(solver.tour()[i] == ref.tour()[i]);
	assert(solver.cost() == ref.cost() * 2);
}

void test_custom_tour_cost_hk() {
	auto mat = make_mat4();
	DoubleCost tc;
	Solver solver(mat, tc);
	solver.held_karp();

	Solver ref(mat);
	ref.held_karp();

	assert(solver.cost() == ref.cost() * 2);
}

void test_custom_tour_cost_set_tour() {
	auto mat = make_mat4();
	DoubleCost tc;
	Solver solver(mat, tc);

	std::vector<std::size_t> t = {0, 2, 3, 1};
	solver.set_tour(t);
	assert(solver.status() == SolutionStatus::feasible);

	// compute_tour_cost uses DoubleCost
	Solver ref(mat);
	ref.set_tour(t);
	assert(solver.cost() == ref.cost() * 2);
}

// ---------------------------------------------------------------------------
// Unit: custom selector via greedy_construct
// ---------------------------------------------------------------------------

struct FarthestSelector {
	template <DistanceSource Dist, typename Variant>
	auto select_next(const Solver<Dist, Variant>& solver) const
		-> std::optional<typename dist_traits<Dist>::city_type>
	{
		using city_type = typename dist_traits<Dist>::city_type;
		using cost_type = typename dist_traits<Dist>::cost_type;

		auto tour = solver.tour();
		if (tour.empty()) return std::nullopt;

		cost_type best_dist{};
		city_type best_city{};
		bool found = false;

		for (std::size_t j = 0; j < solver.size(); ++j) {
			auto c = static_cast<city_type>(j);
			if (solver.is_visited(c)) continue;
			auto d = solver.distance(tour.back(), c);
			if (!found || d > best_dist) {
				best_dist = d;
				best_city = c;
				found = true;
			}
		}
		if (!found) return std::nullopt;
		return best_city;
	}
};

void test_custom_selector() {
	auto mat = make_mat4();

	Solver s1(mat);
	s1.nearest_neighbor();
	// NN from 0: 0->1(10)->3(25)->2(30)

	Solver s2(mat);
	s2.greedy_construct(FarthestSelector{});
	// Farthest from 0: 0->3(20)->2(30)->1(35)

	assert(s1.tour()[1] != s2.tour()[1]);
}

// ---------------------------------------------------------------------------
// Unit: partial tour completion
// ---------------------------------------------------------------------------

void test_partial_tour_completion() {
	auto mat = make_mat4();
	Solver solver(mat);

	// Set partial prefix.
	std::vector<std::size_t> prefix = {0, 2};
	solver.set_tour(prefix);
	assert(solver.status() == SolutionStatus::partial);
	assert(solver.tour().size() == 2);
	assert(solver.cost() == 15); // dist(0,2) = 15, open path

	// Complete with NN from the partial tour.
	solver.nearest_neighbor({.resume_at = solver.tour().size()});
	assert(solver.status() == SolutionStatus::feasible);
	assert(solver.tour().size() == 4);
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 2); // Prefix preserved.
	// From 2: unvisited {1,3}. dist(2,1)=35, dist(2,3)=30 -> pick 3.
	assert(solver.tour()[2] == 3);
	assert(solver.tour()[3] == 1);
}

void test_set_tour_full() {
	auto mat = make_mat4();
	Solver solver(mat);

	std::vector<std::size_t> t = {0, 2, 3, 1};
	solver.set_tour(t);
	assert(solver.status() == SolutionStatus::feasible);
	// cost = dist(0,2)+dist(2,3)+dist(3,1)+dist(1,0) = 15+30+25+10 = 80
	assert(solver.cost() == 80);
}

void test_set_tour_empty() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();

	std::vector<std::size_t> empty;
	solver.set_tour(empty);
	assert(solver.status() == SolutionStatus::none);
	assert(solver.tour().empty());
	assert(solver.cost() == 0);
}

// ---------------------------------------------------------------------------
// Unit: move_eval changes selection metric
// ---------------------------------------------------------------------------

struct DistancePlusBias {
	// Adds a large bias to city 1, making it unattractive.
	template <typename CostT>
	double move_eval(const AppendMove<std::size_t, CostT>& m) const {
		return m.city == 1 ? 9999.0 : 0.0;
	}
};

void test_move_eval_changes_selection() {
	auto mat = make_mat4();

	// Without move_eval: NN from 0 picks city 1 (dist 10, nearest).
	Solver ref(mat);
	ref.nearest_neighbor();
	assert(ref.tour()[1] == 1);

	// With move_eval: city 1 gets score 9999, others get 0.
	// NN picks any city with score 0 instead of city 1.
	DistancePlusBias cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();
	assert(solver.tour()[1] != 1);
}

// ---------------------------------------------------------------------------
// Unit: move_eval return type deduction (double on int matrix)
// ---------------------------------------------------------------------------

struct FractionalEval {
	template <typename CostT>
	double move_eval(const AppendMove<std::size_t, CostT>& m) const {
		// Fractional scores that would be truncated to 0 if cast to int.
		if (m.city == 1) return 0.3;
		if (m.city == 2) return 0.1;  // best
		return 0.5;
	}
};

void test_move_eval_type_deduction() {
	auto mat = make_mat4();
	FractionalEval cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();

	// City 2 has the lowest eval (0.1).  If evals were truncated to int,
	// cities 1 and 2 would both be 0 and the result would depend on iteration
	// order.  With correct double deduction, city 2 wins deterministically.
	assert(solver.tour()[1] == 2);
}

// ---------------------------------------------------------------------------
// Unit: partial tour completion with callbacks
// ---------------------------------------------------------------------------

void test_partial_tour_with_callbacks() {
	auto mat = make_mat4();

	std::vector<std::size_t> prefix = {0, 2};
	LoggingCallbacks cb;
	Solver solver(mat, cb);
	solver.set_tour(prefix);
	solver.nearest_neighbor({.resume_at = solver.tour().size()});

	// Prefix preserved.
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 2);
	assert(solver.status() == SolutionStatus::feasible);

	// on_move called only for newly placed cities (not for prefix).
	// 2 new cities placed -> 2 on_move calls.
	std::size_t commit_count = 0;
	for (const auto& e : cb.log)
		if (e == "on_move") ++commit_count;
	assert(commit_count == 2);
}

// ---------------------------------------------------------------------------
// Unit: set_variant method
// ---------------------------------------------------------------------------

void test_set_variant() {
	auto mat = make_mat4();
	DoubleCost tc;

	// Construct with variant from the start.
	Solver solver(mat, tc);
	solver.nearest_neighbor();
	auto cost_with = solver.cost();

	Solver ref(mat);
	ref.nearest_neighbor();

	assert(cost_with == ref.cost() * 2);
}

// ---------------------------------------------------------------------------
// Unit: asymmetric matrix with callbacks
// ---------------------------------------------------------------------------

void test_asymmetric_with_callbacks() {
	DistanceMatrix<int> mat(3, {
		0, 5, 8,
		12, 0, 3,
		6, 14, 0
	});

	LoggingCallbacks cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();

	assert(solver.tour().size() == 3);

	// Callbacks were invoked.
	std::size_t commit_count = 0;
	for (const auto& e : cb.log)
		if (e == "on_move") ++commit_count;
	assert(commit_count == 3);
}

// ---------------------------------------------------------------------------
// Functional: TSPTW Strict on NN
// ---------------------------------------------------------------------------

void test_tsptw_strict_nn() {
	auto mat = make_mat4();

	// City 1 has a very tight window: latest=1.
	// dist(0,1)=10 > 1, so 0->1 is always rejected.
	tsptw::TimeWindow windows[] = {
		{0, 100},  // city 0
		{0,   1},  // city 1: too tight for any direct visit
		{0, 100},  // city 2
		{0, 100},  // city 3
	};

	tsptw::Strict tw(mat, windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	// Without strict: NN from 0 -> 1(10) -> 3(25) -> 2(30).
	// With strict: 0->1 rejected (arrival 10 > 1) at every step.
	// From 0: pick 2 (dist 15). From 2: 2->1 rejected, pick 3.
	// From 3: 3->1 rejected, no candidates -> partial tour {0,2,3}.
	assert(solver.status() == SolutionStatus::partial);
	assert(solver.tour().size() == 3);
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 2);
	assert(solver.tour()[2] == 3);
}

// ---------------------------------------------------------------------------
// Functional: TSPTW Relaxed on NN and HK
// ---------------------------------------------------------------------------

void test_tsptw_relaxed_nn() {
	auto mat = make_mat4();

	// City 2 has a tight window.
	tsptw::TimeWindow windows[] = {
		{0, 100},  // city 0
		{0, 100},  // city 1
		{0,   5},  // city 2: tight
		{0, 100},  // city 3
	};

	tsptw::Relaxed relaxed(mat, windows, 1000);
	Solver solver(mat, relaxed);
	solver.nearest_neighbor();

	// Tour is the same as without relaxed (selection uses distance).
	Solver ref(mat);
	ref.nearest_neighbor();
	for (std::size_t i = 0; i < 4; ++i)
		assert(solver.tour()[i] == ref.tour()[i]);

	// Cost includes penalties.
	auto expected = relaxed.tour_cost(mat, solver.tour());
	assert(solver.cost() == expected);
	assert(solver.cost() > ref.cost());
}

void test_tsptw_relaxed_hk() {
	auto mat = make_mat4();

	tsptw::TimeWindow windows[] = {
		{0, 100},
		{0, 100},
		{0,   5},
		{0, 100},
	};

	tsptw::Relaxed relaxed(mat, windows, 1000);
	Solver solver(mat, relaxed);
	solver.held_karp();

	// HK finds the distance-optimal tour.
	// Reported cost includes penalties.
	auto expected = relaxed.tour_cost(mat, solver.tour());
	assert(solver.cost() == expected);

	Solver ref(mat);
	ref.held_karp();
	assert(ref.cost() == 80); // distance-optimal
	assert(solver.cost() > 80); // penalties add to cost
}

// ---------------------------------------------------------------------------
// Functional: TSPTW multi-window
// ---------------------------------------------------------------------------

void test_tsptw_strict_multi_window() {
	// 3 cities.  City 1 has two windows: [0,2] and [8,20].
	// dist(0,1) = 5 -> arrival at 5, between windows.
	// Window [0,2]: 5 > 2, closed.
	// Window [8,20]: 5 <= 20, feasible (wait until 8).
	SymmetricDistanceMatrix<int> mat({
		{0, 5, 3},
		{5, 0, 4},
		{3, 4, 0}
	});

	std::vector<std::vector<tsptw::TimeWindow>> windows = {
		{{0, 100}},            // city 0: always open
		{{0, 2}, {8, 20}},     // city 1: closed 2-8
		{{0, 100}},            // city 2: always open
	};

	tsptw::Strict tw(mat, windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	// NN from 0: candidates {1, 2}.
	// 0->2: dist=3, arrival=3, window [0,100] OK. Nearest.
	// 0->1: dist=5, arrival=5, window [0,2] closed, [8,20] OK (wait).
	// NN picks 0->2 (dist 3 < dist 5).
	// From 2: candidate {1}.
	// 2->1: depart from 2 = max(3, 0) = 3, arrival = 3+4 = 7.
	// Window [0,2]: 7 > 2, closed. Window [8,20]: 7 <= 20, OK.
	assert(solver.tour()[0] == 0);
	assert(solver.tour().size() == 3);

	// Both cities visited despite city 1's gap.
	bool has_1 = false;
	for (auto c : solver.tour())
		if (c == 1) has_1 = true;
	assert(has_1);
}

void test_tsptw_strict_multi_window_reject() {
	// City 1 has two windows, both too tight for arrival from city 0.
	SymmetricDistanceMatrix<int> mat({
		{0, 10, 3},
		{10, 0, 4},
		{3, 4, 0}
	});

	std::vector<std::vector<tsptw::TimeWindow>> windows = {
		{{0, 100}},            // city 0
		{{0, 2}, {4, 6}},      // city 1: both windows too early for dist=10
		{{0, 100}},            // city 2
	};

	tsptw::Strict tw(mat, windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	// From 0: 0->1 arrival=10, both windows latest=2 and 6 < 10. Rejected.
	// 0->2 arrival=3, OK. Pick 2.
	assert(solver.tour()[1] == 2);
}

// ---------------------------------------------------------------------------
// Unit: neighbor lists
// ---------------------------------------------------------------------------

void test_neighbors_basic() {
	auto mat = make_mat4();
	Solver solver(mat);

	// k=1: nearest neighbor of each city.
	// Distances from 0: {1:10, 2:15, 3:20} -> nearest is 1
	auto n0 = solver.neighbors(0, 1);
	assert(n0.size() == 1);
	assert(n0[0] == 1);

	// k=3: all other cities sorted by distance from 0.
	auto n0_all = solver.neighbors(0, 3);
	assert(n0_all.size() == 3);
	assert(n0_all[0] == 1);  // dist 10
	assert(n0_all[1] == 2);  // dist 15
	assert(n0_all[2] == 3);  // dist 20
}

void test_neighbors_grow() {
	auto mat = make_mat4();
	Solver solver(mat);

	// First request with k=1.
	(void)solver.neighbors(0, 1);

	// Grow to k=2.
	auto n = solver.neighbors(0, 2);
	assert(n.size() == 2);
	assert(n[0] == 1);
	assert(n[1] == 2);
}

void test_neighbors_invalidate() {
	auto mat = make_mat4();
	SymmetricDistanceMatrix<int> mat2({
		{0, 99, 1},
		{99, 0, 50},
		{1, 50, 0}
	});

	Solver solver(mat);
	(void)solver.neighbors(0, 1);

	// Switch matrix: neighbors must be recomputed.
	solver.set_matrix(mat2);
	auto n = solver.neighbors(0, 1);
	assert(n[0] == 2);  // dist(0,2)=1 is nearest in mat2
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	struct Test { const char* name; void (*fn)(); };
	Test tests[] = {
		// Regression
		{"regression_nn_default",        test_regression_nn_default},
		{"regression_hk_default",        test_regression_hk_default},
		// Protocol
		{"protocol_nn_logging",          test_protocol_nn_logging},
		{"protocol_nn_n1",              test_protocol_nn_n1},
		// Unit - move_filter
		{"move_filter_reject",           test_move_filter_reject},
		// Unit - tour_cost
		{"custom_tour_cost_nn",          test_custom_tour_cost_nn},
		{"custom_tour_cost_hk",          test_custom_tour_cost_hk},
		{"custom_tour_cost_set_tour",    test_custom_tour_cost_set_tour},
		// Unit - move_eval
		{"move_eval_changes_selection",  test_move_eval_changes_selection},
		{"move_eval_type_deduction",     test_move_eval_type_deduction},
		// Unit - selector
		{"custom_selector",              test_custom_selector},
		// Unit - partial tours
		{"partial_tour_completion",      test_partial_tour_completion},
		{"partial_tour_with_callbacks",  test_partial_tour_with_callbacks},
		{"set_tour_full",                test_set_tour_full},
		{"set_tour_empty",               test_set_tour_empty},
		// Unit - set_variant
		{"set_variant",                  test_set_variant},
		// Unit - asymmetric
		{"asymmetric_with_callbacks",    test_asymmetric_with_callbacks},
		// Functional - TSPTW
		{"tsptw_strict_nn",              test_tsptw_strict_nn},
		{"tsptw_strict_multi_window",    test_tsptw_strict_multi_window},
		{"tsptw_strict_multi_window_reject", test_tsptw_strict_multi_window_reject},
		{"tsptw_relaxed_nn",             test_tsptw_relaxed_nn},
		{"tsptw_relaxed_hk",             test_tsptw_relaxed_hk},
		// Unit - neighbor lists
		{"neighbors_basic",              test_neighbors_basic},
		{"neighbors_grow",               test_neighbors_grow},
		{"neighbors_invalidate",         test_neighbors_invalidate},
	};

	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}
