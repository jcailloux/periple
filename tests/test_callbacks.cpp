#include <periple/periple.hpp>
#include <periple/variants/tsptw.hpp>
#include <logging_callbacks.hpp>

#include <cassert>
#include <cstdio>
#include <cstddef>
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
// Regression: DefaultCallbacks produce the same result as before
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

	// Explicit DefaultCallbacks should give the same result.
	solver.clear();
	solver.nearest_neighbor({}, DefaultCallbacks{});
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
	Solver solver(mat);
	solver.nearest_neighbor({}, cb);

	// n=4 cities.  Fresh build: on_commit for start city, then 3 steps.
	// Step 1: move_filter x3 candidates, on_commit x1
	// Step 2: move_filter x2 candidates, on_commit x1
	// Step 3: move_filter x1 candidate, on_commit x1
	// Total: move_filter = 3+2+1 = 6, on_commit = 1(start) + 3 = 4
	std::size_t filter_count = 0, commit_count = 0;
	for (const auto& entry : cb.log) {
		if (entry == "move_filter") ++filter_count;
		else if (entry == "on_commit") ++commit_count;
	}
	assert(filter_count == 6);
	assert(commit_count == 4);

	// Verify ordering: on_commit first (start city), then blocks of
	// (move_filter..., on_commit).
	assert(cb.log[0] == "on_commit");  // start city
	assert(cb.log[1] == "move_filter");
}

void test_protocol_nn_n1() {
	// Single city: only on_commit for start, no filtering.
	SymmetricDistanceMatrix<int> mat(1);
	LoggingCallbacks cb;
	Solver solver(mat);
	solver.nearest_neighbor({}, cb);

	assert(solver.tour().size() == 1);
	std::size_t filter_count = 0, commit_count = 0;
	for (const auto& e : cb.log) {
		if (e == "move_filter") ++filter_count;
		else if (e == "on_commit") ++commit_count;
	}
	assert(filter_count == 0);
	assert(commit_count == 1);
}

// ---------------------------------------------------------------------------
// Unit: move_filter rejects a specific city
// ---------------------------------------------------------------------------

struct RejectCity1 {
	bool move_filter(std::span<const std::size_t>,
	                 const AppendMove<std::size_t>& m) const {
		return m.city != 1;
	}
};

void test_move_filter_reject() {
	auto mat = make_mat4();
	RejectCity1 cb;
	Solver solver(mat);
	solver.nearest_neighbor({.start_city = 0}, cb);

	// Without filter, NN from 0 goes to 1 (nearest, dist 10).
	// With filter rejecting city 1, city 1 cannot be 2nd.
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] != 1);
	// City 1 still appears (fallback puts it somewhere).
	bool has_1 = false;
	for (auto c : solver.tour())
		if (c == 1) has_1 = true;
	assert(has_1);
}

// ---------------------------------------------------------------------------
// Unit: custom tour_cost
// ---------------------------------------------------------------------------

struct DoubleCost {
	template <DistanceSource D>
	auto operator()(const D& dist,
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
	template <DistanceSource Dist, typename Callbacks>
	auto evaluate(const Dist& dist,
	              std::span<const typename dist_traits<Dist>::city_type> tour,
	              typename dist_traits<Dist>::city_type candidate,
	              const Callbacks&) const
		-> typename dist_traits<Dist>::cost_type
	{
		// Negate distance: minimizing the negative = maximizing distance.
		return -dist(tour.back(), candidate);
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

	// Complete with NN.
	solver.nearest_neighbor();
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
	Solver solver(mat);
	solver.nearest_neighbor({}, tw);

	// Without strict: NN from 0 -> 1(10) -> 3(25) -> 2(30).
	// With strict: 0->1 rejected (arrival 10 > 1).
	// From 0: pick 2 (dist 15). From 2: 2->1 arr=50>1 rejected, pick 3.
	// From 3: 3->1 rejected, fallback picks 1. Tour: {0,2,3,1}.
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 2);
	assert(solver.tour()[2] == 3);
	assert(solver.tour()[3] == 1);
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
	auto expected = relaxed(mat, solver.tour());
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
	auto expected = relaxed(mat, solver.tour());
	assert(solver.cost() == expected);

	Solver ref(mat);
	ref.held_karp();
	assert(ref.cost() == 80); // distance-optimal
	assert(solver.cost() > 80); // penalties add to cost
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
		// Unit - selector
		{"custom_selector",              test_custom_selector},
		// Unit - partial tours
		{"partial_tour_completion",      test_partial_tour_completion},
		{"set_tour_full",                test_set_tour_full},
		{"set_tour_empty",               test_set_tour_empty},
		// Functional - TSPTW
		{"tsptw_strict_nn",              test_tsptw_strict_nn},
		{"tsptw_relaxed_nn",             test_tsptw_relaxed_nn},
		{"tsptw_relaxed_hk",             test_tsptw_relaxed_hk},
	};

	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}
