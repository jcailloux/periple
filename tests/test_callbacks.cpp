#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <logging_callbacks.hpp>

#include <cassert>
#include <cstdio>
#include <cstddef>
#include <optional>
#include <string>
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

	// n=4 cities.  Fresh build: move_prepare for start city, then 3 steps.
	// Each step: evaluate candidates (move_prepare + move_filter per candidate),
	// then append the winner (move_prepare again).
	// Step 1 (from city 0): 3 candidates -> 3x(prepare+filter), 1x append prepare
	// Step 2 (from city 1): 2 candidates -> 2x(prepare+filter), 1x append prepare
	// Step 3 (from city 3): 1 candidate  -> 1x(prepare+filter), 1x append prepare
	// Plus: start city append = 1x prepare, closing edge = 1x(prepare+filter)
	// move_filter: 3+2+1 + 1(closing) = 7
	// move_prepare: 1(start) + 3+1 + 2+1 + 1+1 + 1(closing) = 11
	std::size_t filter_count = 0, prepare_count = 0;
	for (const auto& entry : cb.log) {
		if (entry == "move_filter") ++filter_count;
		else if (entry == "move_prepare") ++prepare_count;
	}
	assert(filter_count == 7 && "expected 7 move_filter calls (3+2+1 candidates + 1 closing)");
	assert(prepare_count == 11 && "expected 11 move_prepare calls");
}

void test_protocol_nn_n1() {
	// Single city: only move_prepare for start city append, no filtering.
	SymmetricDistanceMatrix<int> mat(1);
	LoggingCallbacks cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();

	assert(solver.tour().size() == 1);
	std::size_t filter_count = 0, prepare_count = 0;
	for (const auto& e : cb.log) {
		if (e == "move_filter") ++filter_count;
		else if (e == "move_prepare") ++prepare_count;
	}
	assert(filter_count == 0 && "no filtering for single city");
	assert(prepare_count == 1 && "one move_prepare for the start city");
}

// ---------------------------------------------------------------------------
// Protocol: reversals replay through the construction pipeline
// ---------------------------------------------------------------------------

// Number of move_prepare / move_filter pairs in the log, asserting it holds
// nothing else and in that order: the shape evaluate_replay emits.
std::size_t checked_replay_pairs(const std::vector<std::string>& log) {
	assert(log.size() % 2 == 0 && "expected one move_filter per move_prepare");
	for (std::size_t k = 0; k < log.size(); k += 2) {
		assert(log[k] == "move_prepare" && "expected move_prepare to open each pair");
		assert(log[k + 1] == "move_filter" && "expected move_filter to close each pair");
	}
	return log.size() / 2;
}

void test_protocol_reversal_replay() {
	auto mat = make_mat4();
	LoggingCallbacks cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();

	const std::size_t n = solver.size();
	for (std::size_t i = 0; i + 1 < n; ++i) {
		for (std::size_t j = i + 1; j < n; ++j) {
			cb.log.clear();
			const auto scored = solver.evaluate_reversal(i, j);
			solver.discard_staging();
			assert(scored && "LoggingCallbacks rejects nothing, so the replay cannot stop early");
			// Positions i+1..n-1, then the closing edge, filtered but not committed.
			assert(checked_replay_pairs(cb.log) == n - i
				&& "the replay covers the suffix from i+1 whatever j, so n - i moves");
		}
	}
}

void test_protocol_two_opt_replay() {
	auto mat = make_mat4();
	LoggingCallbacks cb;
	Solver solver(mat, cb);

	// Deliberately bad tour: 15+35+25+20 = 95, against 80 for the optimum.
	// Reversing positions 2..3 gives [0,2,3,1] and recovers the 15.
	std::vector<std::size_t> bad = {0, 2, 1, 3};
	solver.set_tour(bad);
	assert(solver.cost() == 95 && "set_tour must cost the sum of its edges");

	cb.log.clear();  // set_tour rebuilds through move_prepare alone, without filtering
	solver.two_opt();
	assert(solver.cost() == 80 && "two_opt must apply the improving reversal");

	// A candidate scored by a direct delta would leave this log empty, and
	// replaying the pipeline again on accept, instead of committing the staged
	// evaluation, would break the alternation.
	assert(checked_replay_pairs(cb.log) > 0
		&& "two_opt must score its candidates through the variant pipeline");
}

// ---------------------------------------------------------------------------
// Protocol: exact DP moves through the pipeline
// ---------------------------------------------------------------------------

void test_protocol_held_karp_logging() {
	auto mat = make_mat4();
	LoggingCallbacks cb;
	Solver solver(mat, cb);
	solver.held_karp();
	assert(solver.status() == SolutionStatus::optimal && "LoggingCallbacks rejects nothing, so held_karp must succeed");

	// Forward: 3 moves from {0}, 2 from each of the 3 one-city states, 1 from
	// each of the 6 two-city states; then 3 closing moves.
	const std::size_t dp_moves = 3 + 3 * 2 + 6 * 1 + 3;
	assert(cb.log.size() == 2 * dp_moves + 5
		&& "expected 18 DP prepare/filter pairs, then the tour rebuilt as 4 appends and the closing edge");
	for (std::size_t k = 0; k < 2 * dp_moves; k += 2) {
		assert(cb.log[k] == "dp_move_prepare" && "expected dp_move_prepare to open each pair");
		assert(cb.log[k + 1] == "dp_move_filter" && "expected dp_move_filter to close each pair");
	}
	for (std::size_t k = 2 * dp_moves; k < cb.log.size(); ++k)
		assert(cb.log[k] == "move_prepare" && "rebuild_and_cost replays the tour through move_prepare alone");
}

// During construction the context exposes the placed prefix.
struct SeesPrefix {
	template <typename CityT, typename Ctx>
	bool move_filter(const AppendMove<CityT>& m, const Ctx& ctx) const {
		assert(ctx.tour().size() == m.pos && "construction must expose the placed prefix");
		assert(ctx.position().size() == m.pos && "and its inverse index");
		return true;
	}

	template <typename CityT>
	bool move_filter(const DPMove<CityT>&) const { return true; }
};

void test_protocol_construction_sees_prefix() {
	auto mat = make_mat4();
	SeesPrefix variant;
	Solver solver(mat, variant);
	solver.nearest_neighbor();
	assert(solver.status() == SolutionStatus::feasible && "SeesPrefix rejects nothing, so the construction must complete");
}

// ---------------------------------------------------------------------------
// Unit: move_filter rejects a specific city
// ---------------------------------------------------------------------------

struct RejectCity1 {
	bool move_filter(const AppendMove<std::size_t>& m) const {
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
// Unit: move_prepare adjusts cost_delta (replaces old move_eval)
// ---------------------------------------------------------------------------

struct PenalizeCity1 {
	template <typename Ctx>
	void move_prepare(const AppendMove<std::size_t>& m, Ctx& ctx) const {
		if (m.city == 1) ctx.cost_delta += 9999.0;
	}
};

void test_move_prepare_changes_selection() {
	auto mat = make_mat4();

	// Without variant: NN from 0 picks city 1 (dist 10, nearest).
	Solver ref(mat);
	ref.nearest_neighbor();
	assert(ref.tour()[1] == 1);

	// With variant: city 1 gets cost_delta 9999, others get 0.
	// NN picks any city with lower score instead of city 1.
	PenalizeCity1 cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();
	assert(solver.tour()[1] != 1);
}

// ---------------------------------------------------------------------------
// Unit: cost_delta is double (fractional precision preserved for selection)
// ---------------------------------------------------------------------------

struct FractionalPenalty {
	template <typename Ctx>
	void move_prepare(const AppendMove<std::size_t>& m, Ctx& ctx) const {
		// Fractional penalties that would be truncated to 0 as int.
		if (m.city == 1) ctx.cost_delta += 0.3;
		else if (m.city == 2) ctx.cost_delta += 0.1;  // best
		else ctx.cost_delta += 0.5;
	}
};

void test_fractional_cost_delta() {
	auto mat = make_mat4();
	FractionalPenalty cb;
	Solver solver(mat, cb);
	solver.nearest_neighbor();

	// City 2 has the lowest penalty (0.1 + dist).
	// dist(0,1)=10+0.3=10.3, dist(0,2)=15+0.1=15.1, dist(0,3)=20+0.5=20.5
	// City 1 still wins on total score (10.3 < 15.1).
	// But from city 1: dist(1,2)=35+0.1=35.1, dist(1,3)=25+0.5=25.5 -> pick 3.
	// From city 3: dist(3,2)=30+0.1=30.1 -> pick 2.
	// Tour: 0->1->3->2
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 1);
	assert(solver.tour()[2] == 3);
	assert(solver.tour()[3] == 2);
}

// ---------------------------------------------------------------------------
// Unit: custom selector via greedy_construct
// ---------------------------------------------------------------------------

struct FarthestSelector {
	template <DistanceSource Dist, typename Variant>
	auto select_next(const Solver<Dist, Variant>& solver) const -> std::optional<typename dist_traits<Dist>::city_type> {
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
}

// ---------------------------------------------------------------------------
// Unit: set_variant method
// ---------------------------------------------------------------------------

void test_set_variant() {
	auto mat = make_mat4();
	PenalizeCity1 cb;

	Solver solver(mat, cb);
	solver.nearest_neighbor();

	// City 1 should not be first pick (penalty makes it unattractive).
	assert(solver.tour()[1] != 1);
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
	std::size_t prepare_count = 0;
	for (const auto& e : cb.log)
		if (e == "move_prepare") ++prepare_count;
	assert(prepare_count > 0);
}

// ---------------------------------------------------------------------------
// Functional: TSPTW Strict on NN
// ---------------------------------------------------------------------------

void test_tsptw_strict_nn() {
	auto mat = make_mat4();

	// City 1 has a very tight window: latest=1.
	// dist(0,1)=10 > 1, so 0->1 is always rejected.
	time_windows::TimeWindow windows[] = {
		{0, 100},  // city 0
		{0,   1},  // city 1: too tight for any direct visit
		{0, 100},  // city 2
		{0, 100},  // city 3
	};

	time_windows::Strict tw(windows);
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

	time_windows::TimeWindow windows[] = {
		{0, 100},  // city 0
		{0, 100},  // city 1
		{0,   5},  // city 2: tight
		{0, 100},  // city 3
	};

	time_windows::Relaxed relaxed(windows, 1000);
	Solver solver(mat, relaxed);
	solver.nearest_neighbor();

	// Tour is the same as without relaxed (selection uses distance + penalty,
	// but penalties are only added for late arrivals).
	Solver ref(mat);
	ref.nearest_neighbor();
	for (std::size_t i = 0; i < 4; ++i)
		assert(solver.tour()[i] == ref.tour()[i]);

	// Cost includes penalties (should be > pure distance cost).
	assert(solver.cost() >= ref.cost());
}

void test_tsptw_relaxed_hk() {
	auto mat = make_mat4();

	time_windows::TimeWindow windows[] = {
		{0, 100},
		{0, 100},
		{0,   5},
		{0, 100},
	};

	time_windows::Relaxed relaxed(windows, 1000);
	Solver solver(mat, relaxed);
	solver.held_karp();

	Solver ref(mat);
	ref.held_karp();
	assert(ref.cost() == 80); // distance-optimal
	// HK with penalties: cost includes time window violations.
	assert(solver.cost() >= 80);
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

	std::vector<std::vector<time_windows::TimeWindow>> windows = {
		{{0, 100}},            // city 0: always open
		{{0, 2}, {8, 20}},     // city 1: closed 2-8
		{{0, 100}},            // city 2: always open
	};

	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

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

	std::vector<std::vector<time_windows::TimeWindow>> windows = {
		{{0, 100}},            // city 0
		{{0, 2}, {4, 6}},      // city 1: both windows too early for dist=10
		{{0, 100}},            // city 2
	};

	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	// From 0: 0->1 arrival=10, both windows latest=2 and 6 < 10. Rejected.
	// 0->2 arrival=3, OK. Pick 2.
	assert(solver.tour()[1] == 2);
}

// ---------------------------------------------------------------------------
// Functional: TSPTW optional windows (some cities unconstrained)
// ---------------------------------------------------------------------------

void test_tsptw_strict_optional_windows() {
	auto mat = make_mat4();

	// City 1 has a tight window, others are unconstrained.
	std::optional<time_windows::TimeWindow> windows[] = {
		std::nullopt,          // city 0: unconstrained
		time_windows::TimeWindow{0, 1},  // city 1: too tight
		std::nullopt,          // city 2: unconstrained
		std::nullopt,          // city 3: unconstrained
	};

	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	// City 1 is rejected (same as test_tsptw_strict_nn).
	assert(solver.status() == SolutionStatus::partial);
	assert(solver.tour().size() == 3);
	for (auto c : solver.tour())
		assert(c != 1);
}

void test_tsptw_relaxed_optional_windows() {
	auto mat = make_mat4();

	std::optional<time_windows::TimeWindow> windows[] = {
		std::nullopt,              // city 0: unconstrained
		std::nullopt,              // city 1: unconstrained
		time_windows::TimeWindow{0, 5},   // city 2: tight
		std::nullopt,              // city 3: unconstrained
	};

	time_windows::Relaxed relaxed(windows, 1000);
	Solver solver(mat, relaxed);
	solver.nearest_neighbor();

	// Full tour, cost includes penalties for city 2.
	assert(solver.status() == SolutionStatus::feasible);
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
// Unit: evaluate_append returns nullopt when filtered
// ---------------------------------------------------------------------------

void test_evaluate_append_filtered() {
	auto mat = make_mat4();
	RejectCity1 cb;
	Solver solver(mat, cb);
	solver.append(static_cast<std::size_t>(0));

	assert(!solver.evaluate_append(1).has_value());
	assert(solver.evaluate_append(2).has_value());
}

// ---------------------------------------------------------------------------
// Unit: position() accessor
// ---------------------------------------------------------------------------

void test_position_accessor() {
	auto mat = make_mat4();
	Solver solver(mat);

	std::vector<std::size_t> t = {0, 2, 3, 1};
	solver.set_tour(t);

	auto pos = solver.position();
	assert(pos.size() == 4);
	assert(pos[0] == 0); // city 0 at position 0
	assert(pos[2] == 1); // city 2 at position 1
	assert(pos[3] == 2); // city 3 at position 2
	assert(pos[1] == 3); // city 1 at position 3
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
		{"protocol_reversal_replay",     test_protocol_reversal_replay},
		{"protocol_two_opt_replay",      test_protocol_two_opt_replay},
		{"protocol_held_karp_logging",   test_protocol_held_karp_logging},
		{"protocol_construction_sees_prefix", test_protocol_construction_sees_prefix},
		// Unit - move_filter
		{"move_filter_reject",           test_move_filter_reject},
		// Unit - move_prepare
		{"move_prepare_changes_selection", test_move_prepare_changes_selection},
		{"fractional_cost_delta",        test_fractional_cost_delta},
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
		{"tsptw_strict_optional_windows", test_tsptw_strict_optional_windows},
		{"tsptw_relaxed_optional_windows", test_tsptw_relaxed_optional_windows},
		// Unit - neighbor lists
		{"neighbors_basic",              test_neighbors_basic},
		{"neighbors_grow",               test_neighbors_grow},
		{"neighbors_invalidate",         test_neighbors_invalidate},
		// Unit - evaluate_append
		{"evaluate_append_filtered",     test_evaluate_append_filtered},
		// Unit - position
		{"position_accessor",            test_position_accessor},
	};

	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}