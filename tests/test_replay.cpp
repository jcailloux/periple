#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <periple/variants/service_times.hpp>

#include "solver_test_access.hpp"

#include <cassert>
#include <cstdio>
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

auto make_mat3() {
	return SymmetricDistanceMatrix<int>({
		{0, 5, 8},
		{5, 0, 4},
		{8, 4, 0}
	});
}

// ---------------------------------------------------------------------------
// Helper variant (no dimension)
// ---------------------------------------------------------------------------

struct PenalizeCity1 {
	template <typename Ctx>
	void move_prepare(const AppendMove<std::size_t>& m, Ctx& ctx) const {
		if (m.city == 1) ctx.cost_delta += 9999.0;
	}
};

// ---------------------------------------------------------------------------
// Test 1: Self-consistency -- evaluate_replay(current_tour, 1) == cost
// ---------------------------------------------------------------------------

void test_self_consistency_no_callbacks() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();

	auto result = solver.evaluate_replay(solver.tour(), 1);
	assert(result.has_value() && "self-consistency NoCallbacks: should not be filtered");
	assert(*result == solver.cost() && "self-consistency NoCallbacks: cost must match");
}

void test_self_consistency_strict() {
	auto mat = make_mat4();
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 100}
	};
	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	auto result = solver.evaluate_replay(solver.tour(), 1);
	assert(result.has_value() && "self-consistency Strict: should not be filtered");
	assert(*result == solver.cost() && "self-consistency Strict: cost must match");
}

void test_self_consistency_relaxed() {
	auto mat = make_mat4();
	// City 1 has tight window -> penalty when visited late.
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 5}, {0, 100}, {0, 100}
	};
	time_windows::Relaxed relaxed(windows, 10);
	Solver solver(mat, relaxed);
	solver.nearest_neighbor();
	// NN avoids city 1 early due to penalty, cost includes violations.

	auto result = solver.evaluate_replay(solver.tour(), 1);
	assert(result.has_value() && "self-consistency Relaxed: should not be filtered");
	assert(*result == solver.cost() && "self-consistency Relaxed: cost must match");
}

void test_self_consistency_composed() {
	auto mat = make_mat4();
	double svc[] = {0, 5, 3, 2};
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 100}
	};
	service_times::ServiceTimes st(svc);
	time_windows::Strict tw(windows);
	auto variant = Composed(st, tw);
	Solver solver(mat, variant);
	solver.nearest_neighbor();

	auto result = solver.evaluate_replay(solver.tour(), 1);
	assert(result.has_value() && "self-consistency Composed: should not be filtered");
	assert(*result == solver.cost() && "self-consistency Composed: cost must match");
}

// ---------------------------------------------------------------------------
// Test 2: Different tour -- evaluate_replay matches set_tour
// ---------------------------------------------------------------------------

void test_different_tour() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();
	// NN: [0,1,3,2], cost = 80.

	// Propose [0,1,2,3] (differs from position 2 onward).
	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2);
	assert(result.has_value() && "different tour: should be feasible");

	// Verify against set_tour.
	Solver verifier(mat);
	verifier.set_tour(proposed);
	assert(*result == verifier.cost() && "different tour: cost must match set_tour");
}

// ---------------------------------------------------------------------------
// Test 3: Strict infeasible -- suffix violates time window
// ---------------------------------------------------------------------------

void test_strict_infeasible() {
	auto mat = make_mat4();
	// City 3 window [0, 40]. NN tour [0,1,3,2]: city 3 at pos 2, arrival 35 <= 40.
	// Proposed [0,1,2,3]: city 3 at pos 3, arrival 75 > 40.
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 40}
	};
	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();
	assert(solver.status() == SolutionStatus::feasible && "setup: NN tour must be feasible");

	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2);
	assert(!result.has_value() && "strict infeasible: must return nullopt");
}

// ---------------------------------------------------------------------------
// Test 4: Strict feasible -- proposed tour passes all windows
// ---------------------------------------------------------------------------

void test_strict_feasible() {
	auto mat = make_mat4();
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 100}
	};
	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2);
	assert(result.has_value() && "strict feasible: should not be filtered");

	Solver verifier(mat, tw);
	verifier.set_tour(proposed);
	assert(*result == verifier.cost() && "strict feasible: cost must match set_tour");
}

// ---------------------------------------------------------------------------
// Test 5: Relaxed penalties -- proposed tour incurs penalties
// ---------------------------------------------------------------------------

void test_relaxed_penalties() {
	auto mat = make_mat4();
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 5}, {0, 100}, {0, 100}
	};
	time_windows::Relaxed relaxed(windows, 10);
	Solver solver(mat, relaxed);
	solver.nearest_neighbor();
	// NN with penalties: [0,2,3,1].

	// Propose [0,2,1,3] (from_pos = 2, different suffix).
	std::vector<std::size_t> proposed = {0, 2, 1, 3};
	auto result = solver.evaluate_replay(proposed, 2);
	assert(result.has_value() && "relaxed penalties: Relaxed never rejects");

	Solver verifier(mat, relaxed);
	verifier.set_tour(proposed);
	assert(*result == verifier.cost() && "relaxed penalties: cost must match set_tour");
}

// ---------------------------------------------------------------------------
// Test 6: Composed -- ServiceTimes + Strict coherence with set_tour
// ---------------------------------------------------------------------------

void test_composed_coherence() {
	auto mat = make_mat4();
	double svc[] = {0, 5, 3, 2};
	// Wide windows: service times shift departures, return can reach 105.
	time_windows::TimeWindow windows[] = {
		{0, 200}, {0, 200}, {0, 200}, {0, 200}
	};
	service_times::ServiceTimes st(svc);
	time_windows::Strict tw(windows);
	auto variant = Composed(st, tw);
	Solver solver(mat, variant);
	solver.nearest_neighbor();

	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2);
	assert(result.has_value() && "composed coherence: should not be filtered");

	Solver verifier(mat, variant);
	verifier.set_tour(proposed);
	assert(*result == verifier.cost() && "composed coherence: cost must match set_tour");
}

// ---------------------------------------------------------------------------
// Test 7: has_callbacks compile-time
// ---------------------------------------------------------------------------

void test_has_callbacks() {
	using MatT = SymmetricDistanceMatrix<int>;
	static_assert(!Solver<MatT>::has_callbacks,
		"NoCallbacks: has_callbacks must be false");
	static_assert(Solver<MatT, time_windows::Strict>::has_callbacks,
		"Strict: has_callbacks must be true");
	static_assert(Solver<MatT, PenalizeCity1>::has_callbacks,
		"PenalizeCity1 (no dimension): has_callbacks must be true");
}

// ---------------------------------------------------------------------------
// Test 8: cumul_costs consistency
// ---------------------------------------------------------------------------

void test_cumul_costs_consistency() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();
	// NN: [0,1,3,2], cost = 80.

	SolverTestAccess access(solver);
	auto costs = access.cumul_costs();
	auto t = solver.tour();

	assert(costs[0] == 0 && "cumul_costs[0] must be 0");

	// Recompute from tour edges and verify each position.
	int running = 0;
	for (std::size_t i = 0; i < t.size(); ++i) {
		assert(costs[i] == running && "cumul_costs[i] must equal sum of edges to position i");
		if (i + 1 < t.size())
			running += mat(t[i], t[i + 1]);
	}
}

// ---------------------------------------------------------------------------
// Test 9: Successive calls with same from_pos
// ---------------------------------------------------------------------------

void test_successive_calls() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();
	// NN: [0,1,3,2].

	std::vector<std::size_t> tour_a = {0, 1, 2, 3};
	auto result_a = solver.evaluate_replay(tour_a, 2);
	assert(result_a.has_value() && "successive calls: first must succeed");

	std::vector<std::size_t> tour_b = {0, 1, 3, 2};
	auto result_b = solver.evaluate_replay(tour_b, 2);
	assert(result_b.has_value() && "successive calls: second must succeed");

	Solver va(mat); va.set_tour(tour_a);
	Solver vb(mat); vb.set_tour(tour_b);
	assert(*result_a == va.cost() && "successive calls: first result must match");
	assert(*result_b == vb.cost() && "successive calls: second result must match");
}

// ---------------------------------------------------------------------------
// Test 10: Early exit -- filter rejects in the replayed suffix
// ---------------------------------------------------------------------------

void test_early_exit() {
	auto mat = make_mat4();
	// City 3 window [0, 40]. NN tour [0,1,3,2]: city 3 at pos 2, arrival 35 <= 40.
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 40}
	};
	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();
	assert(solver.status() == SolutionStatus::feasible);

	// Propose [0,1,2,3]: city 2 at pos 2 (arrival 45, OK),
	// city 3 at pos 3 (arrival 75 > 40, rejected). Early exit at pos 3.
	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2);
	assert(!result.has_value() && "early exit: must return nullopt");
}

// ---------------------------------------------------------------------------
// Test 11: close_tour infeasible -- return-to-depot violates window
// ---------------------------------------------------------------------------

void test_close_tour_infeasible() {
	auto mat = make_mat3();
	// Depot window [0, 10]. Tour 0->1->2: return arrival = 9+8 = 17 > 10.
	time_windows::TimeWindow windows[] = {
		{0, 10}, {0, 100}, {0, 100}
	};
	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	assert(solver.status() == SolutionStatus::infeasible
		&& "close_tour: return violates depot window");
	assert(solver.tour().size() == 3
		&& "close_tour: all cities should be in the tour");
}

// ---------------------------------------------------------------------------
// Test 12: close_tour feasible -- generous depot window
// ---------------------------------------------------------------------------

void test_close_tour_feasible() {
	auto mat = make_mat3();
	// Depot window [0, 20]. Tour 0->1->2: return arrival = 9+8 = 17 <= 20.
	time_windows::TimeWindow windows[] = {
		{0, 20}, {0, 100}, {0, 100}
	};
	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();

	assert(solver.status() == SolutionStatus::feasible
		&& "close_tour: return within depot window");
	assert(solver.tour().size() == 3);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	struct Test { const char* name; void (*fn)(); };
	Test tests[] = {
		// Self-consistency
		{"self_consistency_no_callbacks", test_self_consistency_no_callbacks},
		{"self_consistency_strict",       test_self_consistency_strict},
		{"self_consistency_relaxed",      test_self_consistency_relaxed},
		{"self_consistency_composed",     test_self_consistency_composed},
		// evaluate_replay correctness
		{"different_tour",               test_different_tour},
		{"strict_infeasible",            test_strict_infeasible},
		{"strict_feasible",              test_strict_feasible},
		{"relaxed_penalties",            test_relaxed_penalties},
		{"composed_coherence",           test_composed_coherence},
		// Compile-time
		{"has_callbacks",                test_has_callbacks},
		// cumul_costs
		{"cumul_costs_consistency",      test_cumul_costs_consistency},
		// Successive calls
		{"successive_calls",             test_successive_calls},
		// Early exit
		{"early_exit",                   test_early_exit},
		// close_tour
		{"close_tour_infeasible",        test_close_tour_infeasible},
		{"close_tour_feasible",          test_close_tour_feasible},
	};

	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}