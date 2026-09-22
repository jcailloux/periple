#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <periple/variants/service_times.hpp>

#include <cassert>
#include <cstdio>
#include <type_traits>
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

// Fractional parts accumulate past 1: summing in double before truncation
// would diverge from Solver::append (per-edge cast to cost_type).
struct FractionalPenalty {
	template <typename Ctx>
	void move_prepare(const AppendMove<std::size_t>& m, Ctx& ctx) const {
		ctx.cost_delta += 0.6 + 0.1 * static_cast<double>(m.city);
	}
};

struct FlatPenalty {
	template <typename Ctx>
	void move_prepare(const AppendMove<std::size_t>&, Ctx& ctx) const {
		ctx.cost_delta += 7.0;
	}
};

// Explicitly declares CumulativeCost: context_for must not duplicate it.
struct ExplicitCumulative {
	using dimension = Dimensions<CumulativeCost, RouteTiming>;
};

// ---------------------------------------------------------------------------
// Test 1: Self-consistency -- evaluate_replay(current_tour, 1) == cost
// ---------------------------------------------------------------------------

void test_self_consistency_no_callbacks() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();

	auto result = solver.evaluate_replay(solver.tour(), 1, 0);
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

	auto result = solver.evaluate_replay(solver.tour(), 1, 0);
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

	auto result = solver.evaluate_replay(solver.tour(), 1, 0);
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

	auto result = solver.evaluate_replay(solver.tour(), 1, 0);
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
	// prefix_cost = dist(0,1) = 10
	auto result = solver.evaluate_replay(proposed, 2, 10);
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
	auto result = solver.evaluate_replay(proposed, 2, 10);
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
	auto result = solver.evaluate_replay(proposed, 2, 10);
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
	// prefix_cost = dist(0,2) = 15 (no penalty in prefix)
	auto result = solver.evaluate_replay(proposed, 2, 15);
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
	// prefix_cost = dist(0,1) = 10 (ServiceTimes/Strict don't add cost_delta)
	auto result = solver.evaluate_replay(proposed, 2, 10);
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
// Test 8: Staging isolation -- evaluate_replay preserves solver state
// ---------------------------------------------------------------------------

void test_staging_isolation() {
	auto mat = make_mat4();
	time_windows::TimeWindow windows[] = {
		{0, 200}, {0, 200}, {0, 200}, {0, 200}
	};
	time_windows::Strict tw(windows);
	Solver solver(mat, tw);
	solver.nearest_neighbor();
	// NN: [0,1,3,2].

	auto original_cost = solver.cost();
	auto original_tour = std::vector<std::size_t>(solver.tour().begin(), solver.tour().end());

	// Evaluate a different tour.
	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2, 10);
	assert(result.has_value() && "staging isolation: eval should succeed");

	// Solver state must be unchanged.
	assert(solver.cost() == original_cost && "staging isolation: cost must not change");
	for (std::size_t i = 0; i < original_tour.size(); ++i)
		assert(solver.tour()[i] == original_tour[i] && "staging isolation: tour must not change");

	// Replaying the original tour must still give the original cost.
	auto self = solver.evaluate_replay(solver.tour(), 1, 0);
	assert(self.has_value() && *self == original_cost && "staging isolation: self-replay must match");
}

// ---------------------------------------------------------------------------
// Test 9: Save and accept -- staging lifecycle
// ---------------------------------------------------------------------------

void test_save_and_accept() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();
	// NN: [0,1,3,2], cost = 80.

	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2, 10);
	assert(result.has_value() && "save_and_accept: eval should succeed");

	solver.save_staging();
	solver.accept_replay([&](std::size_t i) { return proposed[i]; }, 2, *result);

	assert(solver.cost() == *result && "save_and_accept: cost must match");
	assert(solver.tour()[2] == 2 && "save_and_accept: tour[2] must be updated");
	assert(solver.tour()[3] == 3 && "save_and_accept: tour[3] must be updated");

	// Self-replay must still be consistent.
	auto self = solver.evaluate_replay(solver.tour(), 1, 0);
	assert(self.has_value() && *self == solver.cost() && "save_and_accept: self-replay must match");
}

// ---------------------------------------------------------------------------
// Test 10: Reject then accept -- record survives subsequent evaluation
// ---------------------------------------------------------------------------

void test_reject_then_accept() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();
	// NN: [0,1,3,2], cost = 80.

	// Evaluate and save tour A.
	std::vector<std::size_t> tour_a = {0, 1, 2, 3};
	auto cost_a = solver.evaluate_replay(tour_a, 2, 10);
	assert(cost_a.has_value() && "reject_then_accept: tour_a eval should succeed");
	solver.save_staging();

	// Evaluate tour B (overwrites staging, but record is preserved).
	std::vector<std::size_t> tour_b = {0, 1, 3, 2};
	auto cost_b = solver.evaluate_replay(tour_b, 2, 10);
	assert(cost_b.has_value() && "reject_then_accept: tour_b eval should succeed");
	// Do NOT save_staging -- we want to accept tour A's record.

	// Accept the recorded tour A.
	solver.accept_replay([&](std::size_t i) { return tour_a[i]; }, 2, *cost_a);

	assert(solver.cost() == *cost_a && "reject_then_accept: cost must match tour_a");
	assert(solver.tour()[2] == 2 && "reject_then_accept: tour must reflect tour_a");

	// Self-replay must be consistent.
	auto self = solver.evaluate_replay(solver.tour(), 1, 0);
	assert(self.has_value() && *self == solver.cost() && "reject_then_accept: self-replay must match");
}

// ---------------------------------------------------------------------------
// Test 11: Direct commit without record
// ---------------------------------------------------------------------------

void test_direct_commit() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();
	// NN: [0,1,3,2], cost = 80.

	std::vector<std::size_t> proposed = {0, 1, 2, 3};
	auto result = solver.evaluate_replay(proposed, 2, 10);
	assert(result.has_value() && "direct commit: eval should succeed");

	// Accept without save_staging -- commit takes staging directly.
	solver.accept_replay([&](std::size_t i) { return proposed[i]; }, 2, *result);

	assert(solver.cost() == *result && "direct commit: cost must match");
	assert(solver.tour()[2] == 2 && "direct commit: tour must be updated");

	auto self = solver.evaluate_replay(solver.tour(), 1, 0);
	assert(self.has_value() && *self == solver.cost() && "direct commit: self-replay must match");
}

// ---------------------------------------------------------------------------
// Test 12: TwoOptMove city_at correctness
// ---------------------------------------------------------------------------

void test_two_opt_city_at() {
	std::vector<std::size_t> tour = {0, 1, 2, 3, 4};
	TwoOptMove<std::size_t> move{1, 3}; // reverse segment (1, 3] = {2, 3}

	auto city_at = move.city_at(tour);
	assert(city_at(0) == 0 && "two_opt city_at: pos 0 unchanged");
	assert(city_at(1) == 1 && "two_opt city_at: pos 1 unchanged (boundary)");
	assert(city_at(2) == 3 && "two_opt city_at: pos 2 reversed");
	assert(city_at(3) == 2 && "two_opt city_at: pos 3 reversed");
	assert(city_at(4) == 4 && "two_opt city_at: pos 4 unchanged");
}

// ---------------------------------------------------------------------------
// Test 13: Span overload matches callable
// ---------------------------------------------------------------------------

void test_span_overload() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();

	std::vector<std::size_t> proposed = {0, 1, 2, 3};

	auto result_span = solver.evaluate_replay(
	    std::span<const std::size_t>(proposed), 2, 10);
	auto result_callable = solver.evaluate_replay(
	    [&](std::size_t i) { return proposed[i]; }, 2, 10);

	assert(result_span.has_value() && result_callable.has_value()
	    && "span overload: both must succeed");
	assert(*result_span == *result_callable
	    && "span overload: results must match");
}

// ---------------------------------------------------------------------------
// Test 14: Successive calls with same from_pos
// ---------------------------------------------------------------------------

void test_successive_calls() {
	auto mat = make_mat4();
	Solver solver(mat);
	solver.nearest_neighbor();
	// NN: [0,1,3,2].

	std::vector<std::size_t> tour_a = {0, 1, 2, 3};
	auto result_a = solver.evaluate_replay(tour_a, 2, 10);
	assert(result_a.has_value() && "successive calls: first must succeed");

	std::vector<std::size_t> tour_b = {0, 1, 3, 2};
	auto result_b = solver.evaluate_replay(tour_b, 2, 10);
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
	auto result = solver.evaluate_replay(proposed, 2, 10);
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
// Test 15: prefix_cost -- replaying from prefix_cost(k-1) gives the tour cost
// ---------------------------------------------------------------------------

template <typename SolverT>
void check_prefix_cost_replay(const SolverT& solver) {
	assert(solver.prefix_cost(0) == 0 && "prefix_cost(0): the first city contributes nothing");
	const auto n = solver.tour().size();
	for (std::size_t k = 1; k < n; ++k) {
		auto result = solver.evaluate_replay(solver.tour(), k, solver.prefix_cost(k - 1));
		assert(result.has_value() && "prefix_cost: self-replay must be feasible");
		assert(*result == solver.cost()
			&& "prefix_cost: replay from prefix_cost(k-1) must give the tour cost");
	}
}

void test_prefix_cost_matches_replay_relaxed() {
	auto mat = make_mat4();
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 5}, {0, 100}, {0, 100}
	};
	time_windows::Relaxed relaxed(windows, 10);
	Solver solver(mat, relaxed);
	solver.nearest_neighbor();
	assert(solver.status() == SolutionStatus::feasible && "setup: Relaxed never rejects");
	check_prefix_cost_replay(solver);
}

void test_prefix_cost_fractional_penalty() {
	auto mat = make_mat4();
	FractionalPenalty penalty;
	Solver solver(mat, penalty);
	solver.nearest_neighbor();
	assert(solver.status() == SolutionStatus::feasible && "setup: FractionalPenalty never rejects");
	check_prefix_cost_replay(solver);
}

void test_prefix_cost_pos0_ignores_delta() {
	auto mat = make_mat4();
	FlatPenalty flat;
	Solver solver(mat, flat);
	solver.nearest_neighbor();

	assert(solver.prefix_cost(0) == 0
		&& "prefix_cost(0): the first city contributes nothing, even with a penalty");
	const auto t0 = solver.tour()[0];
	const auto t1 = solver.tour()[1];
	assert(solver.prefix_cost(1) == solver.distance(t0, t1) + 7
		&& "prefix_cost(1): first edge plus its penalty");
}

// ---------------------------------------------------------------------------
// Test 16: context_for -- CumulativeCost is implicit for every variant
// ---------------------------------------------------------------------------

void test_context_auto_cumulative_cost() {
	using MatT = SymmetricDistanceMatrix<int>;
	static_assert(!Solver<MatT>::context_type::has_dim<CumulativeCost>,
		"NoCallbacks: no dimensions");
	static_assert(Solver<MatT, PenalizeCity1>::context_type::has_dim<CumulativeCost>,
		"variant without dimension: CumulativeCost is implicit");
	static_assert(Solver<MatT, time_windows::Strict>::context_type::has_dim<CumulativeCost>
		&& Solver<MatT, time_windows::Strict>::context_type::has_dim<RouteTiming>,
		"Strict: declared RouteTiming plus implicit CumulativeCost");
	static_assert(Solver<MatT, time_windows::Relaxed>::context_type::has_dim<CumulativeCost>
		&& Solver<MatT, time_windows::Relaxed>::context_type::has_dim<RouteTiming>,
		"Relaxed: declared RouteTiming plus implicit CumulativeCost");
	static_assert(std::is_same_v<Solver<MatT, ExplicitCumulative>::context_type,
	                             EvalContext<std::size_t, int, CumulativeCost, RouteTiming>>,
		"explicit CumulativeCost: deduplicated, not doubled");
}

// ---------------------------------------------------------------------------
// Test 17: prefix_cost reads committed values while a staging is active
// ---------------------------------------------------------------------------

void test_committed_read_during_staging() {
	auto mat = make_mat4();
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 5}, {0, 100}, {0, 100}
	};
	time_windows::Relaxed relaxed(windows, 10);
	Solver solver(mat, relaxed);
	solver.nearest_neighbor();
	// NN with penalties: [0,2,3,1].

	// Evaluate a different suffix from position 2 and leave the staging active.
	std::vector<std::size_t> other = {0, 2, 1, 3};
	auto pending = solver.evaluate_replay(other, 2, solver.prefix_cost(1));
	assert(pending.has_value() && "committed read: Relaxed never rejects");
	assert(*pending != solver.cost() && "setup: the other suffix must cost differently");

	// prefix_cost(2) must return the committed value, not the staged one.
	auto self = solver.evaluate_replay(solver.tour(), 3, solver.prefix_cost(2));
	assert(self.has_value() && *self == solver.cost()
		&& "committed read: prefix_cost must ignore the staged suffix");

	solver.discard_staging();
	auto again = solver.evaluate_replay(solver.tour(), 1, 0);
	assert(again.has_value() && *again == solver.cost()
		&& "committed read: state unchanged after discard_staging");
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
		// Staging
		{"staging_isolation",            test_staging_isolation},
		{"save_and_accept",              test_save_and_accept},
		{"reject_then_accept",           test_reject_then_accept},
		{"direct_commit",                test_direct_commit},
		// TwoOptMove
		{"two_opt_city_at",              test_two_opt_city_at},
		// Span overload
		{"span_overload",                test_span_overload},
		// Successive calls
		{"successive_calls",             test_successive_calls},
		// Early exit
		{"early_exit",                   test_early_exit},
		// close_tour
		{"close_tour_infeasible",        test_close_tour_infeasible},
		{"close_tour_feasible",          test_close_tour_feasible},
		// prefix_cost / CumulativeCost
		{"prefix_cost_matches_replay_relaxed", test_prefix_cost_matches_replay_relaxed},
		{"prefix_cost_fractional_penalty",     test_prefix_cost_fractional_penalty},
		{"prefix_cost_pos0_ignores_delta",     test_prefix_cost_pos0_ignores_delta},
		{"context_auto_cumulative_cost",       test_context_auto_cumulative_cost},
		{"committed_read_during_staging",      test_committed_read_during_staging},
	};

	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}