#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <periple/variants/service_times.hpp>

#include <cassert>
#include <cstdio>
#include <optional>
#include <vector>

using namespace periple;

// ---------------------------------------------------------------------------
// Test matrix
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
// ServiceTimes standalone
// ---------------------------------------------------------------------------

void test_service_times_standalone() {
	auto mat = make_mat4();
	double svc[] = {0, 5, 3, 2};
	service_times::ServiceTimes st(svc);
	Solver solver(mat, st);
	solver.nearest_neighbor();

	assert(solver.status() == SolutionStatus::feasible);
	assert(solver.tour().size() == 4);

	// Service times do not affect cost (they only affect departure time),
	// so the tour cost should equal the pure distance cost.
	Solver ref(mat);
	ref.nearest_neighbor();
	assert(solver.cost() == ref.cost());
}

// ---------------------------------------------------------------------------
// ServiceTimes shifts departure for TW
// ---------------------------------------------------------------------------

void test_service_times_shifts_departure() {
	// 3 cities. dist(0,1)=5, dist(1,2)=4. No service: arrival at 2 = 5+4=9.
	// With service time of 3 at city 1: departure from 1 = 5+3=8, arrival at 2 = 8+4=12.
	SymmetricDistanceMatrix<int> mat({
		{0, 5, 8},
		{5, 0, 4},
		{8, 4, 0}
	});

	double svc[] = {0, 3, 0};
	service_times::ServiceTimes st(svc);
	Solver solver(mat, st);
	solver.append(static_cast<std::size_t>(0));
	solver.append(static_cast<std::size_t>(1));

	// After appending city 1 with service time 3:
	// arrival at 1 = 5, departure = 5 + 3 = 8.
	// Now evaluate city 2: arrival = 8 + 4 = 12.
	auto score = solver.evaluate(2);
	assert(score.has_value());
	// Score = dist(1,2) + cost_delta. cost_delta = 0 (service times
	// only affect departure, not cost_delta). So score = 4.0.
	assert(*score == 4.0);
}

// ---------------------------------------------------------------------------
// Composed: ServiceTimes + Strict TW
// ---------------------------------------------------------------------------

void test_composed_svc_strict() {
	// 3 cities. dist(0,1)=5, dist(1,2)=4, dist(0,2)=8.
	// Without service: arrival at 2 via 0->1->2 = 5+4=9.
	// With service time 10 at city 1: departure from 1 = 5+10=15,
	//   arrival at 2 = 15+4=19.
	// TW city 2: [0, 12]. Without svc: arrival 9 <= 12, OK.
	//   With svc: arrival 19 > 12, rejected by Strict.
	SymmetricDistanceMatrix<int> mat({
		{0, 5, 8},
		{5, 0, 4},
		{8, 4, 0}
	});

	double svc[] = {0, 10, 0};
	time_windows::TimeWindow windows[] = {
		{0, 100},  // city 0
		{0, 100},  // city 1
		{0,  12},  // city 2: reachable without service, not with
	};

	service_times::ServiceTimes st(svc);
	time_windows::Strict tw(windows);
	auto variant = Composed(st, tw);

	// ServiceTimes first (adjusts departure), then Strict (checks feasibility).
	Solver solver(mat, variant);
	solver.nearest_neighbor();

	// From 0: nearest is 1 (dist 5). Append 1.
	// From 1: departure = 5+10 = 15. candidate 2: arrival = 15+4 = 19 > 12 -> rejected.
	// No more candidates -> partial.
	assert(solver.status() == SolutionStatus::partial);
	assert(solver.tour().size() == 2);
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 1);
}

void test_composed_svc_strict_feasible() {
	// Same setup but with generous windows: composition should still find full tour.
	SymmetricDistanceMatrix<int> mat({
		{0, 5, 8},
		{5, 0, 4},
		{8, 4, 0}
	});

	double svc[] = {0, 2, 0};
	time_windows::TimeWindow windows[] = {
		{0, 100},
		{0, 100},
		{0, 100},
	};

	service_times::ServiceTimes st(svc);
	time_windows::Strict tw(windows);
	auto variant = Composed(st, tw);
	Solver solver(mat, variant);
	solver.nearest_neighbor();

	assert(solver.status() == SolutionStatus::feasible);
	assert(solver.tour().size() == 3);
}

// ---------------------------------------------------------------------------
// Composed: ServiceTimes + Relaxed TW
// ---------------------------------------------------------------------------

void test_composed_svc_relaxed() {
	SymmetricDistanceMatrix<int> mat({
		{0, 5, 8},
		{5, 0, 4},
		{8, 4, 0}
	});

	double svc[] = {0, 10, 0};
	time_windows::TimeWindow windows[] = {
		{0, 100},
		{0, 100},
		{0,  12},  // arrival 19 > 12, violation = 7
	};

	service_times::ServiceTimes st(svc);
	time_windows::Relaxed relaxed(windows, 1000);
	auto variant = Composed(st, relaxed);
	Solver solver(mat, variant);
	solver.nearest_neighbor();

	// Relaxed never rejects, so full tour.
	assert(solver.status() == SolutionStatus::feasible);
	assert(solver.tour().size() == 3);

	// Cost includes violation penalty.
	Solver ref(mat);
	ref.nearest_neighbor();
	assert(solver.cost() > ref.cost());
}

// ---------------------------------------------------------------------------
// Composed: set_tour self-consistency
// ---------------------------------------------------------------------------

void test_composed_set_tour_consistency() {
	auto mat = make_mat4();
	double svc[] = {0, 5, 3, 2};
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 100}
	};

	service_times::ServiceTimes st(svc);
	time_windows::Relaxed relaxed(windows, 1000);
	auto variant = Composed(st, relaxed);

	Solver solver(mat, variant);
	solver.nearest_neighbor();
	auto cost1 = solver.cost();

	// Reimport the same tour via set_tour -> rebuild_and_cost.
	std::vector<std::size_t> tour_copy(solver.tour().begin(), solver.tour().end());
	Solver verifier(mat, variant);
	verifier.set_tour(tour_copy);

	assert(verifier.cost() == cost1 && "composed set_tour cost must be self-consistent");
}

// ---------------------------------------------------------------------------
// Composed: dimension deduplication (both variants share RouteTiming)
// ---------------------------------------------------------------------------

void test_composed_dimension_dedup() {
	// ServiceTimes and Strict both declare dimension = RouteTiming.
	// Composed should merge and deduplicate to a single RouteTiming.
	// This is a compile-time check: if dedup fails, EvalContext would have
	// two RouteTiming members and std::get<RouteTiming> would be ambiguous.
	auto mat = make_mat4();
	double svc[] = {0, 0, 0, 0};
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 100}
	};

	service_times::ServiceTimes st(svc);
	time_windows::Strict tw(windows);
	auto variant = Composed(st, tw);
	Solver solver(mat, variant);
	solver.nearest_neighbor();
	assert(solver.status() == SolutionStatus::feasible);
}

// ---------------------------------------------------------------------------
// Composed: variant without dimension + variant with dimension
// ---------------------------------------------------------------------------

struct PenalizeCity1 {
	template <typename Ctx>
	void move_prepare(const AppendMove<std::size_t>& m, Ctx& ctx) const {
		if (m.city == 1) ctx.cost_delta += 9999.0;
	}
};

void test_composed_mixed_dimensions() {
	// PenalizeCity1 has no dimension, Strict has RouteTiming.
	// Composed should still work, with merged dims = RouteTiming.
	auto mat = make_mat4();
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}, {0, 100}
	};

	PenalizeCity1 penalty;
	time_windows::Strict tw(windows);
	auto variant = Composed(penalty, tw);
	Solver solver(mat, variant);
	solver.nearest_neighbor();

	assert(solver.status() == SolutionStatus::feasible);
	// City 1 should not be first pick due to penalty.
	assert(solver.tour()[1] != 1);
}

// ---------------------------------------------------------------------------
// Composed: two variants without dimensions
// ---------------------------------------------------------------------------

struct RejectCity2 {
	bool move_filter(const AppendMove<std::size_t>& m) const {
		return m.city != 2;
	}
};

void test_composed_no_dimensions() {
	// Neither variant has a dimension: the context only carries the implicit CumulativeCost.
	auto mat = make_mat4();

	PenalizeCity1 penalty;
	RejectCity2 reject;
	auto variant = Composed(penalty, reject);
	Solver solver(mat, variant);
	solver.nearest_neighbor();

	// City 2 is rejected, city 1 is penalized.
	// From 0: candidates {1,3} (2 rejected). 1 has penalty 9999.
	// dist(0,1)+9999 vs dist(0,3)+0 -> pick 3 (20 < 10009).
	// From 3: candidate {1} only. 1 has penalty.
	// Pick 1 anyway (only option).
	assert(solver.status() == SolutionStatus::partial);
	assert(solver.tour().size() == 3);
	for (auto c : solver.tour())
		assert(c != 2);
}

// ---------------------------------------------------------------------------
// Composed: HK with ServiceTimes + Relaxed TW
// ---------------------------------------------------------------------------

void test_composed_hk_svc_relaxed() {
	SymmetricDistanceMatrix<int> mat({
		{0, 5, 8},
		{5, 0, 4},
		{8, 4, 0}
	});

	double svc[] = {0, 2, 0};
	time_windows::TimeWindow windows[] = {
		{0, 100}, {0, 100}, {0, 100}
	};

	service_times::ServiceTimes st(svc);
	time_windows::Relaxed relaxed(windows, 1000);
	auto variant = Composed(st, relaxed);
	Solver solver(mat, variant);
	solver.held_karp();

	assert(solver.status() == SolutionStatus::optimal);
	assert(solver.tour().size() == 3);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	struct Test { const char* name; void (*fn)(); };
	Test tests[] = {
		{"service_times_standalone",        test_service_times_standalone},
		{"service_times_shifts_departure",  test_service_times_shifts_departure},
		{"composed_svc_strict",             test_composed_svc_strict},
		{"composed_svc_strict_feasible",    test_composed_svc_strict_feasible},
		{"composed_svc_relaxed",            test_composed_svc_relaxed},
		{"composed_set_tour_consistency",   test_composed_set_tour_consistency},
		{"composed_dimension_dedup",        test_composed_dimension_dedup},
		{"composed_mixed_dimensions",       test_composed_mixed_dimensions},
		{"composed_no_dimensions",          test_composed_no_dimensions},
		{"composed_hk_svc_relaxed",         test_composed_hk_svc_relaxed},
	};

	for (const auto& t : tests) {
		std::printf("  %s ... ", t.name);
		std::fflush(stdout);
		t.fn();
		std::printf("OK\n");
	}
}