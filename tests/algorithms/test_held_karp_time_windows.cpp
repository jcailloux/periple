// held_karp under time windows, against optima found by brute force. Under
// waiting or penalties, a costlier partial tour that departs earlier can be the
// one with the only feasible or the cheapest completion: held_karp() keeps every
// non-dominated state, and AllSets and ReachableSets, which keep one state per
// (set, city), claim no optimum or infeasibility they cannot prove. A depot that
// opens late: the DP departs from city 0 as append() places it.

#include <periple/algorithms/held_karp.hpp>
#include <periple/distance/matrix.hpp>
#include <periple/variants/time_windows.hpp>

#include <cassert>
#include <cstdio>
#include <span>
#include <vector>

using namespace periple;

struct Instance {
	std::size_t n;
	std::vector<int> weights;                       // n x n, row-major
	std::vector<time_windows::TimeWindow> windows;
	bool relaxed;                                   // Relaxed with penalty weight 10, else Strict
	int optimum;                                    // by brute force
	SolutionStatus one_state;                       // under AllSets and ReachableSets
};

// held_karp reaches status: at the optimum when optimal, above it when feasible.
template <typename Variant, typename... Engine>
void check_under(const Instance& instance, SolutionStatus status, const Variant& variant, Engine... engine) {
	DistanceMatrix<int> dist(instance.n, instance.weights);
	Solver solver(dist, variant);
	solver.held_karp(engine...);
	assert(solver.status() == status && "held_karp must reach the status expected of its engine");
	assert((status != SolutionStatus::optimal || solver.cost() == instance.optimum)
		&& "held_karp must reach the brute-force optimum");
	assert((status != SolutionStatus::feasible || solver.cost() > instance.optimum)
		&& "under one state per (set, city), this instance's tour is costlier than the optimum");
	assert((status != SolutionStatus::partial || solver.tour().size() == 1)
		&& "without a tour, the solver holds the start city alone");
}

template <typename... Engine>
void check(const Instance& instance, SolutionStatus status, Engine... engine) {
	const std::span<const time_windows::TimeWindow> windows(instance.windows);
	if (instance.relaxed)
		check_under(instance, status, time_windows::Relaxed(windows, 10), engine...);
	else
		check_under(instance, status, time_windows::Strict(windows), engine...);
}

int main() {
	const Instance instances[] = {
		// Strict with waiting: one state per (set, city) finds no tour.
		{7,
		 { 0, 27, 11,  7, 13, 16,  1,
		  30,  0,  7,  6, 30, 18,  9,
		  22, 11,  0, 13,  5, 17, 24,
		  30, 25, 23,  0,  7,  9, 15,
		   4, 20, 22,  4,  0, 14, 14,
		  25,  8, 23,  6,  4,  0, 21,
		  20,  3, 30,  8, 17, 12,  0},
		 {{0, 1e9}, {69, 80}, {40, 75}, {33, 61}, {60, 81}, {4, 34}, {89, 125}},
		 false, 108, SolutionStatus::partial},
		{7,
		 { 0, 14, 19, 26,  9,  2, 24,
		   1,  0, 21, 20, 14, 13, 12,
		  22,  4,  0,  5, 21, 17, 28,
		  14, 27, 30,  0, 16,  5, 10,
		  20,  3, 12,  2,  0, 12,  2,
		  22, 27, 23, 27,  5,  0,  9,
		  11, 28, 27, 30, 29,  7,  0},
		 {{0, 1e9}, {42, 79}, {73, 84}, {34, 66}, {40, 46}, {40, 41}, {32, 67}},
		 false, 70, SolutionStatus::partial},
		// Strict with waiting: one state per (set, city) finds a costlier tour.
		{6,
		 { 0,  1, 10,  9,  4,  3,
		  12,  0, 18,  3, 17, 17,
		   1, 20,  0,  5, 15, 29,
		  12, 20,  1,  0, 27, 23,
		  21, 26, 26,  5,  0, 19,
		  27, 29, 10, 14, 18,  0},
		 {{0, 1e9}, {46, 91}, {0, 17}, {71, 130}, {48, 76}, {30, 77}},
		 false, 94, SolutionStatus::feasible},
		// Strict with waiting: into ({0, 1, 2, 3}, 3), 0, 2, 1, 3 arrives at 20 for 20,
		// and 0, 1, 2, 3, cheaper at 17, waits at city 2 and arrives at 25: city 4, 5
		// away, closes at 27. The cheaper state misses a deadline: dropping it loses
		// nothing, and one state per (set, city) proves the optimum.
		{5,
		 {  0,   1,  10, 100, 100,
		  100,   0,   1,   5, 100,
		  100,   5,   0,  15, 100,
		  100, 100, 100,   0,   5,
		    1, 100, 100, 100,   0},
		 {{0, 1e9}, {0, 1e9}, {10, 1e9}, {0, 1e9}, {0, 27}},
		 false, 26, SolutionStatus::optimal},
		// Relaxed, deadlines only: one state per (set, city) finds a costlier tour.
		{6,
		 { 0, 21, 16, 25,  8, 17,
		   5,  0,  2, 25, 22, 27,
		  10, 29,  0, 19,  8,  3,
		  22,  6, 18,  0,  6, 16,
		  17, 26, 21,  8,  0, 11,
		  16, 12,  4, 25, 26,  0},
		 {{0, 1e9}, {0, 42}, {0, 16}, {0, 36}, {0, 54}, {0, 44}},
		 true, 118, SolutionStatus::feasible},
		// Relaxed, deadlines only: one state per (set, city) finds a costlier tour,
		// losing only states that cheaper ones replace.
		{5,
		 { 0, 17, 10, 18, 20,
		  14,  0, 21, 13,  2,
		  27, 16,  0, 29, 30,
		   6, 16, 20,  0,  1,
		   2, 28, 30, 30,  0},
		 {{0, 1e9}, {0, 16}, {0, 42}, {0, 74}, {0, 61}},
		 true, 142, SolutionStatus::feasible},
		// Strict, deadlines only, the depot opening at 25: departing at 0 instead
		// reports a tour of 59 that misses a deadline. Without waiting elsewhere,
		// one state per (set, city) suffices: every engine reaches 60.
		{6,
		 { 0,  5, 20, 10,  7,  2,
		  20,  0, 14,  8, 26, 17,
		  28, 19,  0, 22, 23, 24,
		  27,  9, 12,  0, 20, 11,
		   3, 12,  3, 16,  0, 18,
		  12, 11, 30, 30, 29,  0},
		 {{25, 1e9}, {0, 63}, {0, 47}, {0, 134}, {0, 87}, {0, 104}},
		 false, 60, SolutionStatus::optimal},
	};

	std::printf("held_karp_time_windows ... ");
	std::fflush(stdout);
	for (const auto& instance : instances) {
		check(instance, SolutionStatus::optimal);
		check(instance, instance.one_state, AllSets{});
		check(instance, instance.one_state, ReachableSets{});
	}
	std::printf("OK\n");
}
