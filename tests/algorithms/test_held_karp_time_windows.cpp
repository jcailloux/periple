// Known bug, pinned until fixed: held_karp keeps a single state per (set, city),
// the cheapest. Under time windows, a costlier partial tour that arrives earlier
// can be the one with the only feasible or the cheapest completion, so held_karp
// reports infeasible for a feasible instance or returns a costlier tour as
// optimal. The asserts state the bug; when one fails, the bug is gone: turn this
// file into an ordinary optimality test.

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
};

template <typename Variant>
void check(const Instance& instance, const Variant& variant) {
	DistanceMatrix<int> dist(instance.n, instance.weights);
	Solver solver(dist, variant);
	solver.held_karp();
	assert(!(solver.status() == SolutionStatus::optimal && solver.cost() == instance.optimum)
		&& "known bug gone: held_karp now reaches the optimum, turn this file into an optimality test");
}

int main() {
	const Instance instances[] = {
		// Strict with waiting: reported infeasible.
		{7,
		 { 0, 27, 11,  7, 13, 16,  1,
		  30,  0,  7,  6, 30, 18,  9,
		  22, 11,  0, 13,  5, 17, 24,
		  30, 25, 23,  0,  7,  9, 15,
		   4, 20, 22,  4,  0, 14, 14,
		  25,  8, 23,  6,  4,  0, 21,
		  20,  3, 30,  8, 17, 12,  0},
		 {{0, 1e9}, {69, 80}, {40, 75}, {33, 61}, {60, 81}, {4, 34}, {89, 125}},
		 false, 108},
		{7,
		 { 0, 14, 19, 26,  9,  2, 24,
		   1,  0, 21, 20, 14, 13, 12,
		  22,  4,  0,  5, 21, 17, 28,
		  14, 27, 30,  0, 16,  5, 10,
		  20,  3, 12,  2,  0, 12,  2,
		  22, 27, 23, 27,  5,  0,  9,
		  11, 28, 27, 30, 29,  7,  0},
		 {{0, 1e9}, {42, 79}, {73, 84}, {34, 66}, {40, 46}, {40, 41}, {32, 67}},
		 false, 70},
		// Strict with waiting: a costlier tour reported optimal.
		{6,
		 { 0, 11,  4, 21, 20,  1,
		  17,  0, 22, 18, 20, 26,
		   8, 13,  0, 11, 20, 18,
		  10, 30, 30,  0, 23, 19,
		   2, 14, 25, 12,  0,  4,
		  27, 28,  7, 15, 20,  0},
		 {{0, 1e9}, {39, 86}, {49, 88}, {3, 48}, {47, 136}, {30, 54}},
		 false, 82},
		// Relaxed, deadlines only: a costlier tour reported optimal.
		{6,
		 { 0, 21, 16, 25,  8, 17,
		   5,  0,  2, 25, 22, 27,
		  10, 29,  0, 19,  8,  3,
		  22,  6, 18,  0,  6, 16,
		  17, 26, 21,  8,  0, 11,
		  16, 12,  4, 25, 26,  0},
		 {{0, 1e9}, {0, 42}, {0, 16}, {0, 36}, {0, 54}, {0, 44}},
		 true, 118},
	};

	std::printf("held_karp_time_windows (known bug) ... ");
	std::fflush(stdout);
	for (const auto& instance : instances) {
		const std::span<const time_windows::TimeWindow> windows(instance.windows);
		if (instance.relaxed)
			check(instance, time_windows::Relaxed(windows, 10));
		else
			check(instance, time_windows::Strict(windows));
	}
	std::printf("still present\n");
}
