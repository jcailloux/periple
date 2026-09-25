// Known bug, pinned until fixed: held_karp keeps a single state per (set, city),
// the cheapest. Under time windows with waiting, a costlier path that departs
// earlier can be the only one with a feasible completion, so held_karp reports
// infeasible although a feasible tour exists. The asserts state the bug; when
// one fails, the bug is gone: turn this file into an ordinary optimality test.

#include <periple/algorithms/held_karp.hpp>
#include <periple/distance/matrix.hpp>
#include <periple/variants/time_windows.hpp>

#include <cassert>
#include <cstdio>
#include <span>
#include <vector>

using namespace periple;

struct Instance {
	std::vector<int> weights;                       // 7 x 7, row-major
	std::vector<time_windows::TimeWindow> windows;
	int optimum;                                    // by brute force
};

int main() {
	const Instance instances[] = {
		{{ 0, 27, 11,  7, 13, 16,  1,
		  30,  0,  7,  6, 30, 18,  9,
		  22, 11,  0, 13,  5, 17, 24,
		  30, 25, 23,  0,  7,  9, 15,
		   4, 20, 22,  4,  0, 14, 14,
		  25,  8, 23,  6,  4,  0, 21,
		  20,  3, 30,  8, 17, 12,  0},
		 {{0, 1e9}, {69, 80}, {40, 75}, {33, 61}, {60, 81}, {4, 34}, {89, 125}},
		 108},
		{{ 0, 14, 19, 26,  9,  2, 24,
		   1,  0, 21, 20, 14, 13, 12,
		  22,  4,  0,  5, 21, 17, 28,
		  14, 27, 30,  0, 16,  5, 10,
		  20,  3, 12,  2,  0, 12,  2,
		  22, 27, 23, 27,  5,  0,  9,
		  11, 28, 27, 30, 29,  7,  0},
		 {{0, 1e9}, {42, 79}, {73, 84}, {34, 66}, {40, 46}, {40, 41}, {32, 67}},
		 70},
	};

	std::printf("held_karp_waiting_windows (known bug) ... ");
	std::fflush(stdout);
	for (const auto& instance : instances) {
		DistanceMatrix<int> dist(7, instance.weights);
		time_windows::Strict tw{std::span<const time_windows::TimeWindow>(instance.windows)};
		Solver solver(dist, tw);
		solver.held_karp();
		assert(solver.status() == SolutionStatus::infeasible
			&& "known bug gone: held_karp now solves this instance, turn this file into an optimality test");
	}
	std::printf("still present\n");
}
