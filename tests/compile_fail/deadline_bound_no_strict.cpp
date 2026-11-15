// The deadline test reads the windows of the Strict in the solver's variant.
#include <periple/periple.hpp>

#include <vector>

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	const std::vector<periple::time_windows::TimeWindow> windows(3, {0.0, 10.0});
	const periple::time_windows::Relaxed relaxed(windows);
	periple::Solver solver(mat, relaxed);
	solver.branch_and_bound(periple::time_windows::DeadlineBound(periple::SpanningTreeBound{}));
}
