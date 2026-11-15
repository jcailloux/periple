// The deadline test reads the windows of one Strict: two are rejected.
#include <periple/periple.hpp>

#include <vector>

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	const std::vector<periple::time_windows::TimeWindow> windows(3, {0.0, 10.0});
	const periple::time_windows::Strict first(windows), second(windows);
	const auto both = periple::Composed(first, second);
	periple::Solver solver(mat, both);
	solver.branch_and_bound(periple::time_windows::DeadlineBound(periple::SpanningTreeBound{}));
}
