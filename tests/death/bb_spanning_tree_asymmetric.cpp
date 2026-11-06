// SpanningTreeBound reads each edge in one direction only.
#include <periple/periple.hpp>

int main() {
	periple::DistanceMatrix<int> mat(3, {0, 1, 9, 9, 0, 1, 1, 9, 0});
	periple::Solver solver(mat);
	solver.branch_and_bound(periple::SpanningTreeBound{});
}
