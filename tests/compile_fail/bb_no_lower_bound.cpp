// A branch and bound strategy is its lower bound.
#include <periple/periple.hpp>

struct NotABound {};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	periple::Solver solver(mat);
	solver.branch_and_bound(NotABound{});
}
