// A branch and bound order has order or pick.
#include <periple/periple.hpp>

struct NotAnOrder {};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	periple::Solver solver(mat);
	solver.branch_and_bound(periple::AssignmentBound{}, NotAnOrder{});
}
