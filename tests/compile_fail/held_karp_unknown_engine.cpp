// held_karp takes one of its two engines.
#include <periple/periple.hpp>

struct NotAnEngine {};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	periple::Solver solver(mat);
	solver.held_karp(NotAnEngine{});
}
