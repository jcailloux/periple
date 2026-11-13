// A matrix tagged symmetric is checked when the solver receives it.
#include <periple/periple.hpp>

int main() {
	periple::DistanceMatrix<int> mat(3, {0, 1, 9, 9, 0, 1, 1, 9, 0}, periple::symmetric);
	periple::Solver solver(mat);
}
