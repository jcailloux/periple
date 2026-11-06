// With a variant the cost depends on the start: a held tour starting elsewhere
// is not one of the tours searched.
#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>

#include <vector>

int main() {
	periple::SymmetricDistanceMatrix<int> mat({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	std::vector<periple::time_windows::TimeWindow> windows(4, {0.0, 1000.0});
	periple::time_windows::Strict variant(windows);
	periple::Solver solver(mat, variant);
	std::vector<std::size_t> tour = {1, 0, 2, 3};
	solver.set_tour(tour);
	solver.branch_and_bound();
}
