#include <periple/periple.hpp>

#include <cassert>
#include <cstddef>
#include <vector>

int main() {
	periple::DistanceMatrix<int> mat(4, {
		 0, 10, 15, 20,
		10,  0, 35, 25,
		15, 35,  0, 30,
		20, 25, 30,  0
	});

	// --- Default construction ---
	periple::Solver<periple::DistanceMatrix<int>> solver;
	assert(solver.status() == periple::SolutionStatus::none);
	assert(solver.tour().empty());
	assert(solver.cost() == 0);

	// --- set_matrix + chaining ---
	solver.set_matrix(mat);
	solver.nearest_neighbor().held_karp();
	assert(solver.cost() == 80);

	// --- set_tour ---
	std::vector<std::size_t> manual = {0, 2, 3, 1};
	solver.set_tour(manual);
	assert(solver.tour().size() == 4);
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 2);

	// --- clear ---
	solver.clear();
	assert(solver.status() == periple::SolutionStatus::none);
	assert(solver.tour().empty());
	assert(solver.cost() == 0);

	// --- reset ---
	solver.set_matrix(mat);
	solver.nearest_neighbor();
	assert(solver.cost() > 0);
	solver.reset();
	assert(solver.status() == periple::SolutionStatus::none);
	assert(solver.tour().empty());
	assert(solver.cost() == 0);
	assert(solver.size() == 0);

	// --- Reuse solver with different instance ---
	solver.set_matrix(mat);
	periple::DistanceMatrix<int> mat2(3, {0, 1, 2, 1, 0, 3, 2, 3, 0});
	solver.set_matrix(mat2);
	solver.nearest_neighbor();
	assert(solver.tour().size() == 3);

	// --- start_city parameter ---
	solver.set_matrix(mat);
	solver.nearest_neighbor({.start_city = 2});
	assert(solver.tour()[0] == 2);
	solver.nearest_neighbor({.start_city = 0});
	assert(solver.tour()[0] == 0);
}
