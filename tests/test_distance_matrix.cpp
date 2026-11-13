#include <periple/distance/matrix.hpp>
#include <periple/core/solver.hpp>
#include <periple/algorithms/nearest_neighbor.hpp>

#include <cassert>
#include <cstddef>
#include <vector>

int main() {
	// --- DistanceMatrix default constructor ---
	periple::DistanceMatrix<int> m0;
	assert(m0.size() == 0);

	// --- DistanceMatrix ---

	periple::DistanceMatrix<int> m(3);
	assert(m.size() == 3);
	assert(m(0, 0) == 0);

	m(0, 1) = 5;
	m(1, 0) = 5;
	assert(m(0, 1) == 5);
	assert(m(1, 0) == 5);

	periple::DistanceMatrix<int> m2(3, {
		0, 10, 15,
	   10,  0, 20,
	   15, 20,  0
	});
	assert(m2(0, 1) == 10);
	assert(m2(1, 2) == 20);
	assert(m2(2, 0) == 15);

	std::vector<std::vector<int>> rows = {
		{0, 10, 15},
		{10, 0, 20},
		{15, 20, 0}
	};
	periple::DistanceMatrix<int> m3(rows);
	assert(m3.size() == 3);
	assert(m3(0, 1) == 10);
	assert(m3(2, 1) == 20);

	// Construct from flat vector
	std::vector<int> flat = {0, 10, 15, 10, 0, 20, 15, 20, 0};
	periple::DistanceMatrix<int> m4(3, flat);
	assert(m4.size() == 3);
	assert(m4(0, 1) == 10);
	assert(m4(1, 2) == 20);
	assert(m4(2, 0) == 15);

	// Construct from moved flat vector
	std::vector<int> flat2 = {0, 5, 5, 0};
	periple::DistanceMatrix<int> m5(2, std::move(flat2));
	assert(m5(0, 1) == 5);

	// Mutate via operator()
	m5(0, 1) = 42;
	assert(m5(0, 1) == 42);

	// Symmetry: declared by the trailing tag only, whatever the data
	assert(!m2.symmetric() && "an untagged matrix is asymmetric");
	periple::DistanceMatrix<int> t1(2, periple::symmetric);
	assert(t1.symmetric() && t1.size() == 2 && t1(0, 1) == 0 && "the tag keeps the zero-filled constructor");
	periple::DistanceMatrix<int> t2(3, {0, 10, 15, 10, 0, 20, 15, 20, 0}, periple::symmetric);
	assert(t2.symmetric() && t2(2, 1) == 20 && "the tag keeps the initializer list constructor");
	periple::DistanceMatrix<int> t3(3, flat, periple::symmetric);
	assert(t3.symmetric() && t3(2, 0) == 15 && "the tag keeps the flat vector constructor");
	periple::DistanceMatrix<int> t4(rows, periple::symmetric);
	assert(t4.symmetric() && t4(0, 2) == 15 && "the tag keeps the rows constructor");
	static_assert(!periple::detail::static_symmetry<periple::DistanceMatrix<int>>,
		"a DistanceMatrix declares symmetry from its data, at run time");

	// --- SymmetricDistanceMatrix default constructor ---
	periple::SymmetricDistanceMatrix<int> s0;
	assert(s0.size() == 0);

	// --- SymmetricDistanceMatrix ---

	periple::SymmetricDistanceMatrix<int> s(3);
	assert(s.size() == 3);

	s.set(0, 1, 7);
	assert(s(0, 1) == 7);
	assert(s(1, 0) == 7);
	s.set(2, 0, 3);
	assert(s(0, 2) == 3);

	periple::SymmetricDistanceMatrix<int> s2(rows);
	assert(s2(0, 1) == 10);
	assert(s2(1, 0) == 10);
	assert(s2(1, 2) == 20);
	assert(s2(2, 1) == 20);

	// Diagonal is always zero
	assert(s2(0, 0) == 0);
	assert(s2(1, 1) == 0);

	// Works with Solver (CTAD)
	periple::Solver solver(s2);
	solver.nearest_neighbor();

	static_assert(periple::DistanceSource<periple::DistanceMatrix<int>>);
	static_assert(periple::DistanceSource<periple::DistanceMatrix<double>>);
	static_assert(periple::DistanceSource<periple::SymmetricDistanceMatrix<int>>);
	static_assert(periple::DistanceSource<periple::SymmetricDistanceMatrix<double>>);
	static_assert(periple::detail::symmetric_by_type<periple::SymmetricDistanceMatrix<int>>,
		"a SymmetricDistanceMatrix is symmetric by its type");
}
