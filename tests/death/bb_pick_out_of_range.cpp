// pick returns an index into the children it is given.
#include <periple/periple.hpp>

#include <cstddef>
#include <span>

struct PastTheEnd {
	template <typename Solver, typename Child>
	std::size_t pick(const Solver&, std::span<const Child> remaining, double) const { return remaining.size(); }
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	periple::Solver solver(mat);
	solver.branch_and_bound(periple::AssignmentBound{}, PastTheEnd{});
}
