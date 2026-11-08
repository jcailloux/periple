// An order permutes the children it is given: one that changes them must fail
// loudly instead of searching another tree.
#include <periple/periple.hpp>

#include <span>

struct Duplicates {
	template <typename Solver, typename Child>
	void order(const Solver&, std::span<Child> children, double) const {
		if (children.size() > 1) children[1] = children[0];
	}
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	periple::Solver solver(mat);
	solver.branch_and_bound(periple::AssignmentBound{}, Duplicates{});
}
