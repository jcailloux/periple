// The default bounds read raw distances: a variant that lowers a move's cost
// must fail loudly instead of making them prune an optimal branch.
#include <periple/periple.hpp>

struct Discount {
	template <typename CityT, typename Ctx>
	void move_prepare(const periple::AppendMove<CityT>& m, Ctx& ctx) const {
		if (m.city == 2) ctx.cost_delta -= 5;
	}
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	Discount variant;
	periple::Solver solver(mat, variant);
	solver.branch_and_bound();
}
