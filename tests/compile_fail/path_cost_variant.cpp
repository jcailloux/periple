// path_cost sums raw distances, so it rejects a variant: prefix_cost is its counterpart.
#include <periple/periple.hpp>

struct Penalty {
	template <typename CityT, typename Ctx>
	void move_prepare(const periple::AppendMove<CityT>&, Ctx& ctx) const { ctx.cost_delta += 1.0; }
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	Penalty penalty;
	periple::Solver solver(mat, penalty);
	solver.nearest_neighbor();
	(void)solver.path_cost(0, 2);
}
