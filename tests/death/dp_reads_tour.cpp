// A variant that reads ctx.tour() must fail loudly under held_karp, which evaluates states, not a tour.
#include <periple/periple.hpp>

struct ReadsTourInDp {
	template <typename CityT>
	bool move_filter(const periple::AppendMove<CityT>&) const { return true; }

	template <typename CityT, typename Ctx>
	bool move_filter(const periple::DPMove<CityT>&, const Ctx& ctx) const {
		return ctx.tour().size() < 1000;
	}
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	ReadsTourInDp variant;
	periple::Solver solver(mat, variant);
	solver.held_karp();
}
