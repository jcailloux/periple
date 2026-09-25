// A variant that reads ctx.tour() must fail loudly under a replay, where no partial tour exists.
#include <periple/periple.hpp>

#include <vector>

struct ReadsTour {
	template <typename CityT, typename Ctx>
	bool move_filter(const periple::AppendMove<CityT>&, const Ctx& ctx) const {
		return ctx.tour().size() < 1000;
	}

	template <typename CityT>
	bool move_filter(const periple::DPMove<CityT>&) const { return true; }
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	ReadsTour variant;
	periple::Solver solver(mat, variant);
	std::vector<std::size_t> bad = {0, 2, 1, 3};
	solver.set_tour(bad);
	solver.two_opt();
}
