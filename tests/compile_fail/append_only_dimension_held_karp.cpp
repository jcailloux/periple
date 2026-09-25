// A dimension without DPMove init or commit must not run under held_karp.
#include <periple/periple.hpp>

struct Load {
	double value = 0;
	void resize(std::size_t) {}
	void reset() {}

	template <periple::DistanceSource Dist, typename CityT>
	void init(const periple::AppendMove<CityT>&, const Dist&) {}
};

struct Capacity {
	using dimension = Load;
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	Capacity variant;
	periple::Solver solver(mat, variant);
	solver.held_karp();
}
