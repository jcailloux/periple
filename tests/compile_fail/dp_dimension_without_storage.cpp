// A dimension that commits DPMove state must take its storage from the algorithm.
#include <periple/periple.hpp>

#include <vector>

struct LazyDpTable {
	std::vector<double> table;
	void resize(std::size_t) {}
	void reset() {}

	template <periple::DistanceSource Dist, typename CityT>
	void init(const periple::AppendMove<CityT>&, const Dist&) {}

	template <periple::DistanceSource Dist, typename CityT>
	void init(const periple::DPMove<CityT>&, const Dist&) {}

	template <typename CityT>
	void commit(const periple::DPMove<CityT>& m) { table.push_back(static_cast<double>(m.set)); }
};

struct Tracked {
	using dimension = LazyDpTable;
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	Tracked variant;
	periple::Solver solver(mat, variant);
	solver.held_karp();
}
