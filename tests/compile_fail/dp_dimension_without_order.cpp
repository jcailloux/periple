// ReachableLabels keeps a state only while no other dominates it, which a
// dimension that keeps DPMove state must say.
#include <periple/periple.hpp>

#include <vector>

struct UnorderedDpTable {
	std::vector<double> table;
	double tentative = 0;
	void resize(std::size_t) {}
	void reset() {}

	template <periple::DistanceSource Dist, typename CityT>
	void init(const periple::AppendMove<CityT>&, const Dist&) {}

	void resize_dp(std::size_t states) { table.resize(states); }
	void begin_dp(std::size_t state) { table[state] = 0; }

	template <periple::DistanceSource Dist, typename CityT>
	void init(const periple::DPMove<CityT>& m, const Dist&) { tentative = table[m.from_state] + 1; }

	template <typename CityT>
	void commit(const periple::DPMove<CityT>& m) { table[m.to_state] = tentative; }
};

struct Tracked {
	using dimension = UnorderedDpTable;
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	Tracked variant;
	periple::Solver solver(mat, variant);
	solver.held_karp();
}
