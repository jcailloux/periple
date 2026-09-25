// A variant with callbacks must handle AppendMove, which costs every tour.
#include <periple/periple.hpp>

struct DpOnly {
	template <typename CityT>
	bool move_filter(const periple::DPMove<CityT>& m) const { return m.to != 1; }
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	DpOnly variant;
	periple::Solver solver(mat, variant);
	solver.held_karp();
}
