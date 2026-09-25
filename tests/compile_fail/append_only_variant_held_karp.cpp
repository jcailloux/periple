// A variant without DPMove callbacks must not run under held_karp.
#include <periple/periple.hpp>

struct RejectCity1 {
	bool move_filter(const periple::AppendMove<std::size_t>& m) const { return m.city != 1; }
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	RejectCity1 variant;
	periple::Solver solver(mat, variant);
	solver.held_karp();
}
