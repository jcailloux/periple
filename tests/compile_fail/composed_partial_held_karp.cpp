// One component without DPMove callbacks is enough to reject a Composed.
#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>

struct RejectCity1 {
	bool move_filter(const periple::AppendMove<std::size_t>& m) const { return m.city != 1; }
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({{0, 1, 2}, {1, 0, 1}, {2, 1, 0}});
	periple::time_windows::TimeWindow windows[] = {{0, 100}, {0, 100}, {0, 100}};
	periple::time_windows::Strict tw(windows);
	RejectCity1 reject;
	auto variant = periple::Composed(tw, reject);
	periple::Solver solver(mat, variant);
	solver.held_karp();
}
