// A dimension that commits along the tour without staging must not run under a replay.
#include <periple/periple.hpp>

#include <vector>

struct CityAt {
	std::vector<std::size_t> at;
	std::size_t pos_ = 0, city_ = 0;
	void resize(std::size_t n) { at.resize(n); }
	void reset() {}

	template <periple::DistanceSource Dist, typename CityT>
	void init(const periple::AppendMove<CityT>& m, const Dist&) {
		pos_ = m.pos;
		city_ = static_cast<std::size_t>(m.city);
	}

	template <typename CityT>
	void commit(const periple::AppendMove<CityT>&) { at[pos_] = city_; }
};

struct Tracked {
	using dimension = CityAt;
};

int main() {
	periple::SymmetricDistanceMatrix<int> mat({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	Tracked variant;
	periple::Solver solver(mat, variant);
	solver.nearest_neighbor();
	solver.two_opt();
}
