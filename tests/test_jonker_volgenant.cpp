#include <periple/core/jonker_volgenant.hpp>
#include <periple/distance/matrix.hpp>
#include <periple/core/solver.hpp>
#include <periple/algorithms/nearest_neighbor.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

// Recompute the ATSP tour cost from a city sequence.
template <typename Dist, typename CityT>
auto tour_cost(const Dist& dist, const std::vector<CityT>& tour) {
	typename Dist::cost_type total{};
	for (std::size_t i = 0; i < tour.size(); ++i)
		total += dist(tour[i], tour[(i + 1) % tour.size()]);
	return total;
}

void test_concept_and_traits() {
	periple::DistanceMatrix<int> m(3, {
		0, 1, 2,
		3, 0, 4,
		5, 6, 0
	});
	auto jv = periple::jonker_volgenant(m);

	static_assert(periple::DistanceSource<decltype(jv)>);
}

void test_distance_values() {
	periple::DistanceMatrix<int> m(3, {
		 0,  1, 10,
		10,  0,  1,
		 1, 10,  0
	});
	auto jv = periple::jonker_volgenant(m);
	assert(jv.size() == 6);
	assert(jv.original_size() == 3);
	assert(jv.big_m() == 31);

	constexpr auto INF = std::numeric_limits<int>::max();

	// Diagonal
	for (std::size_t i = 0; i < 6; ++i)
		assert(jv(i, i) == 0);

	// Same-class edges: infinity
	assert(jv(0, 1) == INF);
	assert(jv(0, 2) == INF);
	assert(jv(1, 2) == INF);
	assert(jv(3, 4) == INF);
	assert(jv(3, 5) == INF);
	assert(jv(4, 5) == INF);

	// Self-pairing: -M
	assert(jv(0, 3) == -31);
	assert(jv(1, 4) == -31);
	assert(jv(2, 5) == -31);

	// Symmetry of self-pairing
	assert(jv(3, 0) == -31);
	assert(jv(4, 1) == -31);
	assert(jv(5, 2) == -31);

	// Cross-pairing: D[real][n+ghost_orig] = C[ghost_orig][real]
	assert(jv(0, 4) == 10);  // C[1][0]
	assert(jv(0, 5) == 1);   // C[2][0]
	assert(jv(1, 3) == 1);   // C[0][1]
	assert(jv(1, 5) == 10);  // C[2][1]
	assert(jv(2, 3) == 10);  // C[0][2]
	assert(jv(2, 4) == 1);   // C[1][2]

	// Symmetry of cross-pairing
	assert(jv(4, 0) == 10);
	assert(jv(5, 0) == 1);
	assert(jv(3, 1) == 1);
}

void test_solve_and_extract_3city() {
	periple::DistanceMatrix<int> m(3, {
		 0,  1, 10,
		10,  0,  1,
		 1, 10,  0
	});
	auto jv = periple::jonker_volgenant(m);

	periple::Solver solver(jv);
	solver.nearest_neighbor();

	auto atsp = jv.atsp_tour(solver.tour());

	// Valid permutation
	std::vector<bool> seen(3, false);
	for (auto c : atsp) {
		assert(c < 3);
		seen[c] = true;
	}
	for (auto s : seen) assert(s);

	// Cost consistency
	int cost = jv.atsp_cost(solver.cost());
	assert(cost == tour_cost(m, atsp));

	// NN from city 0 finds the optimal tour 0->1->2->0 (cost 3)
	assert(cost == 3);
}

void test_solve_and_extract_4city() {
	periple::DistanceMatrix<int> m(4, {
		 0,  2,  9, 10,
		 1,  0,  6,  4,
		15,  7,  0,  8,
		 6,  3, 12,  0
	});
	auto jv = periple::jonker_volgenant(m);

	periple::Solver solver(jv);
	solver.nearest_neighbor();

	auto atsp = jv.atsp_tour(solver.tour());

	// Valid permutation
	std::vector<bool> seen(4, false);
	for (auto c : atsp) {
		assert(c < 4);
		seen[c] = true;
	}
	for (auto s : seen) assert(s);

	// Cost consistency
	int cost = jv.atsp_cost(solver.cost());
	assert(cost == tour_cost(m, atsp));
}

void test_user_provided_big_m() {
	periple::DistanceMatrix<int> m(3, {
		0, 1, 2,
		3, 0, 4,
		5, 6, 0
	});
	auto jv = periple::jonker_volgenant(m, 100);
	assert(jv.big_m() == 100);
	assert(jv(0, 3) == -100);
}

void test_double_costs() {
	periple::DistanceMatrix<double> m(3, {
		0.0,  1.5, 10.0,
		10.0, 0.0,  1.5,
		1.5, 10.0,  0.0
	});
	auto jv = periple::jonker_volgenant(m);

	static_assert(periple::DistanceSource<decltype(jv)>);

	periple::Solver solver(jv);
	solver.nearest_neighbor();

	auto atsp = jv.atsp_tour(solver.tour());

	double cost = jv.atsp_cost(solver.cost());
	double expected = tour_cost(m, atsp);
	assert(std::abs(cost - expected) < 1e-9);
}

void test_2city() {
	periple::DistanceMatrix<int> m(2, {
		0, 3,
		7, 0
	});
	auto jv = periple::jonker_volgenant(m);
	assert(jv.size() == 4);
	assert(jv.original_size() == 2);

	periple::Solver solver(jv);
	solver.nearest_neighbor();

	auto atsp = jv.atsp_tour(solver.tour());

	int cost = jv.atsp_cost(solver.cost());
	assert(cost == 3 + 7); // only possible tour: 0->1->0
}

void test_build_symmetric_tour() {
	periple::DistanceMatrix<int> m(3, {
		 0,  1, 10,
		10,  0,  1,
		 1, 10,  0
	});
	auto jv = periple::jonker_volgenant(m);

	std::vector<std::size_t> atsp_in = {0, 1, 2};
	auto sym = jv.symmetric_tour(std::span<const std::size_t>(atsp_in));

	// Size 2n
	assert(sym.size() == 6);

	// Alternates real-ghost: [c0, c0+n, c1, c1+n, c2, c2+n]
	for (std::size_t i = 0; i < 3; ++i) {
		assert(sym[2 * i] == atsp_in[i]);
		assert(sym[2 * i + 1] == atsp_in[i] + 3);
	}
}

void test_round_trip() {
	periple::DistanceMatrix<int> m(4, {
		 0,  2,  9, 10,
		 1,  0,  6,  4,
		15,  7,  0,  8,
		 6,  3, 12,  0
	});
	auto jv = periple::jonker_volgenant(m);

	std::vector<std::size_t> original = {2, 0, 3, 1};

	// ATSP -> symmetric -> ATSP = identity
	auto sym = jv.symmetric_tour(std::span<const std::size_t>(original));
	auto recovered = jv.atsp_tour(std::span<const std::size_t>(sym));

	assert(recovered.size() == original.size());
	for (std::size_t i = 0; i < original.size(); ++i)
		assert(recovered[i] == original[i]);
}

void test_vector_overloads() {
	periple::DistanceMatrix<int> m(3, {
		 0,  1, 10,
		10,  0,  1,
		 1, 10,  0
	});
	auto jv = periple::jonker_volgenant(m);

	// symmetric_tour: vector vs span overloads produce same result
	std::vector<std::size_t> atsp_in = {0, 2, 1};
	auto sym_vec = jv.symmetric_tour(std::span<const std::size_t>(atsp_in));
	assert(sym_vec.size() == 6);

	std::vector<std::size_t> sym_span(6);
	jv.symmetric_tour(std::span<const std::size_t>(atsp_in), sym_span);
	for (std::size_t i = 0; i < 6; ++i)
		assert(sym_vec[i] == sym_span[i]);

	// atsp_tour: vector vs span overloads produce same result
	auto atsp_vec = jv.atsp_tour(std::span<const std::size_t>(sym_vec));
	assert(atsp_vec.size() == 3);

	std::vector<std::size_t> atsp_span(3);
	jv.atsp_tour(std::span<const std::size_t>(sym_vec), atsp_span);
	for (std::size_t i = 0; i < 3; ++i)
		assert(atsp_vec[i] == atsp_span[i]);

	// Round-trip: results match original input
	for (std::size_t i = 0; i < 3; ++i)
		assert(atsp_vec[i] == atsp_in[i]);
}

int main() {
	test_concept_and_traits();
	test_distance_values();
	test_solve_and_extract_3city();
	test_solve_and_extract_4city();
	test_user_provided_big_m();
	test_double_costs();
	test_2city();
	test_build_symmetric_tour();
	test_round_trip();
	test_vector_overloads();
}
