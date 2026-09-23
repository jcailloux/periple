#include <periple/algorithms/registry.hpp>
#include <periple/distance/matrix.hpp>
#include <periple/distance/jonker_volgenant.hpp>
#include <solver_test_access.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <limits>
#include <numeric>
#include <vector>

// ---------------------------------------------------------------------------
// Matrix factories
// ---------------------------------------------------------------------------

auto sym(std::size_t n) {
	return periple::SymmetricDistanceMatrix<int>(n);
}

auto sym(std::vector<std::vector<int>> rows) {
	return periple::SymmetricDistanceMatrix<int>(std::move(rows));
}

auto asym(std::size_t n) {
	return periple::DistanceMatrix<int>(n);
}

auto asym(std::size_t n, std::initializer_list<int> values) {
	return periple::DistanceMatrix<int>(n, values);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void assert_valid_tour(auto tour, std::size_t n) {
	assert(tour.size() == n);
	std::vector<bool> seen(n, false);
	for (auto c : tour) {
		auto i = static_cast<std::size_t>(c);
		assert(i < n && !seen[i]);
		seen[i] = true;
	}
}

template <periple::DistanceSource Dist>
auto recompute_cost(const Dist& dist, auto tour) {
	using cost_type = typename periple::dist_traits<Dist>::cost_type;
	cost_type total{};
	const auto n = tour.size();
	for (std::size_t i = 0; i < n; ++i)
		total += dist(tour[i], tour[(i + 1) % n]);
	return total;
}

template <periple::DistanceSource Dist>
auto brute_force_optimal(const Dist& dist) {
	using traits    = periple::dist_traits<Dist>;
	using cost_type = typename traits::cost_type;
	auto city = [](std::size_t i) {
		return static_cast<typename traits::city_type>(i);
	};

	const auto n = dist.size();
	if (n <= 1) return cost_type{};

	std::vector<std::size_t> perm(n - 1);
	std::iota(perm.begin(), perm.end(), std::size_t{1});
	cost_type best = std::numeric_limits<cost_type>::max();
	do {
		cost_type c = dist(city(0), city(perm[0]));
		for (std::size_t i = 0; i + 1 < perm.size(); ++i)
			c += dist(city(perm[i]), city(perm[i + 1]));
		c += dist(city(perm.back()), city(0));
		best = std::min(best, c);
	} while (std::next_permutation(perm.begin(), perm.end()));
	return best;
}

// ---------------------------------------------------------------------------
// Core test: one algorithm x one distance source
// ---------------------------------------------------------------------------

template <typename Algo, periple::DistanceSource Dist>
void test_one(const Algo& algo, const Dist& dist) {
	const auto n = dist.size();
	periple::Solver solver(dist);
	periple::run_checked(solver, [&] { algo(solver); });

	if (n == 0) {
		assert(solver.tour().empty());
		assert(solver.cost() == 0);
		return;
	}

	assert_valid_tour(solver.tour(), n);
	assert(solver.cost() == recompute_cost(dist, solver.tour()));

	if constexpr (Algo::is_exact) {
		assert(solver.status() == periple::SolutionStatus::optimal);
		assert(solver.cost() == brute_force_optimal(dist));
	} else {
		assert(solver.status() == periple::SolutionStatus::feasible);
		assert(solver.cost() >= brute_force_optimal(dist));
	}
}

// Asymmetric test via JonkerVolgenant (for symmetric-only algorithms).
template <typename Algo>
void test_one_jv(const Algo& algo, const periple::DistanceMatrix<int>& dist) {
	const auto n = dist.size();
	if (n < 2) return;

	auto jv = periple::jonker_volgenant(dist);
	periple::Solver solver(jv);
	periple::run_checked(solver, [&] { algo(solver); });

	auto tour = jv.atsp_tour(solver.tour());
	auto cost = jv.atsp_cost(solver.cost());

	assert(tour.size() == n);
	assert_valid_tour(tour, n);
	assert(cost == recompute_cost(dist, tour));

	if constexpr (Algo::is_exact)
		assert(cost == brute_force_optimal(dist));
	else
		assert(cost >= brute_force_optimal(dist));
}

// Cache invalidation: reuse a solver across two different matrices.
// Catches stale cache bugs when invalidate_caches() is incomplete.
template <typename Algo>
void test_cache_invalidation(const Algo& algo) {
	auto m1 = sym({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	});
	auto m2 = sym({
		{ 0,  3,  4,  2,  7},
		{ 3,  0,  4,  6,  3},
		{ 4,  4,  0,  5,  8},
		{ 2,  6,  5,  0,  6},
		{ 7,  3,  8,  6,  0}
	});

	periple::Solver solver(m1);
	periple::run_checked(solver, [&] { algo(solver); });
	assert(solver.cost() == recompute_cost(m1, solver.tour()));

	// Switch to a different-sized matrix and solve again
	solver.set_matrix(m2);
	periple::run_checked(solver, [&] { algo(solver); });

	assert_valid_tour(solver.tour(), m2.size());
	assert(solver.cost() == recompute_cost(m2, solver.tour()));

	if constexpr (Algo::is_exact)
		assert(solver.cost() == brute_force_optimal(m2));
}

// ---------------------------------------------------------------------------
// Test batteries
// ---------------------------------------------------------------------------

template <typename Algo>
void run_symmetric(const Algo& algo) {
	test_one(algo, sym(0));
	test_one(algo, sym(1));

	test_one(algo, sym({
		{0,  7},
		{7,  0}
	}));

	test_one(algo, sym({
		{ 0, 10, 15},
		{10,  0, 20},
		{15, 20,  0}
	}));

	test_one(algo, sym({
		{ 0, 10, 15, 20},
		{10,  0, 35, 25},
		{15, 35,  0, 30},
		{20, 25, 30,  0}
	}));

	test_one(algo, sym({
		{ 0,  3,  4,  2,  7},
		{ 3,  0,  4,  6,  3},
		{ 4,  4,  0,  5,  8},
		{ 2,  6,  5,  0,  6},
		{ 7,  3,  8,  6,  0}
	}));
}

template <typename Algo>
void run_asymmetric(const Algo& algo) {
	test_one(algo, asym(0));
	test_one(algo, asym(1, {0}));

	auto a2 = asym(2, {0, 5, 3, 0});
	auto a3 = asym(3, {
		 0,  5,  8,
		12,  0, 10,
		 6, 14,  0
	});
	auto a4 = asym(4, {
		 0,  5,  8, 12,
		15,  0, 10,  7,
		 6, 14,  0,  9,
		11,  3, 13,  0
	});
	auto a5 = asym(5, {
		 0,  3,  7,  2, 10,
		 8,  0,  4,  6,  3,
		 5,  9,  0, 11,  8,
		 2,  6,  5,  0,  7,
		12,  4,  8,  6,  0
	});

	if constexpr (Algo::symmetric_only) {
		test_one_jv(algo, a2);
		test_one_jv(algo, a3);
		test_one_jv(algo, a4);
		test_one_jv(algo, a5);
	} else {
		test_one(algo, a2);
		test_one(algo, a3);
		test_one(algo, a4);
		test_one(algo, a5);
	}
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	if (argc > 1 && std::strcmp(argv[1], "--help") == 0) {
		std::printf("Usage: test_algorithms [TAGS]\n"
		            "  TAGS: comma-separated algorithm tags (default: all)\n");
		periple::print_algorithm_tags(stdout);
		return 0;
	}

	const char* filter = (argc > 1) ? argv[1] : nullptr;

	int count = periple::for_each_algorithm(filter, [](const auto& algo) {
		std::printf("%s ... ", algo.name);
		std::fflush(stdout);
		run_symmetric(algo);
		run_asymmetric(algo);
		test_cache_invalidation(algo);
		std::printf("OK\n");
	});

	if (count == 0) {
		std::fprintf(stderr, "No algorithms matched filter '%s'\n", filter);
		periple::print_algorithm_tags();
		return 1;
	}
}
