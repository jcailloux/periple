#include <periple/io/tsplib_parser.hpp>
#include <periple/distance/functions.hpp>
#include <periple/core/solver.hpp>
#include <periple/algorithms/nearest_neighbor.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>

// ---------------------------------------------------------------------------
// Instance validation via load_tsplib (precomputed matrix, O(N^2) memory)
// ---------------------------------------------------------------------------

struct Instance {
	const char* path;
	std::size_t expected_n;
};

void validate_matrix(const Instance& inst) {
	std::string full_path = std::string(PERIPLE_INSTANCES_DIR) + inst.path;
	auto mat = periple::load_tsplib(full_path);

	if (mat.size() != inst.expected_n) {
		std::cerr << "FAIL " << inst.path
				  << ": expected " << inst.expected_n
				  << ", got " << mat.size() << "\n";
	}
	assert(mat.size() == inst.expected_n);

	std::size_t n = mat.size();
	for (std::size_t i = 0; i < n; ++i)
		assert(mat(i, i) == 0);
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = i + 1; j < n; ++j) {
			assert(mat(i, j) >= 0);
			assert(mat(i, j) == mat(j, i));
		}

	std::cout << "  OK  " << inst.path << " (" << n << ") [matrix]\n";
}

// ---------------------------------------------------------------------------
// Instance validation via load_tsplib_coords (on-the-fly, O(N) memory)
// Spot-checks a sample of distances instead of all N^2 pairs.
// ---------------------------------------------------------------------------

void validate_coords(const Instance& inst) {
	std::string full_path = std::string(PERIPLE_INSTANCES_DIR) + inst.path;
	auto cd = periple::load_tsplib_coords(full_path);

	if (cd.size() != inst.expected_n) {
		std::cerr << "FAIL " << inst.path
				  << ": expected " << inst.expected_n
				  << ", got " << cd.size() << "\n";
	}
	assert(cd.size() == inst.expected_n);

	std::size_t n = cd.size();

	// Diagonal is zero
	for (std::size_t i = 0; i < n; i += std::max<std::size_t>(1, n / 50))
		assert(cd(i, i) == 0);

	// Spot-check: non-negative and symmetric
	for (std::size_t i = 0; i < n; i += std::max<std::size_t>(1, n / 50))
		for (std::size_t j = i + 1; j < n; j += std::max<std::size_t>(1, n / 50)) {
			assert(cd(i, j) >= 0);
			assert(cd(i, j) == cd(j, i));
		}

	std::cout << "  OK  " << inst.path << " (" << n << ") [coords]\n";
}

// ---------------------------------------------------------------------------
// Cross-validation: matrix and coords produce the same distances
// ---------------------------------------------------------------------------

void cross_validate(const Instance& inst) {
	std::string full_path = std::string(PERIPLE_INSTANCES_DIR) + inst.path;
	auto mat = periple::load_tsplib(full_path);
	auto cd = periple::load_tsplib_coords(full_path);

	assert(mat.size() == cd.size());
	std::size_t n = mat.size();

	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = i + 1; j < n; ++j)
			assert(mat(i, j) == cd(i, j));

	std::cout << "  OK  " << inst.path << " (" << n << ") [cross]\n";
}

// Cross-validate EUC_2D instances using periple::euclidean<int>(2)
// instead of the parser's internal distance function.
void cross_validate_euclidean(const Instance& inst) {
	std::string full_path = std::string(PERIPLE_INSTANCES_DIR) + inst.path;
	auto mat = periple::load_tsplib(full_path);
	auto cd = periple::load_tsplib_coords(full_path);

	std::size_t n = cd.size();
	std::vector<double> coords(n * 2);
	for (std::size_t i = 0; i < n; ++i) {
		const double* p = cd.point(i);
		coords[i * 2]     = p[0];
		coords[i * 2 + 1] = p[1];
	}

	periple::CoordinateDistance pub(2, std::move(coords),
		periple::euclidean<int>(2));

	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = i + 1; j < n; ++j)
			assert(mat(i, j) == pub(i, j));

	std::cout << "  OK  " << inst.path << " (" << n << ") [euclidean]\n";
}

// ---------------------------------------------------------------------------
// Test lists
// ---------------------------------------------------------------------------

void test_instances() {
	// Tier 1-4: validated with precomputed matrix (exhaustive check)
	Instance matrix_instances[] = {
		// Tier 1 -- exact (N <= 25)
		{"/tier1/burma14.tsp",   14},
		{"/tier1/ulysses16.tsp", 16},
		{"/tier1/gr17.tsp",      17},
		{"/tier1/gr21.tsp",      21},
		{"/tier1/ulysses22.tsp", 22},
		{"/tier1/gr24.tsp",      24},

		// Tier 2 -- small heuristics (N <= 200)
		{"/tier2/att48.tsp",     48},
		{"/tier2/berlin52.tsp",  52},
		{"/tier2/st70.tsp",      70},
		{"/tier2/eil76.tsp",     76},
		{"/tier2/kroA100.tsp",  100},
		{"/tier2/kroB100.tsp",  100},
		{"/tier2/ch130.tsp",    130},
		{"/tier2/ch150.tsp",    150},
		{"/tier2/kroA200.tsp",  200},

		// Tier 3 -- medium heuristics (200 < N <= 1000)
		{"/tier3/a280.tsp",     280},
		{"/tier3/lin318.tsp",   318},
		{"/tier3/pcb442.tsp",   442},
		{"/tier3/att532.tsp",   532},
		{"/tier3/rat575.tsp",   575},
		{"/tier3/rat783.tsp",   783},

		// Tier 4 -- large (N > 1000)
		{"/tier4/pr1002.tsp",  1002},
		{"/tier4/u1817.tsp",   1817},
		{"/tier4/pr2392.tsp",  2392},
		{"/tier4/pcb3038.tsp", 3038},
		{"/tier4/fnl4461.tsp", 4461},
		{"/tier4/rl5934.tsp",  5934},
	};

	// Tier 5: only via load_tsplib_coords (matrix would need multiple GB)
	Instance coord_only_instances[] = {
		{"/tier5/usa13509.tsp", 13509},
		{"/tier5/d18512.tsp",   18512},
		{"/tier5/pla33810.tsp", 33810},
		{"/tier5/pla85900.tsp", 85900},
	};

	// Coordinate-based Tier 1-2 instances for cross-validation (matrix vs parser coords)
	Instance cross_instances[] = {
		{"/tier1/burma14.tsp",   14},  // GEO
		{"/tier1/ulysses16.tsp", 16},  // GEO
		{"/tier2/att48.tsp",     48},  // ATT
		{"/tier2/berlin52.tsp",  52},  // EUC_2D
	};

	// EUC_2D instances: cross-validate parser vs periple::euclidean<int>(2)
	Instance euc2d_instances[] = {
		{"/tier2/berlin52.tsp",  52},
		{"/tier2/st70.tsp",      70},
		{"/tier2/eil76.tsp",     76},
		{"/tier2/kroA100.tsp",  100},
	};

	std::cout << "Validating instances (matrix)...\n";
	for (const auto& inst : matrix_instances)
		validate_matrix(inst);

	std::cout << "Validating instances (coords)...\n";
	for (const auto& inst : coord_only_instances)
		validate_coords(inst);

	std::cout << "Cross-validating matrix vs coords...\n";
	for (const auto& inst : cross_instances)
		cross_validate(inst);

	std::cout << "Cross-validating EUC_2D via periple::euclidean...\n";
	for (const auto& inst : euc2d_instances)
		cross_validate_euclidean(inst);

	std::cout << "All instances validated.\n";
}

// ---------------------------------------------------------------------------
// Distance functions: euclidean, manhattan, chebyshev
// ---------------------------------------------------------------------------

void test_distance_functions() {
	double a[] = {0.0, 0.0};
	double b[] = {3.0, 4.0};

	// euclidean<int>(2) = TSPLIB EUC_2D: nint(sqrt(9+16)) = 5
	assert(periple::euclidean<int>(2)(a, b) == 5);

	// euclidean<double>(2): exact
	double ed = periple::euclidean<double>(2)(a, b);
	assert(std::abs(ed - 5.0) < 1e-10);

	// Rounding: sqrt(2) = 1.414... -> nint = 1
	double c[] = {0.0, 0.0}, d[] = {1.0, 1.0};
	assert(periple::euclidean<int>(2)(c, d) == 1);

	// manhattan<int>(2): |3| + |4| = 7
	assert(periple::manhattan<int>(2)(a, b) == 7);

	// manhattan<double>(2): exact
	assert(periple::manhattan<double>(2)(a, b) == 7.0);

	// ceiling_euclidean<int>(2): ceil(sqrt(9+16)) = 5
	assert(periple::ceiling_euclidean<int>(2)(a, b) == 5);

	// ceiling_euclidean rounding: ceil(sqrt(2)) = ceil(1.414...) = 2
	assert(periple::ceiling_euclidean<int>(2)(c, d) == 2);

	// chebyshev<int>(2): max(3, 4) = 4
	assert(periple::chebyshev<int>(2)(a, b) == 4);

	// 3D euclidean: sqrt(1+4+4) = 3
	double e[] = {0.0, 0.0, 0.0}, f[] = {1.0, 2.0, 2.0};
	assert(periple::euclidean<int>(3)(e, f) == 3);

	// 3D manhattan: 1+2+2 = 5
	assert(periple::manhattan<int>(3)(e, f) == 5);

	// 3D chebyshev: max(1,2,2) = 2
	assert(periple::chebyshev<int>(3)(e, f) == 2);

	// CoordinateDistance CTAD deduces CostT from distance function
	std::vector<double> coords = {0.0, 0.0, 3.0, 4.0};
	periple::CoordinateDistance cd(2, coords, periple::euclidean<double>(2));
	static_assert(std::is_same_v<decltype(cd)::cost_type, double>);
	assert(std::abs(cd(0, 1) - 5.0) < 1e-10);

	// Construction from vector<pair<double, double>>
	std::vector<std::pair<double, double>> pairs = {{0.0, 0.0}, {3.0, 4.0}};
	periple::CoordinateDistance from_pairs(pairs, periple::euclidean<int>(2));
	assert(from_pairs.size() == 2);
	assert(from_pairs.dim() == 2);
	assert(from_pairs(0, 1) == 5);

	// Construction from vector<array<double, 3>>
	std::vector<std::array<double, 3>> arrays = {{0.0, 0.0, 0.0}, {1.0, 2.0, 2.0}};
	periple::CoordinateDistance from_arrays(arrays, periple::euclidean<int>(3));
	assert(from_arrays.size() == 2);
	assert(from_arrays.dim() == 3);
	assert(from_arrays(0, 1) == 3);

	// Construction from vector<tuple<double, double>>
	std::vector<std::tuple<double, double>> tuples = {{0.0, 0.0}, {3.0, 4.0}};
	periple::CoordinateDistance from_tuples(tuples, periple::euclidean<int>(2));
	assert(from_tuples.size() == 2);
	assert(from_tuples(0, 1) == 5);

	// Construction from vector<vector<double>>
	std::vector<std::vector<double>> vecs = {{0.0, 0.0}, {3.0, 4.0}};
	periple::CoordinateDistance from_vecs(vecs, periple::euclidean<int>(2));
	assert(from_vecs.size() == 2);
	assert(from_vecs(0, 1) == 5);
}

// ---------------------------------------------------------------------------
// Solver integration: CoordinateDistance works with Solver via CTAD
// ---------------------------------------------------------------------------

void test_solver_integration() {
	std::string path = std::string(PERIPLE_INSTANCES_DIR) + "/tier1/burma14.tsp";

	auto mat = periple::load_tsplib(path);
	periple::Solver solver_mat(mat);
	solver_mat.nearest_neighbor();
	assert(solver_mat.cost() > 0);
	assert(solver_mat.tour().size() == 14);

	auto cd = periple::load_tsplib_coords(path);
	periple::Solver solver_cd(cd);
	solver_cd.nearest_neighbor();
	assert(solver_cd.cost() == solver_mat.cost());
}

// ---------------------------------------------------------------------------
// Error handling
// ---------------------------------------------------------------------------

void test_missing_dimension() {
	std::istringstream in(
		"EDGE_WEIGHT_TYPE : EUC_2D\n"
		"NODE_COORD_SECTION\n"
		"1 0.0 0.0\n");
	bool caught = false;
	try { periple::load_tsplib(in); }
	catch (const std::runtime_error&) { caught = true; }
	assert(caught);
}

void test_unsupported_type() {
	std::istringstream in(
		"DIMENSION : 2\n"
		"EDGE_WEIGHT_TYPE : MAN_2D\n"
		"NODE_COORD_SECTION\n"
		"1 0.0 0.0\n"
		"2 1.0 1.0\n");
	bool caught = false;
	try { periple::load_tsplib(in); }
	catch (const std::runtime_error&) { caught = true; }
	assert(caught);
}

void test_missing_format_for_explicit() {
	std::istringstream in(
		"DIMENSION : 2\n"
		"EDGE_WEIGHT_TYPE : EXPLICIT\n"
		"EDGE_WEIGHT_SECTION\n"
		"0 5 5 0\n");
	bool caught = false;
	try { periple::load_tsplib(in); }
	catch (const std::runtime_error&) { caught = true; }
	assert(caught);
}

void test_no_data_section() {
	std::istringstream in(
		"DIMENSION : 2\n"
		"EDGE_WEIGHT_TYPE : EUC_2D\n"
		"EOF\n");
	bool caught = false;
	try { periple::load_tsplib(in); }
	catch (const std::runtime_error&) { caught = true; }
	assert(caught);
}

void test_coords_rejects_explicit() {
	std::istringstream in(
		"DIMENSION : 3\n"
		"EDGE_WEIGHT_TYPE : EXPLICIT\n"
		"EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
		"EDGE_WEIGHT_SECTION\n"
		"0 10 15\n"
		"10 0 20\n"
		"15 20 0\n");
	bool caught = false;
	try { periple::load_tsplib_coords(in); }
	catch (const std::runtime_error&) { caught = true; }
	assert(caught);
}

int main() {
	test_distance_functions();
	test_instances();
	test_solver_integration();
	test_missing_dimension();
	test_unsupported_type();
	test_missing_format_for_explicit();
	test_no_data_section();
	test_coords_rejects_explicit();
}