#include <periple/periple.hpp>
#include <periple/variants/time_windows.hpp>
#include <solver_test_access.hpp>

#include <cassert>
#include <cstddef>
#include <optional>
#include <vector>

// Appends nothing, so greedy_construct(resume_at) only truncates the tour.
struct StopImmediately {
	template <periple::DistanceSource Dist, typename Variant>
	auto select_next(const periple::Solver<Dist, Variant>&) const
		-> std::optional<typename periple::dist_traits<Dist>::city_type> {
		return std::nullopt;
	}
};

// 5 cities: the tour 0 1 2 3 4 costs 5, every other tour more; 0 4 1 2 3 costs 6.
periple::DistanceMatrix<int> five_cities() {
	const std::size_t n = 5;
	std::vector<int> w(n * n, 10);
	for (std::size_t i = 0; i < n; ++i) {
		w[i * n + i] = 0;
		w[i * n + (i + 1) % n] = 1;
	}
	w[0 * n + 4] = 1;
	w[4 * n + 1] = 1;
	w[3 * n + 0] = 2;
	return periple::DistanceMatrix<int>(n, w);
}

int main() {
	periple::DistanceMatrix<int> mat(4, {
		 0, 10, 15, 20,
		10,  0, 35, 25,
		15, 35,  0, 30,
		20, 25, 30,  0
	});

	// --- Default construction ---
	periple::Solver<periple::DistanceMatrix<int>> solver;
	assert(solver.status() == periple::SolutionStatus::none);
	assert(solver.tour().empty());
	assert(solver.cost() == 0);

	// --- set_matrix + chaining ---
	solver.set_matrix(mat);
	solver.nearest_neighbor().held_karp();
	assert(solver.cost() == 80);

	// --- set_tour ---
	std::vector<std::size_t> manual = {0, 2, 3, 1};
	solver.set_tour(manual);
	assert(solver.tour().size() == 4);
	assert(solver.tour()[0] == 0);
	assert(solver.tour()[1] == 2);

	// --- is_visited after set_tour (partial, complete, known cost) ---
	std::vector<std::size_t> partial = {0, 2};
	solver.set_tour(partial);
	assert(solver.is_visited(2) && !solver.is_visited(1) && "set_tour (partial): only the prefix is visited");
	solver.set_tour(manual);
	for (std::size_t c = 0; c < 4; ++c)
		assert(solver.is_visited(c) && "set_tour (complete): every city is visited");
	solver.set_tour(partial);
	solver.set_tour(manual, 80);
	for (std::size_t c = 0; c < 4; ++c)
		assert(solver.is_visited(c) && "set_tour (known cost): every city is visited");

	// --- clear ---
	solver.clear();
	assert(solver.status() == periple::SolutionStatus::none);
	assert(solver.tour().empty());
	assert(solver.cost() == 0);

	// --- reset ---
	solver.set_matrix(mat);
	solver.nearest_neighbor();
	assert(solver.cost() > 0);
	solver.reset();
	assert(solver.status() == periple::SolutionStatus::none);
	assert(solver.tour().empty());
	assert(solver.cost() == 0);
	assert(solver.size() == 0);

	// --- Reuse solver with different instance ---
	solver.set_matrix(mat);
	periple::DistanceMatrix<int> mat2(3, {0, 1, 2, 1, 0, 3, 2, 3, 0});
	solver.set_matrix(mat2);
	solver.nearest_neighbor();
	assert(solver.tour().size() == 3);

	// --- start_city parameter ---
	solver.set_matrix(mat);
	solver.nearest_neighbor({.start_city = 2});
	assert(solver.tour()[0] == 2);
	solver.nearest_neighbor({.start_city = 0});
	assert(solver.tour()[0] == 0);

	// --- tour_version_ contract ---
	// run_checked asserts that a call changing the tour changes tour_version_.
	// Without the bump a tour-derived cache stays marked valid for a tour it no
	// longer describes, which no observable behaviour would reveal.
	solver.set_matrix(mat);
	periple::run_checked(solver, [&] { solver.append(0); });
	periple::run_checked(solver, [&] { solver.append(2); });
	periple::run_checked(solver, [&] { solver.set_tour(manual); });
	periple::run_checked(solver, [&] { solver.set_tour(partial, 25); });
	periple::run_checked(solver, [&] { solver.clear(); });
	periple::run_checked(solver, [&] { solver.nearest_neighbor(); });
	// resume_at truncates the tour without appending anything.
	periple::run_checked(solver, [&] {
		solver.greedy_construct(StopImmediately{}, {.resume_at = 2});
	});
	assert(solver.tour().size() == 2 && "resume_at must truncate the tour");
	periple::run_checked(solver, [&] { solver.set_matrix(mat); });

	// resume_at over a whole tour judges it as set_tour does.
	solver.set_tour(manual);
	solver.nearest_neighbor({.resume_at = 4});
	assert(solver.status() == periple::SolutionStatus::feasible && solver.cost() == 80
		&& "resume_at over a whole tour keeps it, closed");

	// --- rotate_to_front ---
	// A rotation reads the same directed cycle from another starting point, so
	// the cost is preserved whether or not the matrix is symmetric.
	solver.set_matrix(mat);
	solver.nearest_neighbor();
	const auto tour_cost = solver.cost();
	periple::run_checked(solver, [&] { solver.rotate_to_front(2); });
	assert(solver.tour()[0] == 2 && "rotate_to_front must put the city first");
	assert(solver.cost() == tour_cost && "rotating a cycle must not change its cost");
	periple::run_checked(solver, [&] { solver.rotate_to_front(2); });
	assert(solver.tour()[0] == 2 && "rotate_to_front must be idempotent");
	assert(solver.cost() == tour_cost && "an idempotent rotation must not change the cost");

	// --- path_cost ---
	// Sub-path costs on an asymmetric matrix, both ways, and invalidation: a new
	// tour must never be measured with the previous tour's prefixes.
	periple::DistanceMatrix<int> asym4(4, {
		 0,  1,  2,  3,
		10,  0,  4,  5,
		20, 40,  0,  6,
		30, 50, 60,  0
	});
	solver.set_matrix(asym4);
	std::vector<std::size_t> path = {0, 1, 2, 3};
	solver.set_tour(path);
	assert(solver.path_cost(2, 2) == 0 && "path_cost: an empty sub-path costs nothing");
	assert(solver.path_cost(0, 3) == 1 + 4 + 6 && "path_cost: forward over the whole path");
	assert(solver.path_cost(1, 3) == 4 + 6 && "path_cost: forward from an inner position");
	assert(solver.path_cost(3, 0) == 60 + 40 + 10 && "path_cost: backward over the whole path");
	assert(solver.path_cost(2, 1) == 40 && "path_cost: backward over one edge");

	std::vector<std::size_t> other = {0, 2, 1, 3};
	solver.set_tour(other);
	assert(solver.path_cost(0, 2) == 2 + 40 && "path_cost: a new tour invalidates the prefixes");

	// --- symmetric: declared by the source, overridden by set_symmetric ---
	solver.set_matrix(mat);
	assert(!solver.symmetric() && "an untagged matrix is asymmetric, even with symmetric data");

	periple::DistanceMatrix<int> tagged(3, {0, 1, 2, 1, 0, 3, 2, 3, 0}, periple::symmetric);
	solver.set_matrix(tagged);
	assert(solver.symmetric() && "the solver takes the source's declaration");

	solver.set_symmetric(false);
	assert(!solver.symmetric() && "set_symmetric(false) overrides a symmetric source");
	solver.set_matrix(tagged);
	assert(solver.symmetric() && "set_matrix replaces the override with the source's declaration");

	solver.set_matrix(mat);
	solver.set_symmetric(true);  // symmetric data: the debug check passes
	assert(solver.symmetric() && "set_symmetric(true) overrides an asymmetric source");
	solver.set_matrix(mat2);
	assert(!solver.symmetric() && "set_matrix replaces the override with the source's declaration");

	// unchecked bypasses the debug check.
	periple::DistanceMatrix<int> asym(3, {0, 1, 2, 3, 0, 4, 5, 6, 0});
	solver.set_matrix(asym);
	solver.set_symmetric(true, periple::unchecked);
	assert(solver.symmetric());

	// reset clears symmetric flag.
	solver.reset();
	assert(!solver.symmetric());

	// --- Shipped sources declare their symmetry ---
	periple::SymmetricDistanceMatrix<int> lower(std::vector<std::vector<int>>{{0, 4}, {4, 0}});
	assert(periple::Solver(lower).symmetric() && "a SymmetricDistanceMatrix is symmetric");
	assert(periple::Solver(periple::jonker_volgenant(asym)).symmetric() && "the Jonker-Volgenant view is symmetric");

	const std::vector<std::pair<double, double>> points = {{0, 0}, {3, 4}, {6, 0}};
	assert(periple::Solver(periple::CoordinateDistance(points, periple::euclidean<int>(2))).symmetric()
		&& "a shipped metric declares the coordinate source symmetric");
	assert(periple::Solver(periple::CoordinateDistance(points, periple::ceiling_euclidean<int>(2))).symmetric()
		&& "a shipped metric declares the coordinate source symmetric");
	assert(periple::Solver(periple::CoordinateDistance(points, periple::manhattan<int>(2))).symmetric()
		&& "a shipped metric declares the coordinate source symmetric");
	assert(periple::Solver(periple::CoordinateDistance(points, periple::chebyshev<int>(2))).symmetric()
		&& "a shipped metric declares the coordinate source symmetric");
	auto sum_of_x = [](const double* a, const double* b) { return static_cast<int>(a[0] + b[0]); };
	periple::CoordinateDistance custom(points, sum_of_x);
	assert(!periple::Solver(custom).symmetric() && "a custom function leaves the coordinate source asymmetric");
	static_assert(!periple::detail::static_symmetry<decltype(custom)>,
		"a custom function declares nothing");
	static_assert(periple::detail::symmetric_by_type<periple::CoordinateDistance<periple::Euclidean<int>, int>>,
		"a shipped metric makes the coordinate source symmetric by its type");

	// --- set_tour judges the tour under the variant ---
	// A held tour is infeasible once a move_filter rejects one of its moves, the
	// closing edge included, and costs its edges.
	using periple::SolutionStatus;
	using periple::time_windows::TimeWindow;
	const auto five = five_cities();
	const std::vector<std::size_t> in_order = {0, 1, 2, 3, 4}, four_first = {0, 4, 1, 2, 3};
	const std::vector<TimeWindow> loose(5, {0.0, 1e9});
	auto late = loose;
	late[4] = {0.0, 1.0};   // in_order reaches city 4 at 4, four_first at 1
	auto depot = loose;
	depot[0] = {0.0, 5.0};  // in_order returns at 5, four_first at 6
	{
		periple::time_windows::Strict strict_late(late), strict_depot(depot);
		periple::Solver judged(five, strict_late);
		judged.set_tour(four_first);
		assert(judged.status() == SolutionStatus::feasible && judged.cost() == 6
			&& "set_tour: a tour every filter admits is feasible");
		judged.set_tour(in_order);
		assert(judged.status() == SolutionStatus::infeasible && judged.cost() == 5
			&& "set_tour: a tour that reaches city 4 late is infeasible, at the cost of its edges");
		judged.set_tour(std::vector<std::size_t>{0, 4});
		assert(judged.status() == SolutionStatus::partial && "set_tour: an admitted prefix is partial");
		judged.set_tour(std::vector<std::size_t>{0, 1, 4});
		assert(judged.status() == SolutionStatus::infeasible && judged.cost() == 11
			&& "set_tour: a prefix that reaches city 4 late is infeasible");
		judged.set_tour(in_order, 42);
		assert(judged.status() == SolutionStatus::infeasible && judged.cost() == 42
			&& "set_tour (known cost): judged the same, at the cost given");

		periple::Solver closing(five, strict_depot);
		closing.set_tour(four_first);
		assert(closing.status() == SolutionStatus::infeasible && closing.cost() == 6
			&& "set_tour: a tour that returns to the depot late is infeasible");
		closing.set_tour(in_order);
		assert(closing.status() == SolutionStatus::feasible && closing.cost() == 5
			&& "set_tour: a tour that returns in time is feasible");
	}

	// --- set_variant clears the solution ---
	// The tour held was costed and judged under the previous variant: to start
	// from it, copy it and pass it to set_tour, which judges it under the new one.
	{
		periple::time_windows::Relaxed relaxed_loose(loose, 10), relaxed_late(late, 10);
		periple::Solver relaxed(five, relaxed_loose);
		relaxed.held_karp();
		assert(relaxed.status() == SolutionStatus::optimal && relaxed.cost() == 5
			&& "every window is loose, so the cheapest tour is optimal");
		const std::vector<std::size_t> held(relaxed.tour().begin(), relaxed.tour().end());
		periple::run_checked(relaxed, [&] { relaxed.set_variant(relaxed_late); });
		assert(relaxed.status() == SolutionStatus::none && relaxed.tour().empty() && relaxed.cost() == 0
			&& "set_variant clears the solution");
		relaxed.set_tour(held);
		assert(relaxed.status() == SolutionStatus::feasible && relaxed.cost() == 5 + 10 * 3
			&& "passed back, the tour is costed under the new windows: 3 late at city 4");

		periple::time_windows::Strict strict_loose(loose), strict_late(late);
		periple::Solver strict(five, strict_loose);
		strict.held_karp();
		const std::vector<std::size_t> kept(strict.tour().begin(), strict.tour().end());
		strict.set_variant(strict_late);
		assert(strict.status() == SolutionStatus::none && strict.tour().empty()
			&& "set_variant clears the solution");
		strict.set_tour(kept);
		assert(strict.status() == SolutionStatus::infeasible
			&& "passed back, the tour is judged under the new windows");
	}
}
