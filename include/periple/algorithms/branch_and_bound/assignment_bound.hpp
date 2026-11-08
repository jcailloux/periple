#pragma once

// Assignment bound for branch_and_bound, any instance
//
// Kuhn (1955), "The Hungarian Method for the Assignment Problem"
//   -- the sum of a feasible dual bounds every assignment, Theorem 5
// Carpaneto, Toth (1980), "Some New Branching and Bounding Criteria for the Asymmetric
//   Travelling Salesman Problem"
//   -- the parent's reduced costs carried to the child, the assignment stopped once its
//      bound reaches the best tour
// Jonker, Volgenant (1987), "A Shortest Augmenting Path Algorithm for Dense and Sparse
//   Linear Assignment Problems"
//   -- shortest augmenting paths by Dijkstra on reduced costs, the columns at the
//      current minimum scanned together, prices updated before augmenting

#include <periple/algorithms/branch_and_bound/bound_detail.hpp>
#include <periple/algorithms/branch_and_bound/search.hpp>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace periple {

// Lower bound for any instance, meant for costs without structure under a
// variant that rejects most tours, such as tight time windows: a node costs a
// few row scans where SpanningArborescenceBound ascends. Passed as
// branch_and_bound(AssignmentBound{}). The rest of the tour leaves the last
// placed city and every unvisited city once, and enters every unvisited city and
// the first one once: an assignment of those rows to those columns, with no city
// to itself and no arc from the last city to the first. The sum of any dual
// without a negative reduced cost bounds it (Kuhn). Each node starts from its
// parent's duals and matching (Carpaneto, Toth): the arc to the city placed takes
// out a row and a column and frees at most two rows, which rise to their cheapest
// reduced cost, then rejoin the matching by shortest augmenting paths (Jonker,
// Volgenant). Within a node the dual sum only grows, so the work stops once it
// reaches the cutoff (Carpaneto, Toth). O(k^2) per path for k unvisited cities,
// O(k^3) at the root; cut short by stop, the sum so far stays a bound.
struct AssignmentBound {
	static constexpr bool assumes_nonnegative_penalties = true;

	template <DistanceSource Dist, typename Variant, typename Stop>
	void begin(const Solver<Dist, Variant>& solver, Stop& stop) {
		n_ = solver.size();
		rows_.reset(n_);
		levels_.assign(n_, {});
		nodes_.resize(n_ + 1);
		col_.resize(n_);
		dist_.resize(n_);
		pred_.resize(n_);
		// The root's: zero duals, every row free. Every other level copies its parent's.
		Level& level = levels_[solver.tour().size()];
		level.u.assign(n_, 0.0);
		level.v.assign(n_, 0.0);
		level.column_of.assign(n_, none);
		level.row_of.assign(n_, none);
		level.sum = 0;
		// One row at a time: stop is read before each, and without a cutoff,
		// raising them all first would prune nothing.
		const std::size_t m = collect(solver);
		for (std::size_t r = 0; r + 1 < m; ++r) {
			free_.assign(1, nodes_[r]);
			solve(solver, level, m, std::numeric_limits<double>::infinity(), stop);
		}
	}

	template <DistanceSource Dist, typename Variant, typename Stop>
	auto lower_bound(const Solver<Dist, Variant>& solver, double cutoff, Stop& stop) -> double {
		const auto tour = solver.tour();
		const std::size_t depth = tour.size();
		Level& level = levels_[depth];
		level = levels_[depth - 1];

		// The arc from the parent's last city to the city placed is now fixed: its
		// row and column leave, and the row matched to that column comes free.
		const auto parent_last = static_cast<std::size_t>(tour[depth - 2]);
		const auto placed = static_cast<std::size_t>(tour[depth - 1]);
		const auto first = static_cast<std::size_t>(tour.front());
		level.sum -= level.u[parent_last] + level.v[placed];
		free_.clear();
		if (const std::size_t j = level.column_of[parent_last]; j != none) level.row_of[j] = none;
		if (const std::size_t i = level.row_of[placed]; i != none) {
			level.column_of[i] = none;
			free_.push_back(i);
		}
		// The city placed now ends the path, which reaches the first city only after the others.
		if (level.column_of[placed] == first) {
			level.column_of[placed] = none;
			level.row_of[first] = none;
			free_.push_back(placed);
		}
		const std::size_t m = collect(solver);
		return detail::rounded_bound<typename dist_traits<Dist>::cost_type>(solve(solver, level, m, cutoff, stop));
	}

private:
	static constexpr std::size_t none = std::numeric_limits<std::size_t>::max();

	// Duals and matching of a node, by city. Every reduced cost c(i, j) - u[i] - v[j]
	// stays at least zero, and zero on a matched pair.
	struct Level {
		std::vector<double> u, v;                    // row and column duals
		std::vector<std::size_t> column_of, row_of;  // the matching, none when free
		double sum = 0;                              // over the node's rows and columns
	};

	// nodes_ = [last, unvisited..., first]: rows nodes_[0, m - 1), columns nodes_[1, m).
	template <DistanceSource Dist, typename Variant>
	auto collect(const Solver<Dist, Variant>& solver) -> std::size_t {
		using city_type = typename dist_traits<Dist>::city_type;
		const auto tour = solver.tour();
		std::size_t m = 0;
		nodes_[m++] = static_cast<std::size_t>(tour.back());
		// Every city written, the visited ones overwritten: whether a city is
		// visited follows no pattern, and a branch on it would mispredict.
		for (std::size_t c = 0; c < n_; ++c) {
			nodes_[m] = c;
			m += !solver.is_visited(static_cast<city_type>(c));
		}
		nodes_[m++] = static_cast<std::size_t>(tour.front());
		return m;
	}

	// The column row i may not take: the last city's is the first city, every
	// other city's is itself.
	auto forbidden(std::size_t i, std::size_t m) const -> std::size_t { return i == nodes_[0] ? nodes_[m - 1] : i; }

	// Raises the rows in free_ to their cheapest reduced cost, then matches each
	// by a shortest augmenting path. Returns the dual sum, cut short once it
	// reaches cutoff or stop fires.
	template <DistanceSource Dist, typename Variant, typename Stop>
	auto solve(const Solver<Dist, Variant>& solver, Level& level, std::size_t m, double cutoff, Stop& stop) -> double {
		for (const std::size_t i : free_) {
			if (stop()) return level.sum;
			const auto& from = rows_.from(solver, i);
			const std::size_t skip = forbidden(i, m);
			double cheapest = std::numeric_limits<double>::infinity();
			for (std::size_t c = 1; c < m; ++c) {
				const std::size_t j = nodes_[c];
				if (j != skip) cheapest = std::min(cheapest, from[j] - level.v[j]);
			}
			level.sum += cheapest - level.u[i];
			level.u[i] = cheapest;
		}
		if (detail::rounded_bound<typename dist_traits<Dist>::cost_type>(level.sum) >= cutoff) return level.sum;
		for (const std::size_t i : free_) {
			double reached = 0;
			if (!augment(solver, level, i, m, cutoff, stop, reached)) return reached;
		}
		return level.sum;
	}

	// Jonker and Volgenant's AUGMENT from the free row i0: Dijkstra on reduced
	// costs, scanning together the columns at the current distance mu, until a
	// free column. The columns scanned before then lower their price by mu less
	// their distance, and their rows rise as much, so the path costs zero; it
	// flips, and the dual sum rises by mu. Returns false, the level unchanged and
	// reached at sum + mu, once that reaches cutoff or stop fires.
	template <DistanceSource Dist, typename Variant, typename Stop>
	bool augment(const Solver<Dist, Variant>& solver, Level& level, std::size_t i0, std::size_t m, double cutoff,
	             Stop& stop, double& reached)
	{
		const std::size_t k = m - 1;
		std::copy_n(nodes_.begin() + 1, k, col_.begin());
		{
			const auto& from = rows_.from(solver, i0);
			const std::size_t skip = forbidden(i0, m);
			for (std::size_t c = 0; c < k; ++c) {
				const std::size_t j = col_[c];
				dist_[j] = j == skip ? std::numeric_limits<double>::infinity() : from[j] - level.u[i0] - level.v[j];
				pred_[j] = i0;
			}
		}
		// col_[0, low) scanned, [low, up) at distance mu to scan, [up, k) farther;
		// col_[0, scanned) lie below mu.
		std::size_t low = 0, up = 0, scanned = 0, end = none;
		double mu = 0;
		while (end == none) {
			if (up == low) {
				scanned = low;
				mu = dist_[col_[up++]];
				for (std::size_t c = up; c < k; ++c) {
					const std::size_t j = col_[c];
					if (dist_[j] > mu) continue;
					if (dist_[j] < mu) {
						up = low;
						mu = dist_[j];
					}
					col_[c] = col_[up];
					col_[up++] = j;
				}
				if (detail::rounded_bound<typename dist_traits<Dist>::cost_type>(level.sum + mu) >= cutoff) {
					reached = level.sum + mu;
					return false;
				}
				for (std::size_t c = low; c < up; ++c)
					if (level.row_of[col_[c]] == none) {
						end = col_[c];
						break;
					}
				if (end != none) break;
			}
			if (stop()) {
				reached = level.sum + mu;
				return false;
			}
			const std::size_t i = level.row_of[col_[low++]];
			const auto& from = rows_.from(solver, i);
			const std::size_t skip = forbidden(i, m);
			const double base = mu - level.u[i];
			for (std::size_t c = up; c < k; ++c) {
				const std::size_t j = col_[c];
				if (j == skip) continue;
				const double h = base + from[j] - level.v[j];
				if (h >= dist_[j]) continue;
				dist_[j] = h;
				pred_[j] = i;
				if (h == mu) {
					if (level.row_of[j] == none) {
						end = j;
						break;
					}
					col_[c] = col_[up];
					col_[up++] = j;
				}
			}
		}
		for (std::size_t c = 0; c < scanned; ++c) {
			const std::size_t j = col_[c];
			const double lift = mu - dist_[j];
			level.v[j] -= lift;
			level.u[level.row_of[j]] += lift;
		}
		level.u[i0] += mu;
		level.sum += mu;
		for (std::size_t j = end;;) {
			const std::size_t i = pred_[j];
			level.row_of[j] = i;
			std::swap(level.column_of[i], j);
			if (i == i0) break;
		}
		return true;
	}

	std::size_t n_ = 0;
	detail::DistanceRows rows_;
	std::vector<Level> levels_;  // levels_[d]: the node at depth d, allocated on first visit
	std::vector<std::size_t> nodes_, free_;
	// Augmentation workspace: col_ orders the columns, dist_ and pred_ by city.
	std::vector<std::size_t> col_, pred_;
	std::vector<double> dist_;
};

} // namespace periple
