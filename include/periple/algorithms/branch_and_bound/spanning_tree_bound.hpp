#pragma once

// Spanning tree bound for branch_and_bound, symmetric instances
//
// Prim (1957), "Shortest Connection Networks and Some Generalizations"
//   -- minimum spanning tree
// Christofides (1970), "The Shortest Hamiltonian Chain of a Graph"
//   -- a chain with fixed ends as a spanning tree under vertex penalties
// Held, Karp (1970), "The Traveling-Salesman Problem and Minimum Spanning Trees"
//   -- lower bounds from a spanning structure under node weights
// Held, Karp (1971), "The Traveling-Salesman Problem and Minimum Spanning Trees: Part II"
//   -- ascent of the weights toward a target, at every node, from the parent's
//      weights; a node is discarded once its bound reaches the best tour
// Held, Wolfe, Crowder (1974), "Validation of Subgradient Optimization"
//   -- an upper bound as the target of the ascent
// Volgenant, Jonker (1982), "A Branch and Bound Algorithm for the Symmetric Traveling
//   Salesman Problem Based on the 1-Tree Relaxation"
//   -- ascent of fixed length, integer bound aiming at u - 1
// Fisher (1985), "An Applications Oriented Guide to Lagrangian Relaxation"
//   -- step factor from 2, halved when the bound stalls

#include <periple/algorithms/branch_and_bound/bound_detail.hpp>
#include <periple/algorithms/branch_and_bound/search.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <span>
#include <vector>

namespace periple {

// Lower bound for symmetric instances. The rest of the tour is a Hamiltonian
// chain from the last placed city through every unvisited city to the first: a
// spanning tree of those cities whose ends have degree 1 and the others degree 2
// (Christofides). Relaxing the degrees with Lagrangian weights pi, as Held and
// Karp do for the 1-tree, gives a bound valid for any pi; each node ascends pi
// from its parent's. O(iterations * k^2) for k unvisited cities, cut short by
// stop: the best value so far stays a bound.
struct SpanningTreeBound {
	std::size_t iterations = 30;  // subgradient steps per node, at most

	static constexpr bool assumes_nonnegative_penalties = true;

	template <DistanceSource Dist, typename Variant>
	void begin(const Solver<Dist, Variant>& solver) {
		assert(solver.symmetric() && "SpanningTreeBound: requires a symmetric distance source (set_symmetric(true))");
		n_ = solver.size();
		pi_.assign(n_, {});
		pi_[solver.tour().size()].assign(n_, 0.0);  // the root's: every other row copies its parent's
		rows_.reset(n_);
		nodes_.resize(n_ + 1);
		key_.resize(n_ + 1);
		parent_.resize(n_ + 1);
		degree_.resize(n_ + 1);
	}

	template <DistanceSource Dist, typename Variant, typename Stop>
	auto lower_bound(const Solver<Dist, Variant>& solver, double cutoff, Stop& stop) -> double {
		using city_type = typename dist_traits<Dist>::city_type;
		const auto tour = solver.tour();
		const std::size_t depth = tour.size();
		pi_[depth] = pi_[depth - 1];
		// Without a tour to beat no value prunes, and a step aims at the cutoff:
		// the weights pass on unchanged.
		if (std::isinf(cutoff)) return stopped;

		// The path ends first: nodes_[0] is the last placed city, nodes_[m - 1] the first one.
		std::size_t m = 0;
		nodes_[m++] = static_cast<std::size_t>(tour.back());
		for (std::size_t c = 0; c < n_; ++c)
			if (!solver.is_visited(static_cast<city_type>(c))) nodes_[m++] = c;
		nodes_[m++] = static_cast<std::size_t>(tour.front());

		const std::span<double> pi(pi_[depth]);
		auto target = [m](std::size_t i) { return (i == 0 || i + 1 == m) ? 1 : 2; };
		double best = stopped;
		double lambda = 2;
		std::size_t stalled = 0;
		for (std::size_t it = 0;; ++it) {
			const double value = tree(solver, m, pi, stop);
			stalled = value > best ? 0 : stalled + 1;
			best = std::max(best, value);
			if (value == stopped || detail::rounded_bound<typename dist_traits<Dist>::cost_type>(best) >= cutoff
			    || it == iterations)
				break;
			double norm = 0;
			for (std::size_t i = 0; i < m; ++i) {
				const double g = degree_[i] - target(i);
				norm += g * g;
			}
			if (norm == 0) break;  // the tree is a path: value is its exact cost
			const double step = lambda * (cutoff - value) / norm;
			for (std::size_t i = 0; i < m; ++i)
				pi[nodes_[i]] += step * (degree_[i] - target(i));
			if (stalled == detail::ascent_patience) {
				lambda /= 2;
				stalled = 0;
			}
		}
		return detail::rounded_bound<typename dist_traits<Dist>::cost_type>(best);
	}

private:
	static constexpr double stopped = -std::numeric_limits<double>::infinity();

	// Lagrangian value of the minimum spanning tree over nodes_[0..m) under pi;
	// fills degree_. Prim, O(m^2): each step adds the city u, then one pass over
	// the cities still out updates their keys and picks the next one.
	template <DistanceSource Dist, typename Variant, typename Stop>
	auto tree(const Solver<Dist, Variant>& solver, std::size_t m, std::span<const double> pi, Stop& stop) -> double {
		std::fill_n(key_.begin(), m, std::numeric_limits<double>::infinity());
		std::fill_n(degree_.begin(), m, 0);
		out_.resize(m - 1);
		std::iota(out_.begin(), out_.end(), std::size_t{1});
		double total = 0;
		std::size_t u = 0;
		while (!out_.empty()) {
			if (stop()) return stopped;
			const auto& from = rows_.from(solver, nodes_[u]);
			const double pi_u = pi[nodes_[u]];
			// The edge joining the two ends is left out: with a city still to visit,
			// the path never takes it. u = 0 is one end, out_.back() the other.
			const std::size_t reachable = u == 0 ? out_.size() - 1 : out_.size();
			std::size_t next = 0;
			for (std::size_t j = 0; j < reachable; ++j) {
				const std::size_t i = out_[j];
				const double w = from[nodes_[i]] + pi_u + pi[nodes_[i]];
				if (w < key_[i]) { key_[i] = w; parent_[i] = u; }
				if (key_[i] < key_[out_[next]]) next = j;
			}
			u = out_[next];
			out_.erase(out_.begin() + static_cast<std::ptrdiff_t>(next));
			total += key_[u];
			++degree_[u];
			++degree_[parent_[u]];
		}
		double weights = pi[nodes_[0]] + pi[nodes_[m - 1]];
		for (std::size_t i = 1; i + 1 < m; ++i) weights += 2 * pi[nodes_[i]];
		return total - weights;
	}

	std::size_t n_ = 0;
	detail::DistanceRows rows_;
	std::vector<std::vector<double>> pi_;  // pi_[d][c]: weight of city c at depth d, allocated on first visit
	std::vector<std::size_t> nodes_;
	std::vector<std::size_t> out_;  // indices into nodes_ not yet in the tree, ascending
	std::vector<double> key_;
	std::vector<std::size_t> parent_;
	std::vector<int> degree_;
};

} // namespace periple
