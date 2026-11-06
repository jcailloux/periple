#pragma once

// Branch and bound (exact, depth first)
//
// Prim (1957), "Shortest Connection Networks and Some Generalizations"
//   -- minimum spanning tree (SpanningTreeBound)
// Land, Doig (1960), "An Automatic Method of Solving Discrete Programming Problems"
//   -- branch and bound
// Little, Murty, Sweeney, Karel (1963), "An Algorithm for the Traveling Salesman Problem"
//   -- branch and bound on tours, row reduction (OutgoingEdgeBound)
// Christofides (1970), "The Shortest Hamiltonian Chain of a Graph"
//   -- a chain with fixed ends as a spanning tree under vertex penalties (SpanningTreeBound)
// Held, Karp (1970), "The Traveling-Salesman Problem and Minimum Spanning Trees"
//   -- lower bounds from a spanning structure under node weights (SpanningTreeBound)
// Held, Karp (1971), "The Traveling-Salesman Problem and Minimum Spanning Trees: Part II"
//   -- ascent of the weights toward a target, at every node, from the parent's
//      weights; a node is discarded once its bound reaches the best tour
// Held, Wolfe, Crowder (1974), "Validation of Subgradient Optimization"
//   -- an upper bound as the target of the ascent
// Volgenant, Jonker (1982), "A Branch and Bound Algorithm for the Symmetric Traveling
//   Salesman Problem Based on the 1-Tree Relaxation"
//   -- depth first search, ascent of fixed length, integer bound aiming at u - 1
// Fisher (1985), "An Applications Oriented Guide to Lagrangian Relaxation"
//   -- step factor from 2, halved when the bound stalls
//
// Extends the tour one city at a time, children cheapest first, through the
// variant pipeline (evaluate_append, append), so every AppendMove variant is
// honored. Little et al., Held, Karp and Volgenant, Jonker branch on edges, which
// gives a smaller tree but no partial tour for a variant to judge. The Bound
// strategy prunes a node whose completion cannot beat the best tour found.
// Backtracking pops the last city: dimensions keep their state per position,
// which the next append overwrites.

#include <periple/core/solver.hpp>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <limits>
#include <numeric>
#include <span>
#include <type_traits>
#include <vector>

namespace periple {

namespace detail {

// A bound computed on raw distances holds only if no move lowers the cost.
template <typename Bound>
concept assumes_nonnegative_penalties = requires { requires Bound::assumes_nonnegative_penalties; };

// A bound that runs long takes a stop check as third argument.
template <typename Bound, typename S, typename... Stop>
concept lower_bound_with = requires(Bound& b, const S& s, double cutoff, Stop&... stop) {
	{ b.lower_bound(s, cutoff, stop...) } -> std::convertible_to<double>;
};

} // namespace detail

// Lower bound for any instance, the row reduction of Little et al.: each city
// that has yet to leave, the last placed one and every unvisited one, leaves
// through its cheapest edge to a city it may still reach. A city's list is
// sorted on first use, O(n log n).
struct OutgoingEdgeBound {
	static constexpr bool assumes_nonnegative_penalties = true;

	template <DistanceSource Dist, typename Variant>
	void begin(const Solver<Dist, Variant>& solver) {
		order_.assign(solver.size(), {});
	}

	template <DistanceSource Dist, typename Variant, typename Stop>
	auto lower_bound(const Solver<Dist, Variant>& solver, double, Stop& stop) -> double {
		using city_type = typename dist_traits<Dist>::city_type;
		const auto tour = solver.tour();
		const auto first = static_cast<std::size_t>(tour.front());
		auto cheapest = [&](std::size_t from, bool to_first) {
			for (const auto to : sorted_from(solver, from)) {
				if (to == from) continue;
				if (!solver.is_visited(static_cast<city_type>(to)) || (to_first && to == first))
					return static_cast<double>(solver.distance(static_cast<city_type>(from), static_cast<city_type>(to)));
			}
			return 0.0;  // unreachable: a city is still unvisited
		};
		// The last city goes on to an unvisited one, never back to the first.
		double total = cheapest(static_cast<std::size_t>(tour.back()), false);
		for (std::size_t u = 0; u < order_.size(); ++u) {
			if (solver.is_visited(static_cast<city_type>(u))) continue;
			// Weights can be negative: a partial sum bounds nothing.
			if (stop()) return -std::numeric_limits<double>::infinity();
			total += cheapest(u, true);
		}
		return total;
	}

private:
	template <DistanceSource Dist, typename Variant>
	auto sorted_from(const Solver<Dist, Variant>& solver, std::size_t from) -> const std::vector<std::size_t>& {
		using city_type = typename dist_traits<Dist>::city_type;
		auto& row = order_[from];
		if (row.empty()) {
			row.resize(order_.size());
			std::iota(row.begin(), row.end(), std::size_t{0});
			const auto origin = static_cast<city_type>(from);
			std::sort(row.begin(), row.end(), [&](std::size_t x, std::size_t y) {
				return solver.distance(origin, static_cast<city_type>(x)) < solver.distance(origin, static_cast<city_type>(y));
			});
		}
		return row;
	}

	std::vector<std::vector<std::size_t>> order_;  // order_[a][k]: k-th nearest city from a (a itself included)
};

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
		distances_.assign(n_, {});
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
		const std::span<double> pi(pi_[depth]);

		// The path ends first: nodes_[0] is the last placed city, nodes_[m - 1] the first one.
		std::size_t m = 0;
		nodes_[m++] = static_cast<std::size_t>(tour.back());
		for (std::size_t c = 0; c < n_; ++c)
			if (!solver.is_visited(static_cast<city_type>(c))) nodes_[m++] = c;
		nodes_[m++] = static_cast<std::size_t>(tour.front());

		// Rounding margin; an integer completion cost rounds the bound up, so the
		// ascent can stop short of the cutoff.
		auto rounded = [](double bound) {
			bound -= 1e-9 * (1.0 + std::abs(bound));
			if constexpr (std::is_integral_v<typename dist_traits<Dist>::cost_type>)
				bound = std::ceil(bound);
			return bound;
		};

		auto target = [m](std::size_t i) { return (i == 0 || i + 1 == m) ? 1 : 2; };
		double best = stopped;
		double lambda = 2;
		std::size_t stalled = 0;
		for (std::size_t it = 0;; ++it) {
			const double value = tree(solver, m, pi, stop);
			stalled = value > best ? 0 : stalled + 1;
			best = std::max(best, value);
			if (value == stopped || rounded(best) >= cutoff || it == iterations || std::isinf(cutoff)) break;
			double norm = 0;
			for (std::size_t i = 0; i < m; ++i) {
				const double g = degree_[i] - target(i);
				norm += g * g;
			}
			if (norm == 0) break;  // the tree is a path: value is its exact cost
			const double step = lambda * (cutoff - value) / norm;
			for (std::size_t i = 0; i < m; ++i)
				pi[nodes_[i]] += step * (degree_[i] - target(i));
			if (stalled == patience) {
				lambda /= 2;
				stalled = 0;
			}
		}
		return rounded(best);
	}

private:
	static constexpr double stopped = -std::numeric_limits<double>::infinity();
	static constexpr std::size_t patience = 5;  // steps without progress before lambda halves

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
			const auto& from = distances_from(solver, nodes_[u]);
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

	// Read once per city: the ascent comes back to the same distances at every step.
	template <DistanceSource Dist, typename Variant>
	auto distances_from(const Solver<Dist, Variant>& solver, std::size_t from) -> const std::vector<double>& {
		using city_type = typename dist_traits<Dist>::city_type;
		auto& row = distances_[from];
		if (row.empty()) {
			row.resize(n_);
			for (std::size_t to = 0; to < n_; ++to)
				row[to] = static_cast<double>(solver.distance(static_cast<city_type>(from), static_cast<city_type>(to)));
		}
		return row;
	}

	std::size_t n_ = 0;
	std::vector<std::vector<double>> pi_;         // pi_[d][c]: weight of city c at depth d, allocated on first visit
	std::vector<std::vector<double>> distances_;  // distances_[a][b], a row allocated on first use
	std::vector<std::size_t> nodes_;
	std::vector<std::size_t> out_;  // indices into nodes_ not yet in the tree, ascending
	std::vector<double> key_;
	std::vector<std::size_t> parent_;
	std::vector<int> degree_;
};

// ---------------------------------------------------------------------------
// Solver::branch_and_bound
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
template <typename Bound>
auto Solver<Dist, Variant>::branch_and_bound(Bound bound, BranchAndBoundParams params) -> Solver& {
	return detail::with_deadline(params.time_limit, [&](auto& deadline) -> Solver& {
		return branch_and_bound_run(bound, params, deadline);
	});
}

template <DistanceSource Dist, typename Variant>
template <typename Bound, typename D>
auto Solver<Dist, Variant>::branch_and_bound_run(
	Bound& bound, const BranchAndBoundParams& params, D& deadline)
	-> Solver&
{
	detail::Poller stop(deadline);
	[[maybe_unused]] detail::Poller stop_bound(deadline);  // not the nodes': a bound's steps cost far less
	constexpr bool takes_stop = detail::lower_bound_with<Bound, Solver, decltype(stop_bound)>;
	static_assert(takes_stop || detail::lower_bound_with<Bound, Solver>,
		"branch_and_bound: the strategy needs lower_bound(const Solver&, double cutoff) returning a double");

	assert(dist_ && "branch_and_bound: no distance source set");

	if (try_trivial()) {
		status_ = SolutionStatus::optimal;
		return *this;
	}

	const auto total = dist_->size();
	if (!bb_cache_) bb_cache_.emplace();
	auto& ws = *bb_cache_;
	ws.children.resize(total);
	ws.next.resize(total);
	ws.cost.resize(total);
	ws.best.resize(total);

	// A complete tour held on entry is the first to beat, if it is among the tours searched.
	const bool held = status_ == SolutionStatus::feasible || status_ == SolutionStatus::optimal;
	bool found = false;
	cost_type best_cost{};
	auto keep = [&](std::size_t rotation) {
		std::rotate_copy(tour_.begin(), tour_.begin() + static_cast<std::ptrdiff_t>(rotation),
		                 tour_.begin() + static_cast<std::ptrdiff_t>(total), ws.best.begin());
		best_cost = cost_;
		found = true;
	};

	if (params.resume_at > 0) {
		assert(params.resume_at <= n_ && "branch_and_bound: resume_at exceeds current tour length");
		assert(params.resume_at < total && "branch_and_bound: resume_at leaves no city to place");
		if (held) keep(0);  // it extends its own prefix
		resume_prefix(params.resume_at);
	} else {
		assert(params.start_city < total && "branch_and_bound: start_city out of bounds");
		if (held) {
			// Without a variant a rotation keeps the cost, so any complete tour bounds the search.
			const auto p = static_cast<std::size_t>(position_[params.start_city]);
			assert((p == 0 || !has_callbacks)
				&& "branch_and_bound: with a variant, a held tour starting elsewhere than start_city does not bound this search (clear() first)");
			if (p == 0 || !has_callbacks) keep(p);
		}
		clear();
		append(static_cast<city_type>(params.start_city));
	}

	auto lower_bound = [&](double cutoff) -> double {
		if constexpr (takes_stop) return bound.lower_bound(*this, cutoff, stop_bound);
		else return bound.lower_bound(*this, cutoff);
	};

	auto check_penalty = [&] {
		if constexpr (detail::assumes_nonnegative_penalties<Bound>)
			assert(ctx_.cost_delta >= 0
				&& "branch_and_bound: move_prepare set a negative cost_delta, which the bound assumes away (pass a bound valid for this variant)");
	};

	// Scores the children of the current node through the pipeline, cheapest first.
	auto expand = [&] {
		const std::size_t d = n_;
		auto& kids = ws.children[d];
		kids.clear();
		kids.reserve(total - d);
		for (std::size_t c = 0; c < total; ++c) {
			if (visited_[c]) continue;
			const auto city = static_cast<city_type>(c);
			const auto score = evaluate_append(city);
			if (!score) continue;
			check_penalty();
			kids.push_back({*score, city});
		}
		std::sort(kids.begin(), kids.end(), [](const auto& a, const auto& b) {
			return a.score < b.score || (a.score == b.score && a.city < b.city);
		});
		ws.next[d] = 0;
		ws.cost[d] = cost_;
	};

	const std::size_t root = n_;
	const std::size_t max_nodes = params.max_nodes ? params.max_nodes
	                                               : std::numeric_limits<std::size_t>::max();
	std::size_t nodes = 0;

	// Depth first; true once the tree is exhausted, false if a limit cuts it short.
	auto search = [&] {
		// An expired limit does no work beyond the start city.
		if (stop()) return false;
		[[maybe_unused]] detail::Poller stop_begin(deadline);  // its units cost differently again
		if constexpr (requires { bound.begin(*this, stop_begin); })
			bound.begin(*this, stop_begin);
		else if constexpr (requires { bound.begin(*this); })
			bound.begin(*this);
		expand();
		for (;;) {
			const std::size_t d = n_;
			if (ws.next[d] == ws.children[d].size()) {
				if (d == root) return true;
				pop_back(ws.cost[d - 1]);
				continue;
			}
			if (nodes == max_nodes || stop()) return false;
			append(ws.children[d][ws.next[d]++].city);
			++nodes;
			if (n_ == total) {
				// append closed the tour and ran the closing edge through the pipeline.
				if (status_ == SolutionStatus::feasible) {
					check_penalty();
					if (!found || cost_ < best_cost) keep(0);
				}
				pop_back(ws.cost[d]);
				continue;
			}
			const double cutoff = found ? static_cast<double>(best_cost) - static_cast<double>(cost_)
			                            : std::numeric_limits<double>::infinity();
			if (lower_bound(cutoff) >= cutoff) {
				pop_back(ws.cost[d]);
				continue;
			}
			expand();
		}
	};
	const bool exhausted = search();

	if (found) {
		set_tour(std::span<const city_type>(ws.best.data(), total));
		// With a longer fixed prefix, the tour is optimal among its completions only.
		status_ = (exhausted && root == 1) ? SolutionStatus::optimal : SolutionStatus::feasible;
	} else if (exhausted) {
		n_ = 0;
		cost_ = {};
		++tour_version_;
		status_ = SolutionStatus::infeasible;
	} else {
		// Stopped before any complete tour: back to the root prefix, still partial.
		while (n_ > root) pop_back(ws.cost[n_ - 1]);
	}
	return *this;
}

// ---------------------------------------------------------------------------
// Solver::branch_and_bound -- facade with the default bound
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::branch_and_bound(BranchAndBoundParams params) -> Solver& {
	if (symmetric_) return branch_and_bound(SpanningTreeBound{}, params);
	return branch_and_bound(OutgoingEdgeBound{}, params);
}

} // namespace periple
