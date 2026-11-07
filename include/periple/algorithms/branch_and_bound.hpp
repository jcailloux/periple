#pragma once

// Branch and bound (exact, depth first)
//
// Prim (1957), "Shortest Connection Networks and Some Generalizations"
//   -- minimum spanning tree (SpanningTreeBound)
// Land, Doig (1960), "An Automatic Method of Solving Discrete Programming Problems"
//   -- branch and bound
// Little, Murty, Sweeney, Karel (1963), "An Algorithm for the Traveling Salesman Problem"
//   -- branch and bound on tours
// Edmonds (1967), "Optimum Branchings"
//   -- a path as a spanning arborescence whose arcs leave distinct nodes; the
//      minimum arborescence by contracting cycles, rule (1), and expanding them,
//      step (I3) (SpanningArborescenceBound)
// Christofides (1970), "The Shortest Hamiltonian Chain of a Graph"
//   -- a chain with fixed ends as a spanning tree under vertex penalties (SpanningTreeBound)
// Held, Karp (1970), "The Traveling-Salesman Problem and Minimum Spanning Trees"
//   -- lower bounds from a spanning structure under node weights; for asymmetric
//      costs, weights on the node an arc leaves (SpanningTreeBound,
//      SpanningArborescenceBound)
// Held, Karp (1971), "The Traveling-Salesman Problem and Minimum Spanning Trees: Part II"
//   -- ascent of the weights toward a target, at every node, from the parent's
//      weights; a node is discarded once its bound reaches the best tour
// Held, Wolfe, Crowder (1974), "Validation of Subgradient Optimization"
//   -- an upper bound as the target of the ascent
// Tarjan (1977), "Finding Optimum Branchings"
//   -- the cheapest arc into each contracted cycle, O(n^2) on a dense graph
//      (SpanningArborescenceBound)
// Camerini, Fratta, Maffioli (1979), "A Note on Finding Optimum Branchings"
//   -- the nested contractions kept, to expand them backward (SpanningArborescenceBound)
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

// Distances as double, a row read on first use: an ascent comes back to the
// same rows at every step.
class DistanceRows {
public:
	void reset(std::size_t n) { rows_.assign(n, {}); }

	template <DistanceSource Dist, typename Variant>
	auto from(const Solver<Dist, Variant>& solver, std::size_t a) -> const std::vector<double>& {
		using city_type = typename dist_traits<Dist>::city_type;
		auto& row = rows_[a];
		if (row.empty()) {
			row.resize(rows_.size());
			for (std::size_t b = 0; b < rows_.size(); ++b)
				row[b] = static_cast<double>(solver.distance(static_cast<city_type>(a), static_cast<city_type>(b)));
		}
		return row;
	}

private:
	std::vector<std::vector<double>> rows_;  // rows_[a][b], a row allocated on first use
};

// Rounding margin; an integer completion cost rounds the bound up, so the
// ascent can stop short of the cutoff.
template <typename Cost>
double rounded_bound(double bound) {
	bound -= 1e-9 * (1.0 + std::abs(bound));
	if constexpr (std::is_integral_v<Cost>) bound = std::ceil(bound);
	return bound;
}

// Subgradient steps without progress before the step factor halves.
inline constexpr std::size_t ascent_patience = 5;

} // namespace detail

// Lower bound for any instance, the directed mirror of SpanningTreeBound. The
// rest of the tour is a path from the last placed city through every unvisited
// city to the first: a spanning arborescence rooted at the last city whose arcs
// leave distinct cities (Edmonds). Relaxing that with weights pi on the city an
// arc leaves, as Held and Karp do for the 1-arborescence, gives a bound valid
// for any pi; each node ascends pi from its parent's. O(iterations * k^2) for k
// unvisited cities, cut short by stop: the best value so far stays a bound.
struct SpanningArborescenceBound {
	std::size_t iterations = 30;  // subgradient steps per node, at most

	static constexpr bool assumes_nonnegative_penalties = true;

	template <DistanceSource Dist, typename Variant>
	void begin(const Solver<Dist, Variant>& solver) {
		n_ = solver.size();
		pi_.assign(n_, {});
		pi_[solver.tour().size()].assign(n_, 0.0);  // the root's: every other row copies its parent's
		rows_.reset(n_);
		nodes_.resize(n_ + 1);
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

		// The root first: nodes_[0] is the last placed city, nodes_[m - 1] the first one.
		std::size_t m = 0;
		nodes_[m++] = static_cast<std::size_t>(tour.back());
		for (std::size_t c = 0; c < n_; ++c)
			if (!solver.is_visited(static_cast<city_type>(c))) nodes_[m++] = c;
		nodes_[m++] = static_cast<std::size_t>(tour.front());

		// The ascent of SpanningTreeBound on out-degrees: on the path every city
		// leaves once, but the first, where it ends.
		const std::span<double> pi(pi_[depth]);
		auto target = [m](std::size_t i) { return i + 1 == m ? 0 : 1; };
		double best = stopped;
		double lambda = 2;
		std::size_t stalled = 0;
		for (std::size_t it = 0;; ++it) {
			const double value = arborescence(solver, m, pi, stop);
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
			if (norm == 0) break;  // the arborescence is the path: value is its exact cost
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
	static constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
	static constexpr unsigned char unseen = 0, walking = 1, done = 2;

	// Lagrangian value of the minimum arborescence rooted at nodes_[0] over
	// nodes_[0..m) under pi, arc a -> b weighing its distance plus pi of a; fills
	// degree_ with the arcs leaving each node. -infinity once stopped.
	//
	// Every node but the root takes its cheapest entering arc. Following them, a
	// walk that comes back on itself closes a cycle, which merges in place into
	// the slot where the walk met it: an arc into the cycle costs its weight less
	// the cycle arc it would replace (Edmonds' rule (1)), an arc out of it the
	// cheapest member's. Expanding the merges backward, the arc into each cycle
	// replaces the cycle arc into the member that holds its head (step (I3)).
	// O(m^2): a merge of c slots costs O(c m) and removes c - 1 of them.
	template <DistanceSource Dist, typename Variant, typename Stop>
	auto arborescence(const Solver<Dist, Variant>& solver, std::size_t m, std::span<const double> pi, Stop& stop)
		-> double
	{
		constexpr double inf = std::numeric_limits<double>::infinity();
		const std::size_t end = m - 1;
		weight_.assign(m * m, inf);
		arc_.resize(m * m);
		cheapest_.assign(m, inf);
		in_.assign(m, 0);
		for (std::size_t a = 0; a < end; ++a) {  // the first city leaves nothing: the path ends there
			if (stop()) return stopped;
			const auto& from = rows_.from(solver, nodes_[a]);
			const double pi_a = pi[nodes_[a]];
			for (std::size_t b = 1; b < m; ++b) {  // nothing enters the root
				// The path never joins its two ends while a city remains between them.
				if (b == a || (a == 0 && b == end && m > 2)) continue;
				const double w = from[nodes_[b]] + pi_a;
				weight_[a * m + b] = w;
				arc_[a * m + b] = a * m + b;
				if (w < cheapest_[b]) { cheapest_[b] = w; in_[b] = a; }
			}
		}

		active_.resize(m);
		std::iota(active_.begin(), active_.end(), std::size_t{0});
		node_of_.resize(m);
		std::iota(node_of_.begin(), node_of_.end(), std::size_t{0});
		parent_.assign(2 * m, none);
		cycle_arc_.resize(2 * m);
		final_arc_.resize(2 * m);
		members_.clear();
		first_member_.assign(1, 0);
		in_cycle_.assign(m, 0);
		reduce_.resize(m);
		state_.assign(m, unseen);
		state_[0] = done;
		std::size_t next = m;  // forest node of the next merge
		for (std::size_t v = 1; v < m; ++v) {
			if (state_[v] != unseen) continue;
			walk_.clear();
			std::size_t x = v;
			while (state_[x] != done) {
				if (state_[x] == unseen) {
					state_[x] = walking;
					walk_.push_back(x);
					x = in_[x];
					continue;
				}
				// x is on the walk: the slots from x to the walk's end form a cycle.
				if (stop()) return stopped;
				cycle_.clear();
				std::size_t y;
				do { y = walk_.back(); walk_.pop_back(); cycle_.push_back(y); } while (y != x);
				for (const std::size_t c : cycle_) {
					in_cycle_[c] = 1;
					reduce_[c] = weight_[in_[c] * m + c];
					parent_[node_of_[c]] = next;
					cycle_arc_[node_of_[c]] = arc_[in_[c] * m + c];
					members_.push_back(node_of_[c]);
				}
				first_member_.push_back(members_.size());
				for (const std::size_t u : active_) {
					if (in_cycle_[u]) continue;
					double into = inf, out = inf;
					std::size_t arc_into = 0, arc_out = 0;
					for (const std::size_t c : cycle_) {
						const double w = weight_[u * m + c] - reduce_[c];
						if (w < into) { into = w; arc_into = arc_[u * m + c]; }
						if (weight_[c * m + u] < out) { out = weight_[c * m + u]; arc_out = arc_[c * m + u]; }
					}
					weight_[u * m + x] = into;
					arc_[u * m + x] = arc_into;
					weight_[x * m + u] = out;
					arc_[x * m + u] = arc_out;
				}
				weight_[x * m + x] = inf;
				// A slot that entered a member enters the cycle, at the same cost.
				std::size_t kept = 0;
				for (const std::size_t u : active_) {
					if (in_cycle_[u] && u != x) continue;
					if (u != x && in_cycle_[in_[u]]) in_[u] = x;
					active_[kept++] = u;
				}
				active_.resize(kept);
				for (const std::size_t c : cycle_) in_cycle_[c] = 0;
				node_of_[x] = next++;
				double best = inf;
				for (const std::size_t u : active_)
					if (u != x && weight_[u * m + x] < best) { best = weight_[u * m + x]; in_[x] = u; }
				walk_.push_back(x);
				x = in_[x];
			}
			for (const std::size_t s : walk_) state_[s] = done;
		}

		for (const std::size_t s : active_)
			if (s != 0) final_arc_[node_of_[s]] = arc_[in_[s] * m + s];
		for (std::size_t t = next; t-- > m;) {
			const std::size_t arc = final_arc_[t];
			std::size_t y = arc % m;  // the head, then its ancestor right under t
			while (parent_[y] != t) y = parent_[y];
			for (std::size_t k = first_member_[t - m]; k < first_member_[t - m + 1]; ++k) {
				const std::size_t c = members_[k];
				final_arc_[c] = c == y ? arc : cycle_arc_[c];
			}
		}
		std::fill_n(degree_.begin(), m, 0);
		double total = 0;
		for (std::size_t b = 1; b < m; ++b) {
			const std::size_t a = final_arc_[b] / m;
			total += rows_.from(solver, nodes_[a])[nodes_[b]] + pi[nodes_[a]];
			++degree_[a];
		}
		double weights = 0;
		for (std::size_t i = 0; i < end; ++i) weights += pi[nodes_[i]];
		return total - weights;
	}

	std::size_t n_ = 0;
	detail::DistanceRows rows_;
	std::vector<std::vector<double>> pi_;  // pi_[d][c]: weight of city c at depth d, allocated on first visit
	std::vector<std::size_t> nodes_;
	std::vector<int> degree_;
	// Arborescence workspace. weight_ and arc_ are m x m, slot by slot; an arc is
	// named a * m + b by its slots a -> b before any merge. Forest nodes: slots
	// 0..m), then one per merge.
	std::vector<double> weight_, cheapest_, reduce_;
	std::vector<std::size_t> arc_, in_, active_, node_of_, parent_, cycle_arc_, final_arc_;
	std::vector<std::size_t> members_, first_member_;  // members of merge t: [first_member_[t - m], first_member_[t - m + 1])
	std::vector<std::size_t> walk_, cycle_;
	std::vector<unsigned char> in_cycle_, state_;
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
	return branch_and_bound(SpanningArborescenceBound{}, params);
}

} // namespace periple
