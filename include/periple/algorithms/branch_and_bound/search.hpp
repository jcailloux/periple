#pragma once

// Branch and bound (exact, depth first): the search, the strategy contract and the default order
//
// Land, Doig (1960), "An Automatic Method of Solving Discrete Programming Problems"
//   -- branch and bound
// Little, Murty, Sweeney, Karel (1963), "An Algorithm for the Traveling Salesman Problem"
//   -- branch and bound on tours
// Held, Karp (1971), "The Traveling-Salesman Problem and Minimum Spanning Trees: Part II"
//   -- a node is discarded once its bound reaches the best tour
// Volgenant, Jonker (1982), "A Branch and Bound Algorithm for the Symmetric Traveling
//   Salesman Problem Based on the 1-Tree Relaxation"
//   -- depth first search
//
// Extends the tour one city at a time, children in the order a strategy sets
// (cheapest first by default), through the variant pipeline (evaluate_append,
// append), so every AppendMove variant is honored. Little et al., Held, Karp and
// Volgenant, Jonker branch on edges, which gives a smaller tree but no partial
// tour for a variant to judge. The Bound strategy prunes a node whose completion
// cannot beat the best tour found. Backtracking pops the last city: dimensions
// keep their state per position, which the next append overwrites.

#include <periple/core/solver.hpp>

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

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

// An order that runs long takes a stop check as last argument, in either form.
template <typename Order, typename S, typename Child, typename... Stop>
concept order_with = requires(Order& o, const S& s, std::span<Child> children, double cutoff, Stop&... stop) {
	o.order(s, children, cutoff, stop...);
};

template <typename Order, typename S, typename Child, typename... Stop>
concept pick_with = requires(Order& o, const S& s, std::span<const Child> remaining, double cutoff, Stop&... stop) {
	{ o.pick(s, remaining, cutoff, stop...) } -> std::convertible_to<std::size_t>;
};

} // namespace detail

// What lower_bound returns for a node without a feasible completion: it prunes
// the node even before any tour is known, when the cutoff is +infinity.
inline constexpr double dead_end = std::numeric_limits<double>::infinity();

// The default order: cheapest score first, ties by city. The first descent
// makes nearest_neighbor's choices.
struct CheapestFirst {
	template <DistanceSource Dist, typename Variant, typename CityT>
	void order(const Solver<Dist, Variant>&, std::span<BranchChild<CityT>> children, double) const {
		std::sort(children.begin(), children.end(), [](const auto& a, const auto& b) {
			return a.score < b.score || (a.score == b.score && a.city < b.city);
		});
	}
};

// ---------------------------------------------------------------------------
// Solver::branch_and_bound
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
template <typename Bound>
auto Solver<Dist, Variant>::branch_and_bound(Bound bound, BranchAndBoundParams params) -> Solver& {
	return branch_and_bound(std::move(bound), CheapestFirst{}, params);
}

template <DistanceSource Dist, typename Variant>
template <typename Bound, typename Order>
	requires (!std::same_as<Order, BranchAndBoundParams>)
auto Solver<Dist, Variant>::branch_and_bound(Bound bound, Order order, BranchAndBoundParams params) -> Solver& {
	return detail::with_deadline(params.time_limit, [&](auto& deadline) -> Solver& {
		return branch_and_bound_run(bound, order, params, deadline);
	});
}

template <DistanceSource Dist, typename Variant>
template <typename Bound, typename Order, typename D>
auto Solver<Dist, Variant>::branch_and_bound_run(
	Bound& bound, Order& order, const BranchAndBoundParams& params, D& deadline)
	-> Solver&
{
	detail::Poller stop(deadline);
	[[maybe_unused]] detail::Poller stop_bound(deadline);  // not the nodes': a bound's steps cost far less
	constexpr bool takes_stop = detail::lower_bound_with<Bound, Solver, decltype(stop_bound)>;
	static_assert(takes_stop || detail::lower_bound_with<Bound, Solver>,
		"branch_and_bound: the strategy needs lower_bound(const Solver&, double cutoff) returning a double");
	using Child = BranchChild<city_type>;
	[[maybe_unused]] detail::Poller stop_order(deadline);  // nor the bound's: an order's units cost differently again
	constexpr bool order_takes_stop = detail::order_with<Order, Solver, Child, decltype(stop_order)>;
	constexpr bool orders = order_takes_stop || detail::order_with<Order, Solver, Child>;
	constexpr bool pick_takes_stop = detail::pick_with<Order, Solver, Child, decltype(stop_order)>;
	constexpr bool picks = pick_takes_stop || detail::pick_with<Order, Solver, Child>;
	static_assert(orders || picks,
		"branch_and_bound: the order needs order(const Solver&, std::span<BranchChild>, double cutoff) "
		"or pick(const Solver&, std::span<const BranchChild>, double cutoff) returning a std::size_t");

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

	// What the rest of the tour must cost less than, to beat the best tour.
	auto current_cutoff = [&] {
		return found ? static_cast<double>(best_cost) - static_cast<double>(cost_)
		             : std::numeric_limits<double>::infinity();
	};

	auto lower_bound = [&](double cutoff) -> double {
		if constexpr (takes_stop) return bound.lower_bound(*this, cutoff, stop_bound);
		else return bound.lower_bound(*this, cutoff);
	};

	auto check_penalty = [&] {
		if constexpr (detail::assumes_nonnegative_penalties<Bound>)
			assert(ctx_.cost_delta >= 0
				&& "branch_and_bound: move_prepare set a negative cost_delta, which the bound assumes away (pass a bound valid for this variant)");
	};

#ifndef NDEBUG
	// An order may only permute the children it was given (ws.scored, by city).
	auto permutes_scored = [&](std::span<const Child> children) {
		ws.ordered.assign(children.begin(), children.end());
		std::sort(ws.ordered.begin(), ws.ordered.end(), [](const Child& a, const Child& b) { return a.city < b.city; });
		return std::equal(ws.scored.begin(), ws.scored.end(), ws.ordered.begin(), ws.ordered.end(),
		                  [](const Child& a, const Child& b) { return a.city == b.city && a.score == b.score; });
	};
#endif

	// Scores the children of the current node through the pipeline, then orders them.
	auto expand = [&]([[maybe_unused]] double cutoff) {
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
		if constexpr (orders) {
			// CheapestFirst only sorts, so it goes unchecked and debug builds run the default at full speed.
			constexpr bool checked = !std::is_same_v<Order, CheapestFirst>;
#ifndef NDEBUG
			if constexpr (checked) ws.scored.assign(kids.begin(), kids.end());
#endif
			if constexpr (order_takes_stop) order.order(*this, std::span<Child>(kids), cutoff, stop_order);
			else order.order(*this, std::span<Child>(kids), cutoff);
			if constexpr (checked)
				assert(permutes_scored(kids) && "branch_and_bound: order must permute the children, not change them");
		}
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
		expand(current_cutoff());
		for (;;) {
			const std::size_t d = n_;
			if (ws.next[d] == ws.children[d].size()) {
				if (d == root) return true;
				pop_back(ws.cost[d - 1]);
				continue;
			}
			if (nodes == max_nodes || stop()) return false;
			if constexpr (picks) {
				auto& kids = ws.children[d];
				const auto remaining = std::span<const Child>(kids).subspan(ws.next[d]);
				const double cutoff = current_cutoff();
				std::size_t j = 0;
				if constexpr (pick_takes_stop) j = order.pick(*this, remaining, cutoff, stop_order);
				else j = order.pick(*this, remaining, cutoff);
				assert(j < remaining.size() && "branch_and_bound: pick must return an index into the remaining children");
				// The picked child comes next, the others keep their order.
				const auto first = kids.begin() + static_cast<std::ptrdiff_t>(ws.next[d]);
				const auto picked = first + static_cast<std::ptrdiff_t>(j);
				std::rotate(first, picked, picked + 1);
			}
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
			const double cutoff = current_cutoff();
			if (lower_bound(cutoff) >= cutoff) {
				pop_back(ws.cost[d]);
				continue;
			}
			expand(cutoff);
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

} // namespace periple
