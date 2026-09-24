#pragma once

// 2-opt local search
//
// Croes (1958), "A Method for Solving Traveling-Salesman Problems"
// Bentley (1992), "Fast Algorithms for Geometric Traveling Salesman Problems"
//   -- neighbor lists, don't-look bits, shorter-side reversal
//
// First-improvement search over neighbor lists, driven by a queue of active
// cities. Three evaluation modes, chosen once per call:
//   symmetric  -- NoCallbacks + set_symmetric(true). O(1) delta with the
//                 classic gain criterion. Moves may wrap around the tour start
//                 and the shorter side is reversed, which caps a move at n/2
//                 swaps but does not preserve the first city (cycle
//                 semantics); rotate_to_front() restores a chosen one.
//   asymmetric -- NoCallbacks, symmetry not declared. The inner edges of the
//                 reversed segment change direction; their cost in both
//                 directions comes from path_fwd_/path_bwd_, so the delta is
//                 still O(1), at the price of an O(n - i) prefix rebuild per
//                 applied move. Only segments not containing position 0.
//   replay     -- any variant. Candidates are evaluated by replaying the
//                 suffix through the variant pipeline, O(n - i) each. Same
//                 segment restriction; goes through the public
//                 evaluate_reversal / accept_reversal pair.
//
// In the two directed modes a reversal also flips the direction of the edges
// inside the segment, which the don't-look bits do not track (only the four
// endpoints are reactivated). Calling two_opt() again can therefore still
// improve the tour; in symmetric mode one call already reaches the fixed point
// of the candidate neighborhood.

#include <periple/core/solver.hpp>

#include <algorithm>
#include <limits>
#include <span>
#include <type_traits>

namespace periple {

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::two_opt(TwoOptParams params) -> Solver& {
	assert(dist_ && "two_opt: no distance source set");
	assert((status_ == SolutionStatus::feasible || status_ == SolutionStatus::optimal)
		&& "two_opt: requires a complete feasible tour (run a construction algorithm first)");

	if (status_ == SolutionStatus::optimal || n_ < 4) return *this;

	if constexpr (has_callbacks) {
		two_opt_run<detail::LocalSearchMode::replay>(params);
	} else {
		if (symmetric_) two_opt_run<detail::LocalSearchMode::symmetric>(params);
		else            two_opt_run<detail::LocalSearchMode::asymmetric>(params);
	}
	return *this;
}

template <DistanceSource Dist, typename Variant>
template <detail::LocalSearchMode Mode>
void Solver<Dist, Variant>::two_opt_run(const TwoOptParams& params) {
	const std::size_t n = n_;
	const std::size_t k = (params.neighbors == 0) ? n - 1
	                                              : std::min(params.neighbors, n - 1);
	ensure_neighbors(k);
	ensure_capacity(n);  // also clears any staging flag left in ctx_
	if (dont_look_.size() < n) dont_look_.resize(n);
	if (queue_.size()     < n) queue_.resize(n);

	if constexpr (Mode == detail::LocalSearchMode::asymmetric)
		ensure_path_costs();

	detail::ActiveQueue<city_type> active({queue_.data(), n}, {dont_look_.data(), n});
	active.reset({tour_.data(), n});

	const std::size_t max_moves = params.max_moves ? params.max_moves
	                                               : std::numeric_limits<std::size_t>::max();
	std::size_t moves = 0;

	while (!active.empty() && moves < max_moves) {
		const city_type a = active.pop();
		// Retried until no improving move remains, then deactivated.
		while (moves < max_moves && two_opt_improve<Mode>(a, k, active))
			++moves;
		active.deactivate(a);
	}

	if constexpr (Mode == detail::LocalSearchMode::replay)
		ctx_.discard_staging();

	// Delta updates accumulate rounding for floating-point costs; restore
	// exact agreement with a sequential recomputation.
	if constexpr (std::is_floating_point_v<cost_type>)
		cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n));
}

template <DistanceSource Dist, typename Variant>
template <detail::LocalSearchMode Mode>
auto Solver<Dist, Variant>::two_opt_improve(
	city_type a, std::size_t k, detail::ActiveQueue<city_type>& active) -> bool
{
	using detail::LocalSearchMode;
	const std::size_t n = n_;
	const auto& dist = *dist_;
	auto pos_of = [this](city_type c) {
		return static_cast<std::size_t>(position_[static_cast<std::size_t>(c)]);
	};
	auto succ = [n](std::size_t p) { return (p + 1 == n) ? 0 : p + 1; };
	auto pred = [n](std::size_t p) { return (p == 0) ? n - 1 : p - 1; };

	const std::size_t pa = pos_of(a);
	const std::span<const city_type> nbrs{
		neighbors_.data() + static_cast<std::size_t>(a) * neighbors_k_, k};

	if constexpr (Mode == LocalSearchMode::symmetric) {
		// Undirected move on edges {t[i], t[succ i]} and {t[j], t[succ j]}:
		// reversing t[succ i .. j] or t[succ j .. i] gives the same cycle.
		auto apply = [&](std::size_t i, std::size_t j) {
			const std::size_t len = (j >= i) ? j - i : j + n - i;  // |succ i .. j|
			if (len <= n - len) reverse_cyclic(succ(i), len);
			else                reverse_cyclic(succ(j), n - len);
		};

		// Successor side: remove {a, b = succ a}, {c, d = succ c}; add {a, c}, {b, d}.
		{
			const city_type b = tour_[succ(pa)];
			const cost_type d_ab = dist(a, b);
			for (city_type c : nbrs) {
				const cost_type d_ac = dist(a, c);
				if (d_ac >= d_ab) break;  // sorted candidates: no gain beyond this point
				const std::size_t pc = pos_of(c);
				const city_type d = tour_[succ(pc)];
				const cost_type added   = d_ac + dist(b, d);
				const cost_type removed = d_ab + dist(c, d);
				if (added < removed) {
					apply(pa, pc);
					cost_ += added;
					cost_ -= removed;
					active.push(b); active.push(c); active.push(d);
					return true;
				}
			}
		}
		// Predecessor side: remove {b = pred a, a}, {d = pred c, c}; add {a, c}, {b, d}.
		{
			const std::size_t pb = pred(pa);
			const city_type b = tour_[pb];
			const cost_type d_ab = dist(b, a);
			for (city_type c : nbrs) {
				const cost_type d_ac = dist(a, c);
				if (d_ac >= d_ab) break;
				const std::size_t pd = pred(pos_of(c));
				const city_type d = tour_[pd];
				const cost_type added   = d_ac + dist(d, b);
				const cost_type removed = d_ab + dist(d, c);
				if (added < removed) {
					apply(pb, pd);  // reverses t[a .. pred c]
					cost_ += added;
					cost_ -= removed;
					active.push(b); active.push(c); active.push(d);
					return true;
				}
			}
		}
		return false;
	} else {
		// Directed move: reverse t[i+1 .. j] with 0 <= i < j <= n-1, creating
		// t[i] -> t[j] and t[i+1] -> t[succ j]. Position 0 is never inside the
		// reversed segment, so the tour start is fixed.
		auto try_move = [&](std::size_t i, std::size_t j) -> bool {
			const city_type ti  = tour_[i];
			const city_type ti1 = tour_[i + 1];
			const city_type tj  = tour_[j];
			const city_type tj1 = tour_[succ(j)];
			if constexpr (Mode == LocalSearchMode::asymmetric) {
				// Boundary edges, plus the inner edges t[i+1..j] which flip
				// direction: they cost path_fwd_[j] - path_fwd_[i+1] now and
				// path_bwd_[j] - path_bwd_[i+1] after the move. Read here
				// rather than through the public path_cost(from, to), which
				// would add its version check to this O(1) loop.
				const cost_type added   = dist(ti, tj) + dist(ti1, tj1)
				                        + (path_bwd_[j] - path_bwd_[i + 1]);
				const cost_type removed = dist(ti, ti1) + dist(tj, tj1)
				                        + (path_fwd_[j] - path_fwd_[i + 1]);
				if (!(added < removed)) return false;
				reverse_range(i + 1, j);
				rebuild_path_costs(i);
				cost_ += added;
				cost_ -= removed;
			} else {
				const auto new_cost = evaluate_reversal(i, j);
				if (!new_cost || !(*new_cost < cost_)) return false;
				accept_reversal(i, j, *new_cost);
			}
			active.push(ti); active.push(ti1); active.push(tj); active.push(tj1);
			return true;
		};

		// No gain-criterion pruning here: inner edges (asymmetric) and
		// penalties (replay) can make a move improving whatever d(a, c) is.
		for (city_type c : nbrs) {
			const std::size_t pc = pos_of(c);
			if (pa >= pc) continue;  // edge a -> c: found when scanning from c
			// Successor side: a = t[i], c = t[j].
			if (try_move(pa, pc)) return true;
			// Predecessor side: a = t[i+1], c = t[j+1].
			if (pa >= 1 && try_move(pa - 1, pc - 1)) return true;
		}
		return false;
	}
}

} // namespace periple
