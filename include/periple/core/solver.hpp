#pragma once

#include <periple/core/dimensions.hpp>
#include <periple/core/log.hpp>
#include <periple/core/caches/hk_cache.hpp>
#include <periple/core/moves/two_opt_move.hpp>
#include <periple/core/active_queue.hpp>
#include <periple/core/local_search_mode.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>

namespace periple {

// ---------------------------------------------------------------------------
// Parameter structs
// ---------------------------------------------------------------------------

struct NearestNeighborParams {
	std::size_t start_city = 0;
	std::size_t resume_at = 0;
};

struct HeldKarpParams {};

struct ConstructParams {
	std::size_t start_city = 0;
	std::size_t resume_at = 0;
};

struct TwoOptParams {
	std::size_t neighbors = 10;  // candidate list size per city (0 = all cities)
	std::size_t max_moves = 0;   // stop after this many applied moves (0 = no limit)
};

// ---------------------------------------------------------------------------
// SolutionStatus
// ---------------------------------------------------------------------------

enum class SolutionStatus { none, partial, feasible, optimal, infeasible };

// Tag type to skip debug symmetry checks in set_symmetric().
struct unchecked_t {};
inline constexpr unchecked_t unchecked{};

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant = NoCallbacks>
class Solver {
public:
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;
	using context_type = typename context_for<Variant, city_type, cost_type>::type;

	static constexpr bool has_callbacks = !std::is_same_v<Variant, NoCallbacks>;

	// Every tour is costed by appending, so AppendMove is required everywhere.
	static_assert(detail::covers_move<Variant, AppendMove<city_type>, context_type, city_type>::value,
		"variant has callbacks but none for AppendMove: every tour is costed by appending, so it would be ignored");
	static_assert(context_type::template dims_handle<AppendMove<city_type>, Dist>,
		"a dimension has no AppendMove init or commit, so its state would not follow the tour");

	// --- Lifecycle ----------------------------------------------------------

	Solver() = default;
	explicit Solver(const Dist& dist);
	Solver(const Dist& dist, const Variant& v);
	void set_matrix(const Dist& dist);
	void set_variant(const Variant& v);
	void clear();  // Clears solution state but keeps the distance source and buffers.
	void reset();  // Resets the solver to its default-constructed state.

	// --- State reading ------------------------------------------------------

	[[nodiscard]] auto status()    const -> SolutionStatus;
	[[nodiscard]] auto size()      const -> std::size_t;
	[[nodiscard]] auto tour()      const -> std::span<const city_type>;
	[[nodiscard]] auto position()  const -> std::span<const city_type>;
	[[nodiscard]] auto cost()      const -> cost_type;
	[[nodiscard]] auto symmetric() const -> bool;
	[[nodiscard]] auto is_visited(city_type city) const -> bool;
	[[nodiscard]] auto distance(city_type i, city_type j) const -> cost_type;
	void set_tour(std::span<const city_type> tour);
	void set_tour(std::span<const city_type> tour, cost_type known_cost);

	// Rotates a complete tour so that city comes first, at equal cost, O(n).
	// Only without a variant: rotating changes the traversal order, hence
	// arrival times and penalties. Useful after two_opt() in symmetric mode,
	// which treats the tour as a cycle and does not preserve its first city.
	void rotate_to_front(city_type city);
	void set_symmetric(bool sym);
	void set_symmetric(bool sym, unchecked_t);

	// --- Neighbor lists -----------------------------------------------------

	// Returns the k nearest neighbors of city, sorted by distance.
	// Computed lazily on first call; grows if k exceeds previous requests.
	[[nodiscard]] auto neighbors(city_type city, std::size_t k) -> std::span<const city_type>;

	// --- Evaluation ---------------------------------------------------------

	// Scores appending city at the end of the tour: init + prepare + filter.
	// Returns the cost of the added edge, distance plus penalty, and not a tour
	// cost: a partial tour has none, and an untruncated double keeps candidates
	// comparable when penalties are fractional. Nullopt if move_filter rejects.
	// Does not modify observable solver state (writes to mutable ctx_).
	[[nodiscard]] auto evaluate_append(city_type city) const -> std::optional<double>;

	// Evaluates a proposed tour by replaying AppendMove from from_pos onward.
	// Returns the total tour cost if feasible, or nullopt if any move_filter
	// rejects. Uses staged dimension buffers (non-destructive).
	template <typename CityFn>
	    requires std::invocable<CityFn&, std::size_t>
	[[nodiscard]] auto evaluate_replay(CityFn&& city_at, std::size_t from_pos,
	                                   cost_type prefix_cost) const -> std::optional<cost_type>;

	[[nodiscard]] auto evaluate_replay(std::span<const city_type> proposed, std::size_t from_pos, cost_type prefix_cost) const
	    -> std::optional<cost_type>;

	// Total cost of the tour obtained by reversing tour[i+1..j], replayed
	// through the variant pipeline. O(n - i), left staged for accept_reversal.
	// Requires a variant (uses prefix_cost).
	[[nodiscard]] auto evaluate_reversal(std::size_t i, std::size_t j) const
	    -> std::optional<cost_type>;

	// Applies a scored reversal: commits the staged dimensions, then reverses
	// in place, so nothing reads the tour while it is being written. O(j - i).
	// Pairs with evaluate_reversal, or with a cost the caller derived from
	// path_cost when there is no variant to stage.
	void accept_reversal(std::size_t i, std::size_t j, cost_type new_cost);

	// Committed cost of tour[0..pos] (edges + variant penalties), O(1).
	// Requires a variant (CumulativeCost dimension).
	[[nodiscard]] auto prefix_cost(std::size_t pos) const -> cost_type;

	// Raw distance along the tour between two positions, following tour order
	// when from <= to and going against it otherwise. O(1), after a lazy O(n)
	// build. Penalties are excluded; prefix_cost is the variant-aware
	// counterpart.
	[[nodiscard]] auto path_cost(std::size_t from, std::size_t to) -> cost_type;

	// Staging lifecycle for local search.
	void save_staging() const;

	// Drops any pending replay evaluation (staged dimension state).
	void discard_staging() const;

	// Applies a tour scored by evaluate_replay. city_at must not read the
	// solver's own tour, which this overwrites as it goes; evaluate_replay
	// only reads, so the same lambda is safe there. For a reversal, use
	// evaluate_reversal and accept_reversal.
	template <typename CityFn>
	void accept_replay(CityFn&& city_at, std::size_t from_pos, cost_type new_cost);

	// --- Construction primitives --------------------------------------------

	auto append(city_type city) -> Solver&;
	[[nodiscard]] auto can_append(city_type city) const -> bool;

	// --- Algorithms (defined inline in algorithms/*.hpp) --------------------

	auto nearest_neighbor(NearestNeighborParams params = {}) -> Solver&;

	auto held_karp(HeldKarpParams params = {}) -> Solver&;

	template <typename Strategy>
	auto greedy_construct(Strategy strategy,
	                      ConstructParams params = {}) -> Solver&;

	auto two_opt(TwoOptParams params = {}) -> Solver&;

#ifdef PERIPLE_TESTING
	template <DistanceSource D, typename V> friend class SolverTestAccess;
#endif

private:
	auto try_trivial() -> bool;
	void ensure_capacity(std::size_t n);
	void ensure_neighbors(std::size_t k);
	void rebuild_position();
	void invalidate_caches();
	void check_symmetry() const;
	auto rebuild_and_cost(std::span<const city_type> t) -> cost_type;
	void close_tour();

	// Local search primitives (shared by improvement algorithms)
	void ensure_path_costs();
	void rebuild_path_costs(std::size_t from);
	void reverse_range(std::size_t lo, std::size_t hi);
	void reverse_cyclic(std::size_t from, std::size_t len);

	template <detail::LocalSearchMode Mode>
	void two_opt_run(const TwoOptParams& params);

	template <detail::LocalSearchMode Mode>
	auto two_opt_improve(city_type a, std::size_t k, detail::ActiveQueue<city_type>& active) -> bool;

	auto variant_ref() const -> const Variant& {
		if constexpr (std::is_same_v<Variant, NoCallbacks>) {
			static constexpr NoCallbacks fallback{};
			return fallback;
		} else {
			assert(variant_ && "variant_ref: no variant set on solver");
			return *variant_;
		}
	}

	// Adjusted edge cost: raw distance + variant's cost_delta (set by prepare).
	auto edge_delta(city_type from, city_type to) const -> double {
		return static_cast<double>((*dist_)(from, to)) + ctx_.cost_delta;
	}

	// State
	const Dist*    dist_       = nullptr;
	const Variant* variant_    = nullptr;
	SolutionStatus status_     = SolutionStatus::none;
	std::size_t    n_          = 0;
	bool           symmetric_  = false;
	std::vector<city_type> tour_;
	cost_type      cost_       = {};
	std::size_t    tour_version_ = 0;  // Incremented by every write to tour_ or n_.

	// Workspace (grow-only)
	std::vector<city_type> position_;  // Inverse index: city -> position in tour.
	std::vector<uint8_t>   visited_;   // visited_[c] == (c is in tour); maintained like position_.
	std::vector<uint8_t>   dont_look_;
	std::vector<city_type> queue_;     // ActiveQueue slots.
	std::vector<city_type> neighbors_;
	std::size_t            neighbors_k_ = 0;

	// Tour-derived, valid iff path_costs_version_ == tour_version_.
	std::vector<cost_type> path_fwd_, path_bwd_;
	std::size_t            path_costs_version_ = 0;

	// Evaluation context (mutable: the evaluate_* methods are const)
	mutable context_type ctx_;

	// Algorithm caches (lazy, one per algorithm that needs persistent state)
	std::optional<HKCache<cost_type, city_type>> hk_cache_;
};

// ---------------------------------------------------------------------------
// Inline implementations -- lifecycle & state
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
Solver<Dist, Variant>::Solver(const Dist& dist) {
	set_matrix(dist);
}

template <DistanceSource Dist, typename Variant>
Solver<Dist, Variant>::Solver(const Dist& dist, const Variant& v)
	: variant_(&v)
{
	set_matrix(dist);
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::set_matrix(const Dist& dist) {
	dist_   = &dist;
	status_ = SolutionStatus::none;
	n_      = 0;
	cost_   = {};
	++tour_version_;
	ctx_.reset();
	invalidate_caches();
	if (symmetric_) check_symmetry();
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::set_variant(const Variant& v) {
	variant_ = &v;
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::clear() {
	status_ = SolutionStatus::none;
	n_      = 0;
	cost_   = {};
	++tour_version_;
	ctx_.reset();
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::reset() {
	*this = Solver();
}

// --- State reading ----------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::status() const -> SolutionStatus {
	return status_;
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::size() const -> std::size_t {
	return dist_ ? dist_->size() : 0;
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::tour() const -> std::span<const city_type> {
	return {tour_.data(), n_};
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::position() const -> std::span<const city_type> {
	return {position_.data(), n_};
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::cost() const -> cost_type {
	return cost_;
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::symmetric() const -> bool {
	return symmetric_;
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::set_symmetric(bool sym) {
	symmetric_ = sym;
	if (sym && dist_) check_symmetry();
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::set_symmetric(bool sym, unchecked_t) {
	symmetric_ = sym;
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::is_visited(city_type city) const -> bool {
	return n_ > 0 && visited_[static_cast<std::size_t>(city)];
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::distance(city_type i, city_type j) const -> cost_type {
	assert(dist_ && "distance: no distance source set");
	return (*dist_)(i, j);
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::set_tour(std::span<const city_type> t) {
	assert(dist_ && "set_tour: no distance source set");
	const auto total = dist_->size();
	assert(t.size() <= total && "set_tour: tour exceeds matrix size");

	assert([&] {
		for (std::size_t i = 0; i < t.size(); ++i) {
			if (static_cast<std::size_t>(t[i]) >= total) return false;
			for (std::size_t j = i + 1; j < t.size(); ++j)
				if (t[i] == t[j]) return false;
		}
		return true;
	}() && "set_tour: invalid tour (duplicate or out-of-bounds city)");

	ensure_capacity(total);
	n_ = t.size();
	std::copy(t.begin(), t.end(), tour_.begin());
	++tour_version_;

	if (n_ > 0) {
		cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
		if (n_ == total) {
			status_ = SolutionStatus::feasible;
			std::fill_n(visited_.data(), total, uint8_t{1});
		} else {
			status_ = SolutionStatus::partial;
			std::fill_n(visited_.data(), total, uint8_t{0});
			for (std::size_t i = 0; i < n_; ++i)
				visited_[static_cast<std::size_t>(t[i])] = 1;
		}
	} else {
		cost_   = {};
		status_ = SolutionStatus::none;
		ctx_.reset();
	}
}

// --- Internal helpers -------------------------------------------------------

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::ensure_capacity(std::size_t n) {
	if (tour_.size()     < n) tour_.resize(n);
	if (position_.size() < n) position_.resize(n);
	if (visited_.size()  < n) visited_.resize(n);
	ctx_.resize(n);
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::rebuild_position() {
	if (position_.size() < n_) position_.resize(n_);
	for (std::size_t i = 0; i < n_; ++i)
		position_[static_cast<std::size_t>(tour_[i])] =
			static_cast<city_type>(i);
}

// Reverses tour_[lo..hi] in place and refreshes position_ for that range.
template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::reverse_range(std::size_t lo, std::size_t hi) {
	std::reverse(tour_.begin() + static_cast<std::ptrdiff_t>(lo),
	             tour_.begin() + static_cast<std::ptrdiff_t>(hi) + 1);
	for (std::size_t p = lo; p <= hi; ++p)
		position_[static_cast<std::size_t>(tour_[p])] = static_cast<city_type>(p);
	++tour_version_;
}

// Reverses the cyclic segment of len positions starting at from, wrapping past
// the end of the tour.
template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::reverse_cyclic(std::size_t from, std::size_t len) {
	std::size_t lo = from;
	std::size_t hi = from + len - 1;
	if (hi >= n_) hi -= n_;
	for (std::size_t s = 0; s < len / 2; ++s) {
		std::swap(tour_[lo], tour_[hi]);
		position_[static_cast<std::size_t>(tour_[lo])] = static_cast<city_type>(lo);
		position_[static_cast<std::size_t>(tour_[hi])] = static_cast<city_type>(hi);
		lo = (lo + 1 == n_) ? 0 : lo + 1;
		hi = (hi == 0) ? n_ - 1 : hi - 1;
	}
	++tour_version_;
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::rotate_to_front(city_type city) {
	static_assert(!has_callbacks,
		"rotate_to_front: only without a variant (rotating changes the traversal order)");
	assert((status_ == SolutionStatus::feasible || status_ == SolutionStatus::optimal)
		&& "rotate_to_front: requires a complete tour");
	assert(static_cast<std::size_t>(city) < dist_->size()
		&& "rotate_to_front: city index out of bounds");

	const auto p = static_cast<std::size_t>(position_[static_cast<std::size_t>(city)]);
	if (p == 0) return;
	std::rotate(tour_.begin(), tour_.begin() + static_cast<std::ptrdiff_t>(p),
	            tour_.begin() + static_cast<std::ptrdiff_t>(n_));
	rebuild_position();
	++tour_version_;
}

// Prefix costs of the tour path in both directions:
//   path_fwd_[q] = sum of d(t[p], t[p+1]) for p < q
//   path_bwd_[q] = sum of d(t[p+1], t[p]) for p < q
// They give the cost of any sub-path in O(1), in either direction, which is
// what an asymmetric segment reversal needs. The closing edge is never inside
// a reversed segment and is not included.
template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::ensure_path_costs() {
	assert(n_ > 0 && "ensure_path_costs: empty tour");
	if (path_costs_version_ == tour_version_) return;
	if (path_fwd_.size() < n_) {
		path_fwd_.resize(n_);
		path_bwd_.resize(n_);
	}
	path_fwd_[0] = cost_type{};
	path_bwd_[0] = cost_type{};
	rebuild_path_costs(0);
}

// Recomputes entries after `from`, which must still be valid, and marks the
// path costs as current.
template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::rebuild_path_costs(std::size_t from) {
	const auto& dist = *dist_;
	for (std::size_t q = from; q + 1 < n_; ++q) {
		path_fwd_[q + 1] = path_fwd_[q] + dist(tour_[q], tour_[q + 1]);
		path_bwd_[q + 1] = path_bwd_[q] + dist(tour_[q + 1], tour_[q]);
	}
	path_costs_version_ = tour_version_;
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::invalidate_caches() {
	if (hk_cache_) hk_cache_->reset();
	neighbors_k_ = 0;
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::check_symmetry() const {
	assert([&] {
		const auto n = dist_->size();
		constexpr std::size_t max_print = 5;
		std::size_t count = 0;
		for (std::size_t i = 0; i < n; ++i)
			for (std::size_t j = i + 1; j < n; ++j) {
				auto dij = (*dist_)(static_cast<city_type>(i),
				                    static_cast<city_type>(j));
				auto dji = (*dist_)(static_cast<city_type>(j),
				                    static_cast<city_type>(i));
				if (dij != dji) {
					if (count == 0)
						detail::log(
							"set_symmetric(true) but distance"
							" matrix has violations:\n");
					if (count < max_print)
						detail::log(
							"  d(%zu, %zu) = %g,"
							" d(%zu, %zu) = %g\n",
							i, j, static_cast<double>(dij),
							j, i, static_cast<double>(dji));
					++count;
				}
			}
		if (count > max_print)
			detail::log("  ... and %zu more\n", count - max_print);
		return count == 0;
	}());
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::neighbors(city_type city, std::size_t k) -> std::span<const city_type> {
	assert(dist_ && "neighbors: no distance source set");
	assert(k < dist_->size() && "neighbors: k must be less than number of cities");
	ensure_neighbors(k);
	return {neighbors_.data() + static_cast<std::size_t>(city) * neighbors_k_, k};
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::ensure_neighbors(std::size_t k) {
	if (k <= neighbors_k_) return;

	const auto n = dist_->size();
	neighbors_k_ = k;
	neighbors_.resize(n * k);

	std::vector<city_type> candidates(n - 1);

	for (std::size_t i = 0; i < n; ++i) {
		std::size_t idx = 0;
		for (std::size_t j = 0; j < n; ++j)
			if (j != i) candidates[idx++] = static_cast<city_type>(j);

		std::partial_sort(candidates.begin(),
		                  candidates.begin() + static_cast<std::ptrdiff_t>(k),
		                  candidates.end(),
			[&](city_type a, city_type b) {
				return (*dist_)(static_cast<city_type>(i), a) <
				       (*dist_)(static_cast<city_type>(i), b);
			});

		std::copy_n(candidates.begin(), k,
		            neighbors_.begin() +
		            static_cast<std::ptrdiff_t>(i * k));
	}
}

// --- rebuild_and_cost -------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::rebuild_and_cost(std::span<const city_type> t) -> cost_type {
	ctx_.reset();
	cost_type total_cost{};

	for (std::size_t i = 0; i < t.size(); ++i) {
		AppendMove<city_type> move{t[i], (i > 0 ? t[i - 1] : city_type{}), i};
		ctx_.init(move, *dist_, {t.data(), i}, {position_.data(), i}, total_cost);
		invoke_prepare(variant_ref(), move, ctx_);

		if (i > 0)
			total_cost += static_cast<cost_type>(edge_delta(t[i - 1], t[i]));

		ctx_.commit(move);
		position_[static_cast<std::size_t>(t[i])] = static_cast<city_type>(i);
	}

	// Closing edge (complete tour only).
	if (t.size() == dist_->size() && t.size() > 1) {
		AppendMove<city_type> closing{t[0], t.back(), t.size()};
		ctx_.init(closing, *dist_, {t.data(), t.size()}, {position_.data(), t.size()}, total_cost);
		invoke_prepare(variant_ref(), closing, ctx_);
		total_cost += static_cast<cost_type>(edge_delta(t.back(), t[0]));
	}

	return total_cost;
}

// --- close_tour -------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::close_tour() {
	if (n_ <= 1) {
		status_ = SolutionStatus::feasible;
		return;
	}
	AppendMove<city_type> closing{tour_[0], tour_[n_ - 1], n_};
	ctx_.init(closing, *dist_, {tour_.data(), n_}, {position_.data(), n_}, cost_);
	invoke_prepare(variant_ref(), closing, ctx_);
	if (!invoke_filter(variant_ref(), closing, ctx_)) {
		status_ = SolutionStatus::infeasible;
		return;
	}
	cost_ += static_cast<cost_type>(edge_delta(tour_[n_ - 1], tour_[0]));
	status_ = SolutionStatus::feasible;
}

// --- Evaluation -------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::evaluate_append(city_type city) const -> std::optional<double> {
	assert(dist_ && "evaluate_append: no distance source set");
	assert(n_ > 0 && "evaluate_append: tour must have at least one city");

	AppendMove<city_type> move{city, tour_[n_ - 1], n_};
	ctx_.init(move, *dist_, {tour_.data(), n_}, {position_.data(), n_}, cost_);
	invoke_prepare(variant_ref(), move, ctx_);
	if (!invoke_filter(variant_ref(), move, ctx_))
		return std::nullopt;

	return edge_delta(tour_[n_ - 1], city);
}

// --- Construction primitives ------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::append(city_type city) -> Solver& {
	assert(dist_ && "append: no distance source set");
	assert((status_ == SolutionStatus::none || status_ == SolutionStatus::partial) && "append: solver must be in none or partial state");
	const auto total = dist_->size();
	assert(static_cast<std::size_t>(city) < total && "append: city index out of bounds");
	ensure_capacity(total);

	if (n_ == 0)
		std::fill_n(visited_.data(), total, uint8_t{0});

	AppendMove<city_type> move{city, (n_ > 0 ? tour_[n_ - 1] : city_type{}), n_};
	ctx_.init(move, *dist_, {tour_.data(), n_}, {position_.data(), n_}, cost_);
	invoke_prepare(variant_ref(), move, ctx_);

	if (n_ > 0)
		cost_ += static_cast<cost_type>(edge_delta(tour_[n_ - 1], city));

	tour_[n_] = city;
	visited_[static_cast<std::size_t>(city)] = 1;
	position_[static_cast<std::size_t>(city)] = static_cast<city_type>(n_);
	++n_;
	++tour_version_;

	ctx_.commit(move);

	if (n_ == total) {
		close_tour();
	} else {
		status_ = SolutionStatus::partial;
	}

	return *this;
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::can_append(city_type city) const -> bool {
	return evaluate_append(city).has_value();
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::try_trivial() -> bool {
	assert(dist_ && "try_trivial: no distance source set");
	const auto total = dist_->size();
	if (total == 0) {
		clear();
		status_ = SolutionStatus::feasible;
		return true;
	}
	if (total == 1) {
		clear();
		append(static_cast<city_type>(0));
		return true;
	}
	return false;
}

// --- Replay-based evaluation ------------------------------------------------

template <DistanceSource Dist, typename Variant>
template <typename CityFn>
    requires std::invocable<CityFn&, std::size_t>
auto Solver<Dist, Variant>::evaluate_replay(
    CityFn&& city_at, std::size_t from_pos,
    cost_type prefix_cost) const -> std::optional<cost_type>
{
	static_assert(context_type::replay_safe,
		"evaluate_replay: a dimension commits AppendMove state without staging (begin_staging, discard_staging, save_staging, commit_staging), so a rejected candidate would overwrite the committed tour's state");
	assert(dist_ && "evaluate_replay: no distance source set");
	assert(n_ == dist_->size() && "evaluate_replay: solver must have a complete tour");
	assert(from_pos >= 1 && from_pos < n_ && "evaluate_replay: from_pos out of range");

	ctx_.begin_staging(from_pos);

	const auto& variant = variant_ref();
	cost_type running_cost = prefix_cost;

	auto prev = city_at(from_pos - 1);
	for (std::size_t i = from_pos; i < n_; ++i) {
		auto city = city_at(i);
		AppendMove<city_type> move{city, prev, i};
		ctx_.init(move, *dist_, running_cost);
		invoke_prepare(variant, move, ctx_);
		if (!invoke_filter(variant, move, ctx_))
			return std::nullopt;
		running_cost += static_cast<cost_type>(edge_delta(prev, city));
		ctx_.commit(move);
		prev = city;
	}

	// Closing edge.
	auto first = city_at(0);
	AppendMove<city_type> closing{first, prev, n_};
	ctx_.init(closing, *dist_, running_cost);
	invoke_prepare(variant, closing, ctx_);
	if (!invoke_filter(variant, closing, ctx_))
		return std::nullopt;
	running_cost += static_cast<cost_type>(edge_delta(prev, first));

	return running_cost;
}

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::evaluate_replay(
    std::span<const city_type> proposed, std::size_t from_pos,
    cost_type prefix_cost) const -> std::optional<cost_type>
{
	assert(proposed.size() == n_ && "evaluate_replay: proposed tour size must match");
	return evaluate_replay([&](std::size_t i) { return proposed[i]; }, from_pos, prefix_cost);
}

// --- Segment reversal -------------------------------------------------------

// The reversal reads positions i+1..j backwards; position 0 stays put, so the
// prefix up to i is untouched and prefix_cost(i) is the exact starting cost.
template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::evaluate_reversal(std::size_t i, std::size_t j) const
    -> std::optional<cost_type>
{
	assert(i < j && j < n_ && "evaluate_reversal: positions out of range");
	const TwoOptMove<city_type> move{i, j};
	return evaluate_replay(move.city_at(tour_), i + 1, prefix_cost(i));
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::accept_reversal(std::size_t i, std::size_t j, cost_type new_cost) {
	assert(i < j && j < n_ && "accept_reversal: positions out of range");
	ctx_.commit_staging(n_);
	reverse_range(i + 1, j);
	cost_ = new_cost;
}

// --- Prefix cost ------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::prefix_cost(std::size_t pos) const -> cost_type {
	static_assert(context_type::template has_dim<CumulativeCost>,
		"prefix_cost: requires a variant (CumulativeCost dimension)");
	assert(pos < n_ && "prefix_cost: position out of range");
	return static_cast<cost_type>(
		ctx_.template dim<CumulativeCost>().costs.committed(pos));
}

// Walking up from `from` to `to` crosses the edges p in [from, to-1], which is
// the difference of the forward prefixes; walking down crosses p in [to,
// from-1] in the other direction, hence the backward ones.
template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::path_cost(std::size_t from, std::size_t to) -> cost_type {
	static_assert(!has_callbacks,
		"path_cost: raw distances only (use prefix_cost with a variant)");
	assert(from < n_ && to < n_ && "path_cost: position out of range");
	ensure_path_costs();
	return (from <= to) ? path_fwd_[to] - path_fwd_[from]
	                    : path_bwd_[from] - path_bwd_[to];
}

// --- Staging lifecycle ------------------------------------------------------

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::save_staging() const {
	ctx_.save_staging(n_);
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::discard_staging() const {
	ctx_.discard_staging();
}

template <DistanceSource Dist, typename Variant>
template <typename CityFn>
void Solver<Dist, Variant>::accept_replay(
    CityFn&& city_at, std::size_t from_pos, cost_type new_cost)
{
	ctx_.commit_staging(n_);

	for (std::size_t i = from_pos; i < n_; ++i)
		tour_[i] = city_at(i);
	for (std::size_t i = from_pos; i < n_; ++i)
		position_[static_cast<std::size_t>(tour_[i])] = static_cast<city_type>(i);
	cost_ = new_cost;
	++tour_version_;
}

// --- Tour import with known cost --------------------------------------------

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::set_tour(std::span<const city_type> t,
                                     cost_type known_cost)
{
	assert(dist_ && "set_tour: no distance source set");
	const auto total = dist_->size();
	assert(t.size() <= total && "set_tour: tour exceeds matrix size");

	assert([&] {
		for (std::size_t i = 0; i < t.size(); ++i) {
			if (static_cast<std::size_t>(t[i]) >= total) return false;
			for (std::size_t j = i + 1; j < t.size(); ++j)
				if (t[i] == t[j]) return false;
		}
		return true;
	}() && "set_tour: invalid tour (duplicate or out-of-bounds city)");

	ensure_capacity(total);
	n_ = t.size();
	std::copy(t.begin(), t.end(), tour_.begin());
	++tour_version_;

	// Rebuild dimensions (the cost is overridden below).
	if (n_ > 0)
		rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));

	cost_ = known_cost;

	if (n_ == total) {
		status_ = SolutionStatus::feasible;
		std::fill_n(visited_.data(), total, uint8_t{1});
	} else if (n_ > 0) {
		status_ = SolutionStatus::partial;
		std::fill_n(visited_.data(), total, uint8_t{0});
		for (std::size_t i = 0; i < n_; ++i)
			visited_[static_cast<std::size_t>(t[i])] = 1;
	} else {
		status_ = SolutionStatus::none;
	}
}

// --- CTAD guides ------------------------------------------------------------

template <DistanceSource Dist>
Solver(const Dist&) -> Solver<Dist>;

template <DistanceSource Dist, typename Variant>
Solver(const Dist&, const Variant&) -> Solver<Dist, Variant>;

} // namespace periple