#pragma once

#include <periple/core/dimensions.hpp>
#include <periple/core/log.hpp>
#include <periple/core/caches/hk_cache.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>

namespace periple {

// ---------------------------------------------------------------------------
// Tags
// ---------------------------------------------------------------------------

struct NoCallbacks {};

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
	void set_symmetric(bool sym);
	void set_symmetric(bool sym, unchecked_t);

	// --- Neighbor lists -----------------------------------------------------

	// Returns the k nearest neighbors of city, sorted by distance.
	// Computed lazily on first call; grows if k exceeds previous requests.
	[[nodiscard]] auto neighbors(city_type city, std::size_t k) -> std::span<const city_type>;

	// --- Evaluation ---------------------------------------------------------

	// Tentative evaluation: init + prepare + filter -> score.
	// Returns nullopt if the variant's move_filter rejects the city.
	// Does not modify observable solver state (writes to mutable ctx_).
	[[nodiscard]] auto evaluate(city_type city) const -> std::optional<double>;

	// --- Construction primitives --------------------------------------------

	auto append(city_type city) -> Solver&;
	[[nodiscard]] auto can_append(city_type city) const -> bool;

	// --- Algorithms (defined inline in algorithms/*.hpp) --------------------

	auto nearest_neighbor(NearestNeighborParams params = {}) -> Solver&;

	auto held_karp(HeldKarpParams params = {}) -> Solver&;

	template <typename Strategy>
	auto greedy_construct(Strategy strategy,
	                      ConstructParams params = {}) -> Solver&;

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

	// Workspace (grow-only)
	std::vector<city_type> position_;  // Inverse index: city -> position in tour.
	std::vector<uint8_t>   visited_;
	std::vector<uint8_t>   dont_look_;
	std::vector<city_type> neighbors_;
	std::size_t            neighbors_k_ = 0;
	std::vector<cost_type> cumul_costs_;  // Cumulative cost at each tour position.

	// Evaluation context (mutable: evaluate() is const)
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

	if (n_ > 0) {
		cost_   = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
		status_ = (n_ == total) ? SolutionStatus::feasible : SolutionStatus::partial;
		if (n_ < total) {
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
	if (cumul_costs_.size() < n) cumul_costs_.resize(n);
	ctx_.resize(n);
}

template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::rebuild_position() {
	if (position_.size() < n_) position_.resize(n_);
	for (std::size_t i = 0; i < n_; ++i)
		position_[static_cast<std::size_t>(tour_[i])] =
			static_cast<city_type>(i);
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
		AppendMove<city_type> move{t[i]};
		ctx_.init(move, *dist_, {t.data(), i}, {position_.data(), i}, total_cost);
		invoke_prepare(variant_ref(), move, ctx_);

		if (i > 0)
			total_cost += static_cast<cost_type>(edge_delta(t[i - 1], t[i]));

		ctx_.commit(move);
		position_[static_cast<std::size_t>(t[i])] = static_cast<city_type>(i);
		cumul_costs_[i] = total_cost;
	}

	// Closing edge (complete tour only).
	if (t.size() == dist_->size() && t.size() > 1) {
		AppendMove<city_type> closing{t[0]};
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
	AppendMove<city_type> closing{tour_[0]};
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
auto Solver<Dist, Variant>::evaluate(city_type city) const -> std::optional<double> {
	assert(dist_ && "evaluate: no distance source set");
	assert(n_ > 0 && "evaluate: tour must have at least one city");

	AppendMove<city_type> move{city};
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

	AppendMove<city_type> move{city};
	ctx_.init(move, *dist_, {tour_.data(), n_}, {position_.data(), n_}, cost_);
	invoke_prepare(variant_ref(), move, ctx_);

	if (n_ > 0)
		cost_ += static_cast<cost_type>(edge_delta(tour_[n_ - 1], city));

	tour_[n_] = city;
	visited_[static_cast<std::size_t>(city)] = 1;
	position_[static_cast<std::size_t>(city)] = static_cast<city_type>(n_);
	cumul_costs_[n_] = cost_;
	++n_;

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
	return evaluate(city).has_value();
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

	// Rebuild dimensions (the cost is overridden below).
	if (n_ > 0)
		rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));

	cost_ = known_cost;

	if (n_ == total) {
		status_ = SolutionStatus::feasible;
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