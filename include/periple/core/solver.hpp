#pragma once

#include <periple/core/traits.hpp>
#include <periple/core/callbacks.hpp>
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
// Parameter structs
// ---------------------------------------------------------------------------

struct NearestNeighborParams {
	std::size_t start_city = 0;
};

struct HeldKarpParams {};

// ---------------------------------------------------------------------------
// SolutionStatus
// ---------------------------------------------------------------------------

enum class SolutionStatus { none, partial, feasible, optimal };

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename TourCost = DefaultTourCost>
class Solver {
public:
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	// --- Lifecycle ----------------------------------------------------------

	Solver() = default;
	explicit Solver(const Dist& dist);
	Solver(const Dist& dist, const TourCost& tc);
	void set_matrix(const Dist& dist);
	void set_tour_cost(const TourCost& tc);
	void clear();  // Clears solution state but keeps the distance source and buffers.
	void reset();  // Resets the solver to its default-constructed state.

	// --- State reading ------------------------------------------------------

	auto status() const -> SolutionStatus;
	auto size()   const -> std::size_t;
	auto tour()   const -> std::span<const city_type>;
	auto cost()   const -> cost_type;
	void set_tour(std::span<const city_type> tour);

	// --- Neighbor lists -----------------------------------------------------

	// Returns the k nearest neighbors of city, sorted by distance.
	// Computed lazily on first call; grows if k exceeds previous requests.
	auto neighbors(city_type city, std::size_t k) -> std::span<const city_type>;

	// --- Algorithms (defined inline in algorithms/*.hpp) --------------------

	template <typename Callbacks = DefaultCallbacks>
	auto nearest_neighbor(NearestNeighborParams params = {},
	                      const Callbacks& cb = {}) -> Solver&;

	auto held_karp(HeldKarpParams params = {}) -> Solver&;

	template <typename Selector, typename Callbacks = DefaultCallbacks>
	auto greedy_construct(const Selector& sel, const Callbacks& cb = {},
	                      ConstructParams params = {}) -> Solver&;

#ifdef PERIPLE_TESTING
	template <DistanceSource D, typename TC> friend class SolverTestAccess;
#endif

private:
	void ensure_shared(std::size_t n);
	void ensure_neighbors(std::size_t k);
	void rebuild_position();
	void invalidate_caches();
	auto compute_tour_cost(std::span<const city_type> t) const -> cost_type;

	// State
	const Dist*    dist_       = nullptr;
	const TourCost* tour_cost_ = nullptr;
	SolutionStatus status_     = SolutionStatus::none;
	std::size_t    n_          = 0;
	std::vector<city_type> tour_;
	cost_type      cost_       = {};

	// Workspace (grow-only)
	std::vector<city_type> position_;  // Inverse index: city -> position in tour.
	std::vector<uint8_t>   visited_;
	std::vector<uint8_t>   dont_look_;
	std::vector<city_type> neighbors_;
	std::size_t            neighbors_k_ = 0;

	// Algorithm caches (lazy, one per algorithm that needs persistent state)
	std::optional<HKCache<cost_type, city_type>> hk_cache_;
};

// ---------------------------------------------------------------------------
// Inline implementations -- lifecycle & state
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename TourCost>
Solver<Dist, TourCost>::Solver(const Dist& dist) {
	set_matrix(dist);
}

template <DistanceSource Dist, typename TourCost>
Solver<Dist, TourCost>::Solver(const Dist& dist, const TourCost& tc)
	: tour_cost_(&tc)
{
	set_matrix(dist);
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::set_matrix(const Dist& dist) {
	dist_   = &dist;
	status_ = SolutionStatus::none;
	n_      = 0;
	cost_   = {};
	invalidate_caches();
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::set_tour_cost(const TourCost& tc) {
	tour_cost_ = &tc;
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::clear() {
	status_ = SolutionStatus::none;
	n_      = 0;
	cost_   = {};
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::reset() {
	*this = Solver();
}

// --- State reading ----------------------------------------------------------

template <DistanceSource Dist, typename TourCost>
auto Solver<Dist, TourCost>::status() const -> SolutionStatus {
	return status_;
}

template <DistanceSource Dist, typename TourCost>
auto Solver<Dist, TourCost>::size() const -> std::size_t {
	return dist_ ? dist_->size() : 0;
}

template <DistanceSource Dist, typename TourCost>
auto Solver<Dist, TourCost>::tour() const -> std::span<const city_type> {
	return {tour_.data(), n_};
}

template <DistanceSource Dist, typename TourCost>
auto Solver<Dist, TourCost>::cost() const -> cost_type {
	return cost_;
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::set_tour(std::span<const city_type> t) {
	assert(dist_);
	const auto total = dist_->size();
	assert(t.size() <= total);

	// Debug validation: no duplicates, cities within bounds.
	assert([&] {
		for (std::size_t i = 0; i < t.size(); ++i) {
			if (static_cast<std::size_t>(t[i]) >= total) return false;
			for (std::size_t j = i + 1; j < t.size(); ++j)
				if (t[i] == t[j]) return false;
		}
		return true;
	}());

	ensure_shared(total);
	n_ = t.size();
	std::copy(t.begin(), t.end(), tour_.begin());

	if (n_ == total) {
		cost_   = compute_tour_cost(std::span<const city_type>(tour_.data(), n_));
		status_ = SolutionStatus::feasible;
	} else if (n_ > 0) {
		// Partial tour: open-path cost (no return edge).
		cost_type path_cost{};
		for (std::size_t i = 0; i + 1 < n_; ++i)
			path_cost += (*dist_)(t[i], t[i + 1]);
		cost_   = path_cost;
		status_ = SolutionStatus::partial;
		// Mark visited cities for potential continuation.
		std::fill_n(visited_.data(), total, uint8_t{0});
		for (std::size_t i = 0; i < n_; ++i)
			visited_[static_cast<std::size_t>(t[i])] = 1;
	} else {
		cost_   = {};
		status_ = SolutionStatus::none;
	}

	rebuild_position();
}

// --- Internal helpers -------------------------------------------------------

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::ensure_shared(std::size_t n) {
	if (tour_.size()     < n) tour_.resize(n);
	if (position_.size() < n) position_.resize(n);
	if (visited_.size()  < n) visited_.resize(n);
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::rebuild_position() {
	if (position_.size() < n_) position_.resize(n_);
	for (std::size_t i = 0; i < n_; ++i)
		position_[static_cast<std::size_t>(tour_[i])] =
			static_cast<city_type>(i);
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::invalidate_caches() {
	if (hk_cache_) hk_cache_->reset();
	neighbors_k_ = 0;
}

template <DistanceSource Dist, typename TourCost>
auto Solver<Dist, TourCost>::neighbors(city_type city, std::size_t k)
	-> std::span<const city_type>
{
	assert(dist_);
	assert(k < dist_->size());
	ensure_neighbors(k);
	return {neighbors_.data() +
		static_cast<std::size_t>(city) * neighbors_k_, k};
}

template <DistanceSource Dist, typename TourCost>
void Solver<Dist, TourCost>::ensure_neighbors(std::size_t k) {
	if (k <= neighbors_k_) return;

	const auto n = dist_->size();
	neighbors_k_ = k;
	neighbors_.resize(n * k);

	// Temporary buffer for sorting candidates.
	std::vector<city_type> candidates(n - 1);

	for (std::size_t i = 0; i < n; ++i) {
		// Fill with all cities except i.
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

template <DistanceSource Dist, typename TourCost>
auto Solver<Dist, TourCost>::compute_tour_cost(std::span<const city_type> t)
	const -> cost_type
{
	if constexpr (std::is_same_v<TourCost, DefaultTourCost>) {
		cost_type total{};
		const auto n = t.size();
		for (std::size_t i = 0; i < n; ++i)
			total += (*dist_)(t[i], t[(i + 1) % n]);
		return total;
	} else {
		assert(tour_cost_);
		return (*tour_cost_)(*dist_, t);
	}
}

// --- CTAD guides ------------------------------------------------------------

template <DistanceSource Dist>
Solver(const Dist&) -> Solver<Dist>;

template <DistanceSource Dist, typename TourCost>
Solver(const Dist&, const TourCost&) -> Solver<Dist, TourCost>;

} // namespace periple
