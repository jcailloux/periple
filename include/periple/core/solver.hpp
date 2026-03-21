#pragma once

#include <periple/core/traits.hpp>
#include <periple/core/caches/hk_cache.hpp>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
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

enum class SolutionStatus { none, feasible, optimal };

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
class Solver {
public:
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

	// --- Lifecycle ----------------------------------------------------------

	Solver() = default;
	explicit Solver(const Dist& dist);
	void set_matrix(const Dist& dist);
	void clear();  // Clears solution state but keeps the distance source and buffers.
	void reset();  // Resets the solver to its default-constructed state.

	// --- State reading ------------------------------------------------------

	auto status() const -> SolutionStatus;
	auto size()   const -> std::size_t;
	auto tour()   const -> std::span<const city_type>;
	auto cost()   const -> cost_type;
	void set_tour(std::span<const city_type> tour); // Does not recompute cost.

	// --- Algorithms (defined inline in algorithms/*.hpp) --------------------

	auto nearest_neighbor(NearestNeighborParams params = {}) -> Solver&;
	auto held_karp(HeldKarpParams params = {})               -> Solver&;

#ifdef PERIPLE_TESTING
	template <DistanceSource D> friend class SolverTestAccess;
#endif

private:
	void ensure_shared(std::size_t n);
	void rebuild_position();
	void invalidate_caches();

	// State
	const Dist*    dist_   = nullptr;
	SolutionStatus status_ = SolutionStatus::none;
	std::size_t    n_      = 0;
	std::vector<city_type> tour_;
	cost_type      cost_   = {};

	// Workspace (grow-only)
	std::vector<city_type> position_;  // Inverse index: city -> position in tour.
	std::vector<uint8_t>   visited_;
	std::vector<uint8_t>   dont_look_;
	std::vector<city_type> neighbors_;

	// Algorithm caches (lazy, one per algorithm that needs persistent state)
	std::optional<HKCache<cost_type, city_type>> hk_cache_;
};

// ---------------------------------------------------------------------------
// Inline implementations --lifecycle & state
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
Solver<Dist>::Solver(const Dist& dist) {
	set_matrix(dist);
}

template <DistanceSource Dist>
void Solver<Dist>::set_matrix(const Dist& dist) {
	dist_   = &dist;
	status_ = SolutionStatus::none;
	n_      = 0;
	cost_   = {};
	invalidate_caches();
}

template <DistanceSource Dist>
void Solver<Dist>::clear() {
	status_ = SolutionStatus::none;
	n_      = 0;
	cost_   = {};
}

template <DistanceSource Dist>
void Solver<Dist>::reset() {
	*this = Solver();
}

// --- State reading ----------------------------------------------------------

template <DistanceSource Dist>
auto Solver<Dist>::status() const -> SolutionStatus {
	return status_;
}

template <DistanceSource Dist>
auto Solver<Dist>::size() const -> std::size_t {
	return dist_ ? dist_->size() : 0;
}

template <DistanceSource Dist>
auto Solver<Dist>::tour() const -> std::span<const city_type> {
	return {tour_.data(), n_};
}

template <DistanceSource Dist>
auto Solver<Dist>::cost() const -> cost_type {
	return cost_;
}

template <DistanceSource Dist>
void Solver<Dist>::set_tour(std::span<const city_type> t) {
	n_ = t.size();
	ensure_shared(n_);
	std::copy(t.begin(), t.end(), tour_.begin());
	cost_ = {};
	rebuild_position();
}

// --- Internal helpers -------------------------------------------------------

template <DistanceSource Dist>
void Solver<Dist>::ensure_shared(std::size_t n) {
	if (tour_.size()     < n) tour_.resize(n);
	if (position_.size() < n) position_.resize(n);
	if (visited_.size()  < n) visited_.resize(n);
}

template <DistanceSource Dist>
void Solver<Dist>::rebuild_position() {
	if (position_.size() < n_) position_.resize(n_);
	for (std::size_t i = 0; i < n_; ++i)
		position_[static_cast<std::size_t>(tour_[i])] =
			static_cast<city_type>(i);
}

template <DistanceSource Dist>
void Solver<Dist>::invalidate_caches() {
	if (hk_cache_) hk_cache_->reset();
}

template <DistanceSource Dist>
Solver(const Dist&) -> Solver<Dist>;

} // namespace periple
