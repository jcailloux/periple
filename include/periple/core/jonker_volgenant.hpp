#pragma once

// Jonker-Volgenant ATSP-to-STSP transformation
//
// Jonker, Volgenant (1983), "Transforming Asymmetric into Symmetric Traveling Salesman Problems"

#include <periple/core/traits.hpp>

#include <cassert>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace periple {

// ---------------------------------------------------------------------------
// JonkerVolgenantView -- zero-copy adapter that presents an n-city asymmetric
// distance source as a 2n-city symmetric one, enabling symmetric-only
// algorithms (e.g. Lin-Kernighan, 2-opt) to solve ATSP instances.
//
// The cost_type must support negative values (signed integers or floats).
// Requires n >= 2.
// ---------------------------------------------------------------------------

template <DistanceSource Dist>
class JonkerVolgenantView {
public:
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;
	static constexpr bool is_symmetric = true;

	JonkerVolgenantView(const Dist& dist, cost_type big_m)
		: dist_(&dist), n_(dist.size()), big_m_(big_m) {
		assert(n_ >= 2);
	}

	cost_type operator()(city_type ci, city_type cj) const {
		auto i = static_cast<std::size_t>(ci);
		auto j = static_cast<std::size_t>(cj);

		bool i_ghost = i >= n_;
		bool j_ghost = j >= n_;

		// Same class (both real or both ghost) -- includes diagonal (i == j).
		if (i_ghost == j_ghost)
			return i == j ? cost_type{} : inf();

		// Cross-class: normalize to (real, ghost_origin).
		std::size_t r = i_ghost ? j : i;
		std::size_t g = (i_ghost ? i : j) - n_;

		// Self-pairing: real city paired with its own ghost.
		if (r == g) return -big_m_;

		// Edge (real r) -- (ghost of g) encodes ATSP cost C[g][r].
		return (*dist_)(static_cast<city_type>(g),
		                static_cast<city_type>(r));
	}

	[[nodiscard]] std::size_t size() const { return 2 * n_; }

	[[nodiscard]] std::size_t original_size() const { return n_; }

	[[nodiscard]] cost_type big_m() const { return big_m_; }

	// Convert a symmetric tour cost back to the ATSP tour cost.
	[[nodiscard]] cost_type atsp_cost(cost_type symmetric_cost) const {
		return symmetric_cost + big_m_ * static_cast<cost_type>(n_);
	}

	// Extract the n-city ATSP tour from a 2n-city symmetric tour.
	// Exploits real-ghost alternation: stride-2 read, zero branches in loop.
	void atsp_tour(std::span<const city_type> sym,
	               std::span<city_type> out) const {
		assert(sym.size() == 2 * n_);
		assert(out.size() >= n_);
		assert(verify_alternation(sym));

		std::size_t start = static_cast<std::size_t>(sym[0]) >= n_ ? 1 : 0;
		for (std::size_t i = 0; i < n_; ++i)
			out[i] = sym[start + 2 * i];
	}

	[[nodiscard]] auto atsp_tour(std::span<const city_type> sym) const
		-> std::vector<city_type> {
		std::vector<city_type> out(n_);
		atsp_tour(sym, out);
		return out;
	}

	// Build a 2n-city symmetric tour from an n-city ATSP tour.
	// Each city is followed by its ghost: [c, c+n, c', c'+n, ...].
	void symmetric_tour(std::span<const city_type> atsp,
	                    std::span<city_type> out) const {
		assert(atsp.size() == n_);
		assert(out.size() >= 2 * n_);

		for (std::size_t i = 0; i < n_; ++i) {
			out[2 * i]     = atsp[i];
			out[2 * i + 1] = static_cast<city_type>(
				static_cast<std::size_t>(atsp[i]) + n_);
		}
	}

	[[nodiscard]] auto symmetric_tour(std::span<const city_type> atsp) const
		-> std::vector<city_type> {
		std::vector<city_type> out(2 * n_);
		symmetric_tour(atsp, out);
		return out;
	}

private:
	static constexpr cost_type inf() {
		if constexpr (std::numeric_limits<cost_type>::has_infinity)
			return std::numeric_limits<cost_type>::infinity();
		else
			return std::numeric_limits<cost_type>::max();
	}

	// Debug-only: verify that the tour alternates real and ghost cities.
	[[nodiscard]] bool verify_alternation(
		[[maybe_unused]] std::span<const city_type> sym) const {
		for (std::size_t i = 0; i + 1 < sym.size(); ++i) {
			bool curr_ghost = static_cast<std::size_t>(sym[i]) >= n_;
			bool next_ghost = static_cast<std::size_t>(sym[i + 1]) >= n_;
			if (curr_ghost == next_ghost) return false;
		}
		return true;
	}

	const Dist* dist_;
	std::size_t n_;
	cost_type big_m_;
};

// ---------------------------------------------------------------------------
// Factory functions
// ---------------------------------------------------------------------------

// Scans the distance source to compute a safe big-M (O(n^2)).
// M = n * max_cost + 1 guarantees that self-pairing is always optimal.
template <DistanceSource Dist>
auto jonker_volgenant(const Dist& dist) -> JonkerVolgenantView<Dist> {
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;
	const auto n = dist.size();
	assert(n >= 2);
	cost_type max_cost{};
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t j = 0; j < n; ++j)
			if (i != j) {
				auto c = dist(static_cast<city_type>(i),
				              static_cast<city_type>(j));
				if (c > max_cost) max_cost = c;
			}
	return {dist, max_cost * static_cast<cost_type>(n) + cost_type{1}};
}

// User-provided big-M (skips the O(n^2) scan).
template <DistanceSource Dist>
auto jonker_volgenant(const Dist& dist,
                      typename dist_traits<Dist>::cost_type big_m)
	-> JonkerVolgenantView<Dist> {
	return {dist, big_m};
}

} // namespace periple
