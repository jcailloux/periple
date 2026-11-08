#pragma once

// What the built-in bounds of branch_and_bound share

#include <periple/core/solver.hpp>

#include <cmath>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace periple {

namespace detail {

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

} // namespace periple
