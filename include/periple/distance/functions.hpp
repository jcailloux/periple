#pragma once

#include <cmath>
#include <cstddef>
#include <type_traits>

namespace periple {

// ---------------------------------------------------------------------------
// Generic distance function factories.
// Each takes a dimension and returns a metric compatible with
// CoordinateDistance: CostT fn(const double* a, const double* b). Every metric
// declares itself symmetric at compile time, and so does the source built on it.
//
// CostT controls the return type:
//   - floating-point (double, float): exact distance, no rounding
//   - integral (int, long, ...):      rounded to nearest integer
// except ceiling_euclidean, which rounds up for both.
//
// Usage:
//   CoordinateDistance cd(3, coords, periple::euclidean<double>(3));
// ---------------------------------------------------------------------------

// L2 norm: sqrt(sum (a_i - b_i)^2)
template <typename CostT = double>
struct Euclidean {
	std::size_t dim;

	static constexpr bool symmetric() { return true; }

	CostT operator()(const double* a, const double* b) const {
		double sum = 0;
		for (std::size_t k = 0; k < dim; ++k) {
			double d = a[k] - b[k];
			sum += d * d;
		}
		double dist = std::sqrt(sum);
		if constexpr (std::is_integral_v<CostT>)
			return static_cast<CostT>(std::round(dist));
		else
			return static_cast<CostT>(dist);
	}
};

template <typename CostT = double>
Euclidean<CostT> euclidean(std::size_t dim) { return {dim}; }

// L2 norm with ceiling: ceil(sqrt(sum (a_i - b_i)^2))
template <typename CostT = double>
struct CeilingEuclidean {
	std::size_t dim;

	static constexpr bool symmetric() { return true; }

	CostT operator()(const double* a, const double* b) const {
		double sum = 0;
		for (std::size_t k = 0; k < dim; ++k) {
			double d = a[k] - b[k];
			sum += d * d;
		}
		return static_cast<CostT>(std::ceil(std::sqrt(sum)));
	}
};

template <typename CostT = double>
CeilingEuclidean<CostT> ceiling_euclidean(std::size_t dim) { return {dim}; }

// L1 norm: sum |a_i - b_i|
template <typename CostT = double>
struct Manhattan {
	std::size_t dim;

	static constexpr bool symmetric() { return true; }

	CostT operator()(const double* a, const double* b) const {
		double sum = 0;
		for (std::size_t k = 0; k < dim; ++k)
			sum += std::abs(a[k] - b[k]);
		if constexpr (std::is_integral_v<CostT>)
			return static_cast<CostT>(std::round(sum));
		else
			return static_cast<CostT>(sum);
	}
};

template <typename CostT = double>
Manhattan<CostT> manhattan(std::size_t dim) { return {dim}; }

// L-infinity norm: max |a_i - b_i|
template <typename CostT = double>
struct Chebyshev {
	std::size_t dim;

	static constexpr bool symmetric() { return true; }

	CostT operator()(const double* a, const double* b) const {
		double mx = 0;
		for (std::size_t k = 0; k < dim; ++k) {
			double d = std::abs(a[k] - b[k]);
			if (d > mx) mx = d;
		}
		if constexpr (std::is_integral_v<CostT>)
			return static_cast<CostT>(std::round(mx));
		else
			return static_cast<CostT>(mx);
	}
};

template <typename CostT = double>
Chebyshev<CostT> chebyshev(std::size_t dim) { return {dim}; }

// ---------------------------------------------------------------------------
// TSPLIB95-specific distance functions.
// These implement the exact formulas from the TSPLIB95 specification,
// including its specific constants and rounding rules. Always 2D, always int.
// ---------------------------------------------------------------------------

namespace tsplib {

// EUC_2D: nint(sqrt(dx^2 + dy^2))
inline int euc_2d(const double* a, const double* b) {
	double dx = a[0] - b[0], dy = a[1] - b[1];
	return static_cast<int>(std::round(std::sqrt(dx * dx + dy * dy)));
}

// CEIL_2D: ceil(sqrt(dx^2 + dy^2))
inline int ceil_2d(const double* a, const double* b) {
	double dx = a[0] - b[0], dy = a[1] - b[1];
	return static_cast<int>(std::ceil(std::sqrt(dx * dx + dy * dy)));
}

// ATT: pseudo-Euclidean distance. ceil(sqrt((dx^2 + dy^2) / 10))
inline int att(const double* a, const double* b) {
	double dx = a[0] - b[0], dy = a[1] - b[1];
	double r = std::sqrt((dx * dx + dy * dy) / 10.0);
	int t = static_cast<int>(r);
	return (t < r) ? t + 1 : t;
}

namespace detail {

// TSPLIB coordinate-to-radians conversion.
// Uses PI = 3.141592 (not M_PI) per the TSPLIB95 specification.
inline double geo_to_radians(double coord) {
	constexpr double pi = 3.141592;
	int deg = static_cast<int>(coord);
	double min = coord - deg;
	return pi * (deg + 5.0 * min / 3.0) / 180.0;
}

} // namespace detail

// GEO: great-circle distance. Uses Earth radius RRR = 6378.388 per TSPLIB95.
inline int geo(const double* a, const double* b) {
	constexpr double rrr = 6378.388;
	double lat1 = detail::geo_to_radians(a[0]), lon1 = detail::geo_to_radians(a[1]);
	double lat2 = detail::geo_to_radians(b[0]), lon2 = detail::geo_to_radians(b[1]);
	// cos is even, but std::cos does not promise equal results for x and -x.
	// Under the default rounding a - b and b - a are exact opposites, so abs
	// makes d(a, b) == d(b, a).
	double q1 = std::cos(std::abs(lon1 - lon2));
	double q2 = std::cos(std::abs(lat1 - lat2));
	double q3 = std::cos(lat1 + lat2);
	return static_cast<int>(
		rrr * std::acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0);
}

} // namespace tsplib

} // namespace periple