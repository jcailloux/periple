#pragma once

#include <periple/core/traits.hpp>

#include <array>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace periple {

// A DistanceSource that stores N points of D coordinates each and computes
// distances on the fly.  Memory is O(N*D) instead of O(N^2).
//
// DistFunc must be callable as: CostT fn(const double* a, const double* b)
// where a and b each point to D consecutive coordinate values.
// CostT is deduced from the return type of DistFunc via CTAD.
//
// Construction from structured data:
//   CoordinateDistance cd(points, periple::euclidean<int>(2));
// where points is vector<pair<double,double>>, vector<array<double,D>>,
// vector<tuple<double,...>>, or vector<vector<double>>.
//
// Construction from flat data:
//   CoordinateDistance cd(dim, flat_coords, periple::euclidean<int>(dim));

template <typename DistFunc, typename CostT = int, typename CityT = std::size_t>
class CoordinateDistance {
public:
	using cost_type = CostT;
	using city_type = CityT;

	// Flat coordinates: [x0,y0,z0, x1,y1,z1, ...], n deduced from size/dim.
	CoordinateDistance(
		std::size_t dim,
		std::vector<double> coords, DistFunc dist_fn)
		: n_(coords.size() / dim), dim_(dim)
		, coords_(std::move(coords))
		, dist_fn_(std::move(dist_fn))
	{}

	// vector<pair<double, double>> -- 2D points
	CoordinateDistance(
		const std::vector<std::pair<double, double>>& points, DistFunc dist_fn)
		: n_(points.size()), dim_(2)
		, coords_(flatten_pairs(points))
		, dist_fn_(std::move(dist_fn))
	{}

	// vector<array<double, D>> -- fixed-dimension points
	template <std::size_t D>
	CoordinateDistance(
		const std::vector<std::array<double, D>>& points, DistFunc dist_fn)
		: n_(points.size()), dim_(D)
		, coords_(flatten_arrays(points))
		, dist_fn_(std::move(dist_fn))
	{}

	// vector<tuple<double, ...>> -- 2D or 3D points
	template <typename... Ts>
		requires (sizeof...(Ts) >= 2) && (std::is_convertible_v<Ts, double> && ...)
	CoordinateDistance(
		const std::vector<std::tuple<Ts...>>& points, DistFunc dist_fn)
		: n_(points.size()), dim_(sizeof...(Ts))
		, coords_(flatten_tuples(points))
		, dist_fn_(std::move(dist_fn))
	{}

	// vector<vector<double>> -- runtime-dimension points
	CoordinateDistance(
		const std::vector<std::vector<double>>& points, DistFunc dist_fn)
		: n_(points.size()), dim_(points.empty() ? 0 : points[0].size())
		, coords_(flatten_vectors(points))
		, dist_fn_(std::move(dist_fn))
	{}

	[[nodiscard]] cost_type operator()(city_type i, city_type j) const {
		if (i == j) return cost_type{};
		return dist_fn_(
			coords_.data() + static_cast<std::size_t>(i) * dim_,
			coords_.data() + static_cast<std::size_t>(j) * dim_
		);
	}

	[[nodiscard]] std::size_t size() const { return n_; }
	[[nodiscard]] std::size_t dim() const { return dim_; }

	[[nodiscard]] const double* point(city_type i) const {
		return coords_.data() + static_cast<std::size_t>(i) * dim_;
	}

private:
	static std::vector<double> flatten_pairs(
		const std::vector<std::pair<double, double>>& pts)
	{
		std::vector<double> flat(pts.size() * 2);
		for (std::size_t i = 0; i < pts.size(); ++i) {
			flat[i * 2]     = pts[i].first;
			flat[i * 2 + 1] = pts[i].second;
		}
		return flat;
	}

	template <std::size_t D>
	static std::vector<double> flatten_arrays(
		const std::vector<std::array<double, D>>& pts)
	{
		std::vector<double> flat(pts.size() * D);
		for (std::size_t i = 0; i < pts.size(); ++i)
			for (std::size_t k = 0; k < D; ++k)
				flat[i * D + k] = pts[i][k];
		return flat;
	}

	template <typename... Ts>
	static std::vector<double> flatten_tuples(
		const std::vector<std::tuple<Ts...>>& pts)
	{
		constexpr std::size_t D = sizeof...(Ts);
		std::vector<double> flat(pts.size() * D);
		for (std::size_t i = 0; i < pts.size(); ++i)
			flatten_one_tuple(pts[i], flat.data() + i * D,
				std::index_sequence_for<Ts...>{});
		return flat;
	}

	template <typename Tuple, std::size_t... Is>
	static void flatten_one_tuple(
		const Tuple& t, double* out, std::index_sequence<Is...>)
	{
		((out[Is] = static_cast<double>(std::get<Is>(t))), ...);
	}

	static std::vector<double> flatten_vectors(
		const std::vector<std::vector<double>>& pts)
	{
		if (pts.empty()) return {};
		std::size_t dim = pts[0].size();
		std::vector<double> flat(pts.size() * dim);
		for (std::size_t i = 0; i < pts.size(); ++i)
			for (std::size_t k = 0; k < dim; ++k)
				flat[i * dim + k] = pts[i][k];
		return flat;
	}

	std::size_t n_;
	std::size_t dim_;
	std::vector<double> coords_;
	[[no_unique_address]] DistFunc dist_fn_;
};

// CTAD guides: deduce DistFunc and CostT from the distance function's return type.

template <typename DistFunc>
CoordinateDistance(std::size_t, std::vector<double>, DistFunc)
	-> CoordinateDistance<
		DistFunc,
		std::invoke_result_t<DistFunc, const double*, const double*>>;

template <typename DistFunc>
CoordinateDistance(const std::vector<std::pair<double, double>>&, DistFunc)
	-> CoordinateDistance<
		DistFunc,
		std::invoke_result_t<DistFunc, const double*, const double*>>;

template <std::size_t D, typename DistFunc>
CoordinateDistance(const std::vector<std::array<double, D>>&, DistFunc)
	-> CoordinateDistance<
		DistFunc,
		std::invoke_result_t<DistFunc, const double*, const double*>>;

template <typename... Ts, typename DistFunc>
CoordinateDistance(const std::vector<std::tuple<Ts...>>&, DistFunc)
	-> CoordinateDistance<
		DistFunc,
		std::invoke_result_t<DistFunc, const double*, const double*>>;

template <typename DistFunc>
CoordinateDistance(const std::vector<std::vector<double>>&, DistFunc)
	-> CoordinateDistance<
		DistFunc,
		std::invoke_result_t<DistFunc, const double*, const double*>>;

} // namespace periple
