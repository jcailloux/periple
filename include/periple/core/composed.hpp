#pragma once

// Variant composition
//
// Composed<Vs...> combines N variants into a single variant.
// move_prepare pipelines through all variants in order.
// move_filter AND-short-circuits through all variants in order.
// Dimensions are merged and deduplicated across all variants.

#include <periple/core/dimensions.hpp>

#include <tuple>
#include <type_traits>

namespace periple {

namespace detail {

// Extract dimensions from a variant (empty if no dimension typedef).
template <typename V, typename = void>
struct extract_dims { using type = Dimensions<>; };

template <typename V>
struct extract_dims<V, std::void_t<typename V::dimension>> {
	using type = typename normalize_dim<typename V::dimension>::type;
};

// Merge dimensions from all variants: extract, concat, deduplicate.
template <typename... Vs>
struct merge_dims {
	using type = typename deduplicate<typename concat_all<typename extract_dims<Vs>::type...>::type>::type;
};

// Conditionally declare 'dimension' based on merged dims.
template <typename MergedDims>
struct composed_dimension_base {};

template <typename T>
struct composed_dimension_base<Dimensions<T>> {
	using dimension = T;
};

template <typename T, typename U, typename... Rest>
struct composed_dimension_base<Dimensions<T, U, Rest...>> {
	using dimension = Dimensions<T, U, Rest...>;
};

} // namespace detail

// ---------------------------------------------------------------------------
// Composed<Vs...>: combines N variants into a single variant
// ---------------------------------------------------------------------------

template <typename... Vs>
class Composed : public detail::composed_dimension_base<typename detail::merge_dims<Vs...>::type> {
	std::tuple<const Vs*...> variants_;

public:
	explicit Composed(const Vs&... vs) : variants_(&vs...) {}

	template <typename Move, typename Ctx>
	void move_prepare(const Move& m, Ctx& ctx) const {
		std::apply([&](const auto*... vs) {
			(invoke_prepare(*vs, m, ctx), ...);
		}, variants_);
	}

	template <typename Move, typename Ctx>
	bool move_filter(const Move& m, const Ctx& ctx) const {
		return std::apply([&](const auto*... vs) {
			return (invoke_filter(*vs, m, ctx) && ...);
		}, variants_);
	}
};

template <typename... Vs>
Composed(const Vs&...) -> Composed<Vs...>;

} // namespace periple