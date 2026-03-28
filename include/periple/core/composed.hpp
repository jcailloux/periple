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

// Normalize a dimension declaration to Dimensions<...> form.
template <typename Dim>
struct normalize_dim { using type = Dimensions<Dim>; };

template <typename... Ts>
struct normalize_dim<Dimensions<Ts...>> { using type = Dimensions<Ts...>; };

// Extract dimensions from a variant (empty if no dimension typedef).
template <typename V, typename = void>
struct extract_dims { using type = Dimensions<>; };

template <typename V>
struct extract_dims<V, std::void_t<typename V::dimension>> {
	using type = typename normalize_dim<typename V::dimension>::type;
};

// Concatenate two Dimensions lists.
template <typename A, typename B> struct concat_dims;

template <typename... As, typename... Bs>
struct concat_dims<Dimensions<As...>, Dimensions<Bs...>> {
	using type = Dimensions<As..., Bs...>;
};

// Concatenate N Dimensions lists.
template <typename... Ds> struct concat_all;
template <> struct concat_all<> { using type = Dimensions<>; };
template <typename D> struct concat_all<D> { using type = D; };

template <typename D1, typename D2, typename... Rest>
struct concat_all<D1, D2, Rest...> {
	using type = typename concat_all<typename concat_dims<D1, D2>::type, Rest...>::type;
};

// Left-fold deduplication: accumulate unique types.
template <typename Acc, typename... Ts> struct unique_fold;
template <typename Acc> struct unique_fold<Acc> { using type = Acc; };

template <typename... Acc, typename T, typename... Rest>
struct unique_fold<Dimensions<Acc...>, T, Rest...> {
	using next = std::conditional_t<(std::is_same_v<T, Acc> || ...), Dimensions<Acc...>, Dimensions<Acc..., T>>;
	using type = typename unique_fold<next, Rest...>::type;
};

template <typename D> struct deduplicate;

template <typename... Ts>
struct deduplicate<Dimensions<Ts...>> {
	using type = typename unique_fold<Dimensions<>, Ts...>::type;
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