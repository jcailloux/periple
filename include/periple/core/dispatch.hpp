#pragma once

#include <concepts>
#include <type_traits>
#include <utility>

namespace periple::detail {

// Returns true if the variant's move_filter accepts the move.
// If the variant does not provide move_filter for these args, returns true.
template <typename Variant, typename... Args>
bool dispatch_filter(const Variant& v, Args&&... args) {
	if constexpr (requires {
		{ v.move_filter(std::forward<Args>(args)...) } -> std::convertible_to<bool>;
	}) {
		return v.move_filter(std::forward<Args>(args)...);
	} else {
		return true;
	}
}

// Returns the variant's move_eval result, or raw_cost if not provided.
template <typename CostT, typename Variant, typename... Args>
auto dispatch_eval(CostT raw_cost, const Variant& v, Args&&... args) -> CostT {
	if constexpr (requires {
		{ v.move_eval(std::forward<Args>(args)...) } -> std::convertible_to<CostT>;
	}) {
		return static_cast<CostT>(v.move_eval(std::forward<Args>(args)...));
	} else {
		return raw_cost;
	}
}

// Calls the variant's on_move if provided. No-op otherwise.
template <typename Variant, typename... Args>
void dispatch_on_move(const Variant& v, Args&&... args) {
	if constexpr (requires { v.on_move(std::forward<Args>(args)...); }) {
		v.on_move(std::forward<Args>(args)...);
	}
}

} // namespace periple::detail