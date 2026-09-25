#pragma once

// Evaluation context and dimension framework
//
// Provides the infrastructure for variant composition:
// - EvalContext: read-only solver state (tour, position, cost) + mutable dimensions
// - EmptyContext: lightweight context for NoCallbacks (no dimensions)
// - Dimension concept and Dimensions<Ts...> type list
// - Dimension list helpers (normalize, concat, deduplicate)
// - context_for<Variant>: deduce the right context type from a variant
// - invoke_prepare / invoke_filter: dispatch to variant callbacks
// - covers_move, dims_handle, replay_safe: compile-time checks that variants
//   and dimensions handle every move an algorithm emits

#include <periple/core/traits.hpp>
#include <periple/core/moves/append_move.hpp>
#include <periple/core/moves/dp_move.hpp>
#include <periple/core/dimensions/cumulative_cost.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <span>
#include <tuple>
#include <type_traits>
#include <vector>

namespace periple {

// ---------------------------------------------------------------------------
// Tags
// ---------------------------------------------------------------------------

struct NoCallbacks {};

// ---------------------------------------------------------------------------
// Dimensions type list
// ---------------------------------------------------------------------------

template <typename... Ts>
struct Dimensions {};

namespace detail {

// Normalize a dimension declaration to Dimensions<...> form.
template <typename Dim>
struct normalize_dim { using type = Dimensions<Dim>; };

template <typename... Ts>
struct normalize_dim<Dimensions<Ts...>> { using type = Dimensions<Ts...>; };

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

// A replay commits a candidate's moves before it is kept, so per-position state is staged.
template <typename D, typename CityT, typename Ctx>
concept commits_append_state =
	requires(D& d, const AppendMove<CityT>& m, Ctx& ctx) { d.commit(m, ctx); } ||
	requires(D& d, const AppendMove<CityT>& m) { d.commit(m); };

template <typename D>
concept stages_state = requires(D& d, std::size_t i) {
	d.begin_staging(i);
	d.discard_staging();
	d.save_staging(i);
	d.commit_staging(i);
};

// A dimension takes part in Move when it initializes or commits state for it.
template <typename D, typename Move, typename Dist, typename Ctx>
concept dimension_handles_move =
	requires(D& d, const Move& m, const Dist& dist, Ctx& ctx) { d.init(m, dist, ctx); } ||
	requires(D& d, const Move& m, const Dist& dist) { d.init(m, dist); } ||
	requires(D& d, const Move& m, Ctx& ctx) { d.commit(m, ctx); } ||
	requires(D& d, const Move& m) { d.commit(m); };

} // namespace detail

// ---------------------------------------------------------------------------
// Dimension concept
// ---------------------------------------------------------------------------

template <typename T>
concept Dimension = requires(T& t, std::size_t n) {
	t.resize(n);
	t.reset();
};

// ---------------------------------------------------------------------------
// EmptyContext -- for NoCallbacks (no dimensions)
// ---------------------------------------------------------------------------

template <typename CityT, typename CostT>
struct EmptyContext {
	double cost_delta = 0;

	template <typename D> static constexpr bool has_dim = false;

	[[nodiscard]] auto tour() const -> std::span<const CityT> { return tour_; }
	[[nodiscard]] auto position() const -> std::span<const CityT> { return position_; }
	[[nodiscard]] auto cost() const -> CostT { return cost_; }

	void resize(std::size_t) {}
	void reset() { cost_delta = 0; }

	template <typename Move, typename Dist>
	void init(const Move&, const Dist&,
	          std::span<const CityT> tour, std::span<const CityT> pos,
	          CostT cost) {
		tour_ = tour; position_ = pos; cost_ = cost; cost_delta = 0;
	}

	template <typename Move, typename Dist>
	void init(const Move&, const Dist&, CostT cost) {
		tour_ = {}; position_ = {}; cost_ = cost; cost_delta = 0;
	}

	template <typename Move> void commit(const Move&) {}

	void begin_staging(std::size_t) {}
	void discard_staging() {}
	void save_staging(std::size_t) {}
	void commit_staging(std::size_t) {}
	[[nodiscard]] bool staging_active() const { return false; }

	template <typename Move, typename Dist>
	static constexpr bool dims_handle = true;

	static constexpr bool replay_safe = true;

	struct Snapshot { double cost_delta; };
	template <typename Move>
	auto snapshot(const Move&) const -> Snapshot { return {cost_delta}; }
	void restore(const Snapshot& s) { cost_delta = s.cost_delta; }

private:
	std::span<const CityT> tour_;
	std::span<const CityT> position_;
	CostT cost_{};
};

// ---------------------------------------------------------------------------
// EvalContext -- typed context with dimensions
// ---------------------------------------------------------------------------

template <typename CityT, typename CostT, Dimension... Dims>
struct EvalContext {
	double cost_delta = 0;

	template <typename D>
	static constexpr bool has_dim = (std::is_same_v<D, Dims> || ...);

	template <typename D> auto& dim() { return std::get<D>(dims_); }
	template <typename D> const auto& dim() const { return std::get<D>(dims_); }

	// Construction only: replays and DPs work without a partial tour.
	[[nodiscard]] auto tour() const -> std::span<const CityT> {
		assert(tour_known_ && "ctx.tour(): no partial tour during a replay or a DP, read the move and the dimensions instead");
		return tour_;
	}
	[[nodiscard]] auto position() const -> std::span<const CityT> {
		assert(tour_known_ && "ctx.position(): no partial tour during a replay or a DP, read the move and the dimensions instead");
		return position_;
	}
	[[nodiscard]] auto cost() const -> CostT { return cost_; }

	void resize(std::size_t n) {
		std::apply([n](auto&... ds) { (ds.resize(n), ...); }, dims_);
	}
	void reset() {
		cost_delta = 0;
		std::apply([](auto&... ds) { (ds.reset(), ...); }, dims_);
	}

	template <typename Move, typename Dist>
	void init(const Move& m, const Dist& dist,
	          std::span<const CityT> tour, std::span<const CityT> pos,
	          CostT cost) {
		tour_ = tour;
		position_ = pos;
		tour_known_ = true;
		cost_ = cost;
		cost_delta = 0;
		std::apply([&](auto&... ds) {
			(init_one(ds, m, dist), ...);
		}, dims_);
	}

	template <typename Move, typename Dist>
	void init(const Move& m, const Dist& dist, CostT cost) {
		tour_ = {};
		position_ = {};
		tour_known_ = false;
		cost_ = cost;
		cost_delta = 0;
		std::apply([&](auto&... ds) {
			(init_one(ds, m, dist), ...);
		}, dims_);
	}

	template <typename Move>
	void commit(const Move& m) {
		std::apply([&](auto&... ds) {
			(commit_one(ds, m), ...);
		}, dims_);
	}

	// --- Staging lifecycle (dispatched to dimensions that support it) -----

	void begin_staging(std::size_t from) {
		std::apply([from](auto&... ds) {
			(([]<typename D>(D& d, std::size_t f) {
				if constexpr (requires { d.begin_staging(f); })
					d.begin_staging(f);
			}(ds, from)), ...);
		}, dims_);
	}

	void discard_staging() {
		std::apply([](auto&... ds) {
			(([]<typename D>(D& d) {
				if constexpr (requires { d.discard_staging(); })
					d.discard_staging();
			}(ds)), ...);
		}, dims_);
	}

	void save_staging(std::size_t to) {
		std::apply([to](auto&... ds) {
			(([]<typename D>(D& d, std::size_t t) {
				if constexpr (requires { d.save_staging(t); })
					d.save_staging(t);
			}(ds, to)), ...);
		}, dims_);
	}

	void commit_staging(std::size_t to) {
		std::apply([to](auto&... ds) {
			(([]<typename D>(D& d, std::size_t t) {
				if constexpr (requires { d.commit_staging(t); })
					d.commit_staging(t);
			}(ds, to)), ...);
		}, dims_);
	}

	[[nodiscard]] bool staging_active() const {
		return std::apply([](const auto&... ds) {
			return (([]<typename D>(const D& d) {
				if constexpr (requires { d.staging_active(); })
					return d.staging_active();
				else
					return false;
			}(ds)) || ...);
		}, dims_);
	}

	// True when every dimension that commits along the tour stages its state.
	static constexpr bool replay_safe =
		((detail::stages_state<Dims> || !detail::commits_append_state<Dims, CityT, EvalContext>) && ...);

	// True when every dimension takes part in Move (init or commit).
	template <typename Move, typename Dist>
	static constexpr bool dims_handle = (detail::dimension_handles_move<Dims, Move, Dist, EvalContext> && ...);

	// Snapshot parameterized by move type.
	// Only instantiated when snapshot() is called (e.g. for AppendMove in
	// constructive algorithms). DP commits inline and never snapshots.
	template <typename Move>
	struct Snapshot {
		double cost_delta;
		std::tuple<
			decltype(std::declval<Dims>().snapshot(std::declval<Move>()))...
		> dims;
	};

	template <typename Move>
	auto snapshot(const Move& m) const -> Snapshot<Move> {
		return {cost_delta, {std::get<Dims>(dims_).snapshot(m)...}};
	}

	template <typename Move>
	void restore(const Snapshot<Move>& s) {
		cost_delta = s.cost_delta;
		restore_dims(s.dims, std::index_sequence_for<Dims...>{});
	}

private:
	// Dispatch init: 3-param (Move, Dist, Ctx) if available, else 2-param.
	template <typename D, typename Move, typename Dist>
	void init_one(D& d, const Move& m, const Dist& dist) {
		if constexpr (requires { d.init(m, dist, *this); }) {
			d.init(m, dist, *this);
		} else if constexpr (requires { d.init(m, dist); }) {
			d.init(m, dist);
		}
	}

	// Dispatch commit: 2-param (Move, Ctx) if available, else 1-param.
	template <typename D, typename Move>
	void commit_one(D& d, const Move& m) {
		if constexpr (requires { d.commit(m, *this); }) {
			d.commit(m, *this);
		} else if constexpr (requires { d.commit(m); }) {
			d.commit(m);
		}
	}

	template <typename Tuple, std::size_t... Is>
	void restore_dims(const Tuple& t, std::index_sequence<Is...>) {
		(std::get<Is>(dims_).restore(std::get<Is>(t)), ...);
	}

	std::span<const CityT> tour_;
	std::span<const CityT> position_;
	bool tour_known_ = false;
	CostT cost_{};
	std::tuple<Dims...> dims_;
};

// ---------------------------------------------------------------------------
// context_for -- deduce the context type from a Variant
// ---------------------------------------------------------------------------

namespace detail {

// Dimensions declared by a variant, normalized (empty if none).
template <typename Variant, typename = void>
struct declared_dims { using type = Dimensions<>; };

template <typename Variant>
struct declared_dims<Variant, std::void_t<typename Variant::dimension>> {
	using type = typename normalize_dim<typename Variant::dimension>::type;
};

template <typename DimList, typename CityT, typename CostT>
struct make_eval_context;

template <typename... Ts, typename CityT, typename CostT>
struct make_eval_context<Dimensions<Ts...>, CityT, CostT> {
	using type = EvalContext<CityT, CostT, Ts...>;
};

} // namespace detail

// NoCallbacks -> EmptyContext. Any other variant -> EvalContext with
// CumulativeCost (needed by replay) ahead of its declared dimensions, deduplicated.
template <typename Variant, typename CityT, typename CostT>
struct context_for {
	using type = typename detail::make_eval_context<
		typename detail::deduplicate<
			typename detail::concat_dims<
				Dimensions<CumulativeCost>,
				typename detail::declared_dims<Variant>::type>::type>::type,
		CityT, CostT>::type;
};

template <typename CityT, typename CostT>
struct context_for<NoCallbacks, CityT, CostT> {
	using type = EmptyContext<CityT, CostT>;
};

// ---------------------------------------------------------------------------
// Invoke helpers -- dispatch to variant callbacks
// ---------------------------------------------------------------------------

template <typename Variant, typename Move, typename Ctx>
void invoke_prepare(const Variant& v, const Move& m, Ctx& ctx) {
	if constexpr (requires { v.move_prepare(m, ctx); }) {
		v.move_prepare(m, ctx);
	} else if constexpr (requires { v.move_prepare(m); }) {
		v.move_prepare(m);
	}
}

template <typename Variant, typename Move, typename Ctx>
bool invoke_filter(const Variant& v, const Move& m, const Ctx& ctx) {
	if constexpr (requires {
		{ v.move_filter(m, ctx) } -> std::convertible_to<bool>;
	}) {
		return v.move_filter(m, ctx);
	} else if constexpr (requires {
		{ v.move_filter(m) } -> std::convertible_to<bool>;
	}) {
		return v.move_filter(m);
	} else {
		return true;
	}
}

// ---------------------------------------------------------------------------
// Move coverage -- variants handle every move an algorithm emits
// ---------------------------------------------------------------------------

template <typename... Vs> class Composed;

namespace detail {

template <typename V, typename Move, typename Ctx>
concept handles_move =
	requires(const V& v, const Move& m, Ctx& ctx) { v.move_prepare(m, ctx); } ||
	requires(const V& v, const Move& m) { v.move_prepare(m); } ||
	requires(const V& v, const Move& m, const Ctx& ctx) { v.move_filter(m, ctx); } ||
	requires(const V& v, const Move& m) { v.move_filter(m); };

// Every move type that variants write callbacks for.
template <typename V, typename Ctx, typename CityT>
concept handles_any_move =
	handles_move<V, AppendMove<CityT>, Ctx> || handles_move<V, DPMove<CityT>, Ctx>;

// V handles Move or has no callbacks; a Composed is checked per component.
template <typename V, typename Move, typename Ctx, typename CityT>
struct covers_move
	: std::bool_constant<handles_move<V, Move, Ctx> || !handles_any_move<V, Ctx, CityT>> {};

template <typename... Vs, typename Move, typename Ctx, typename CityT>
struct covers_move<Composed<Vs...>, Move, Ctx, CityT>
	: std::bool_constant<(covers_move<Vs, Move, Ctx, CityT>::value && ...)> {};

} // namespace detail

} // namespace periple