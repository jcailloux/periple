#pragma once

// Evaluation context and dimension framework
//
// Provides the infrastructure for variant composition:
// - EvalContext: read-only solver state (tour, position, cost) + mutable dimensions
// - EmptyContext: lightweight context for NoCallbacks (no dimensions)
// - Dimension concept and Dimensions<Ts...> type list
// - context_for<Variant>: deduce the right context type from a variant
// - invoke_prepare / invoke_filter: dispatch to variant callbacks

#include <periple/core/traits.hpp>
#include <periple/core/moves.hpp>

#include <algorithm>
#include <cstddef>
#include <span>
#include <tuple>
#include <type_traits>
#include <vector>

namespace periple {

// ---------------------------------------------------------------------------
// Dimensions type list
// ---------------------------------------------------------------------------

template <typename... Ts>
struct Dimensions {};

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

	template <typename Move> void commit(const Move&) {}

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

	template <typename D> auto& dim() { return std::get<D>(dims_); }
	template <typename D> const auto& dim() const { return std::get<D>(dims_); }

	[[nodiscard]] auto tour() const -> std::span<const CityT> { return tour_; }
	[[nodiscard]] auto position() const -> std::span<const CityT> { return position_; }
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

	template <typename D, typename Move>
	static void commit_one(D& d, const Move& m) {
		if constexpr (requires { d.commit(m); }) {
			d.commit(m);
		}
	}

	template <typename Tuple, std::size_t... Is>
	void restore_dims(const Tuple& t, std::index_sequence<Is...>) {
		(std::get<Is>(dims_).restore(std::get<Is>(t)), ...);
	}

	std::span<const CityT> tour_;
	std::span<const CityT> position_;
	CostT cost_{};
	std::tuple<Dims...> dims_;
};

// ---------------------------------------------------------------------------
// context_for -- deduce the context type from a Variant
// ---------------------------------------------------------------------------

namespace detail {

template <typename Dim, typename CityT, typename CostT>
struct make_eval_context {
	using type = EvalContext<CityT, CostT, Dim>;
};

template <typename... Ts, typename CityT, typename CostT>
struct make_eval_context<Dimensions<Ts...>, CityT, CostT> {
	using type = EvalContext<CityT, CostT, Ts...>;
};

} // namespace detail

template <typename Variant, typename CityT, typename CostT, typename = void>
struct context_for {
	using type = EmptyContext<CityT, CostT>;
};

template <typename Variant, typename CityT, typename CostT>
struct context_for<Variant, CityT, CostT,
                   std::void_t<typename Variant::dimension>> {
	using type = typename detail::make_eval_context<
		typename Variant::dimension, CityT, CostT>::type;
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

} // namespace periple