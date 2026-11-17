#pragma once

// TSP with Time Windows (TSPTW)
//
// Savelsbergh (1985), "Local Search in Routing Problems with Time Windows"

#include <periple/core/composed.hpp>
#include <periple/core/dimensions/route_timing.hpp>
#include <periple/core/moves/dp_move.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>

namespace periple::time_windows {

struct TimeWindow {
	double earliest;
	double latest;
};

namespace detail {

// ---------------------------------------------------------------------------
// WindowStore -- flat storage for per-city time windows
// ---------------------------------------------------------------------------

class WindowStore {
public:
	// Single window per city.
	explicit WindowStore(std::span<const TimeWindow> windows)
		: windows_(windows.begin(), windows.end())
	{
		offsets_.resize(windows.size() + 1);
		for (std::size_t i = 0; i <= windows.size(); ++i)
			offsets_[i] = i;
	}

	// Optional window per city (nullopt = unconstrained).
	explicit WindowStore(std::span<const std::optional<TimeWindow>> windows) {
		offsets_.resize(windows.size() + 1);
		std::size_t pos = 0;
		for (std::size_t i = 0; i < windows.size(); ++i) {
			offsets_[i] = pos;
			if (windows[i]) {
				windows_.push_back(*windows[i]);
				++pos;
			}
		}
		offsets_[windows.size()] = pos;
	}

	// Multiple windows per city (sorted by earliest internally).
	explicit WindowStore(std::span<const std::vector<TimeWindow>> windows) {
		offsets_.reserve(windows.size() + 1);
		offsets_.push_back(0);
		for (const auto& ws : windows) {
			windows_.insert(windows_.end(), ws.begin(), ws.end());
			offsets_.push_back(windows_.size());
		}
		sort_windows();
	}

	[[nodiscard]] std::span<const TimeWindow> operator[](std::size_t city) const {
		return {windows_.data() + offsets_[city],
		        offsets_[city + 1] - offsets_[city]};
	}

private:
	void sort_windows() {
		for (std::size_t i = 0; i + 1 < offsets_.size(); ++i) {
			auto begin = windows_.begin() + static_cast<std::ptrdiff_t>(offsets_[i]);
			auto end   = windows_.begin() + static_cast<std::ptrdiff_t>(offsets_[i + 1]);
			std::sort(begin, end, [](const TimeWindow& a, const TimeWindow& b) {
				return a.earliest < b.earliest;
			});
		}
	}

	std::vector<TimeWindow> windows_;
	std::vector<std::size_t> offsets_;
};

// ---------------------------------------------------------------------------
// Helpers -- shared feasibility and timing logic
// ---------------------------------------------------------------------------

inline bool is_feasible(std::span<const TimeWindow> ws, double arrival) {
	if (ws.empty()) return true;
	for (const auto& w : ws) {
		if (arrival <= w.latest)
			return true;
	}
	return false;
}

inline double departure_time(std::span<const TimeWindow> ws, double arrival) {
	if (ws.empty()) return arrival;
	for (const auto& w : ws) {
		if (arrival <= w.latest)
			return std::max(arrival, w.earliest);
	}
	return arrival;
}

inline double violation_amount(std::span<const TimeWindow> ws, double arrival) {
	if (ws.empty()) return 0.0;
	double min_violation = std::numeric_limits<double>::max();
	for (const auto& w : ws) {
		if (arrival <= w.latest) return 0.0;
		min_violation = std::min(min_violation, arrival - w.latest);
	}
	return min_violation;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Strict: hard constraint via move_filter (reject infeasible moves)
// ---------------------------------------------------------------------------

struct Strict {
	using dimension = RouteTiming;

	// Single window per city.
	explicit Strict(std::span<const TimeWindow> windows) : store_(windows) {}

	// Optional window per city (nullopt = unconstrained).
	explicit Strict(std::span<const std::optional<TimeWindow>> windows) : store_(windows) {}

	// Multiple windows per city.
	explicit Strict(std::span<const std::vector<TimeWindow>> windows) : store_(windows) {}

	// A city's windows, sorted by earliest; empty for a city without one.
	template <typename CityT>
	[[nodiscard]] std::span<const TimeWindow> windows(CityT city) const {
		return store_[static_cast<std::size_t>(city)];
	}

	// --- AppendMove ---

	template <typename CityT, typename Ctx>
	void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const {
		auto ws = windows(m.city);
		double arr = ctx.template dim<RouteTiming>().arrival;
		ctx.template dim<RouteTiming>().departure += detail::departure_time(ws, arr) - arr;
	}

	template <typename CityT, typename Ctx>
	bool move_filter(const AppendMove<CityT>& m, const Ctx& ctx) const {
		return detail::is_feasible(windows(m.city), ctx.template dim<RouteTiming>().arrival);
	}

	// --- DPMove ---

	template <typename CityT, typename Ctx>
	void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const {
		auto ws = windows(m.to);
		double arr = ctx.template dim<RouteTiming>().arrival;
		ctx.template dim<RouteTiming>().departure += detail::departure_time(ws, arr) - arr;
	}

	template <typename CityT, typename Ctx>
	bool move_filter(const DPMove<CityT>& m, const Ctx& ctx) const {
		return detail::is_feasible(windows(m.to), ctx.template dim<RouteTiming>().arrival);
	}

private:
	detail::WindowStore store_;
};

// ---------------------------------------------------------------------------
// Relaxed: soft constraint via penalties in cost_delta
// ---------------------------------------------------------------------------

struct Relaxed {
	using dimension = RouteTiming;

	// Single window per city.
	Relaxed(std::span<const TimeWindow> windows, double penalty_weight = 1000) : store_(windows), penalty_weight_(penalty_weight) {}

	// Optional window per city (nullopt = unconstrained).
	Relaxed(std::span<const std::optional<TimeWindow>> windows, double penalty_weight = 1000) : store_(windows) , penalty_weight_(penalty_weight) {}

	// Multiple windows per city.
	Relaxed(std::span<const std::vector<TimeWindow>> windows, double penalty_weight = 1000) : store_(windows), penalty_weight_(penalty_weight) {}

	// A city's windows, sorted by earliest; empty for a city without one.
	template <typename CityT>
	[[nodiscard]] std::span<const TimeWindow> windows(CityT city) const {
		return store_[static_cast<std::size_t>(city)];
	}

	// --- AppendMove ---

	template <typename CityT, typename Ctx>
	void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const {
		auto ws = windows(m.city);
		double arr = ctx.template dim<RouteTiming>().arrival;
		ctx.template dim<RouteTiming>().departure += detail::departure_time(ws, arr) - arr;
		double violation = detail::violation_amount(ws, arr);
		if (violation > 0.0)
			ctx.cost_delta += penalty_weight_ * violation;
	}

	// --- DPMove ---

	template <typename CityT, typename Ctx>
	void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const {
		auto ws = windows(m.to);
		double arr = ctx.template dim<RouteTiming>().arrival;
		ctx.template dim<RouteTiming>().departure += detail::departure_time(ws, arr) - arr;
		double violation = detail::violation_amount(ws, arr);
		if (violation > 0.0)
			ctx.cost_delta += penalty_weight_ * violation;
	}

private:
	detail::WindowStore store_;
	double penalty_weight_;
};

} // namespace periple::time_windows

namespace periple::detail {

// Both wait for a window to open, and time_windows::detail::departure_time
// never returns less than the arrival.
template <> struct only_delays<time_windows::Strict> : std::true_type {};
template <> struct only_delays<time_windows::Relaxed> : std::true_type {};

} // namespace periple::detail

namespace periple::time_windows::detail {

// How many Strict a variant holds, and whether nothing in it makes a time
// earlier (periple::detail::only_delays), through compositions.
template <typename V>
struct timing {
	static constexpr std::size_t stricts = std::is_same_v<V, Strict> ? 1 : 0;
	static constexpr bool delays_only = periple::detail::only_delays<V>::value;
};

template <typename... Vs>
struct timing<Composed<Vs...>> {
	static constexpr std::size_t stricts = (timing<Vs>::stricts + ... + 0);
	static constexpr bool delays_only = (timing<Vs>::delays_only && ...);
};

// The variants under which the deadline test of DeadlineBound is exact, and
// branch_and_bound() applies it.
template <typename Variant>
inline constexpr bool deadline_test_exact = timing<Variant>::stricts == 1 && timing<Variant>::delays_only;

// The Strict of a variant that holds exactly one.
inline const Strict& strict_of(const Strict& windows) { return windows; }

template <typename... Vs>
const Strict& strict_of(const Composed<Vs...>& composed) {
	static constexpr bool holds[] = {(timing<Vs>::stricts > 0)...};
	constexpr auto i = static_cast<std::size_t>(std::find(holds, holds + sizeof...(Vs), true) - holds);
	return strict_of(composed.template component<i>());
}

// Per city, the end of its last window plus a margin, +infinity without one.
// The deadline test adds times in another order than RouteTiming, so it rounds
// differently: the margin keeps a tour that arrives just on time.
inline void due_times(const Strict& windows, std::span<double> out) {
	for (std::size_t c = 0; c < out.size(); ++c) {
		const auto ws = windows.windows(c);
		double latest = ws.empty() ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
		for (const auto& w : ws) latest = std::max(latest, w.latest);
		out[c] = latest + 1e-9 * (1.0 + std::abs(latest));
	}
}

} // namespace periple::time_windows::detail