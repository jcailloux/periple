#pragma once

#include <periple/core/solver.hpp>
#include <periple/algorithms/nearest_neighbor.hpp>
#include <periple/algorithms/held_karp.hpp>

#include <cstdio>
#include <cstring>
#include <tuple>

namespace periple {

// ---------------------------------------------------------------------------
// Algorithm descriptors -- one struct per algorithm
// ---------------------------------------------------------------------------

struct AlgoNearestNeighbor {
	static constexpr const char* tag  = "NN";
	static constexpr const char* name = "nearest_neighbor";
	static constexpr bool is_exact          = false;
	static constexpr bool symmetric_only    = false;
	static constexpr bool is_metaheuristic  = false;
	static constexpr int  max_tier          = 5;

	template <DistanceSource Dist, typename TC>
	void operator()(Solver<Dist, TC>& s, unsigned = 0) const { s.nearest_neighbor(); }

	template <DistanceSource Dist, typename TC, typename Variant>
	void operator()(Solver<Dist, TC>& s, const Variant& variant) const { s.nearest_neighbor(variant); }
};

struct AlgoHeldKarp {
	static constexpr const char* tag  = "HK";
	static constexpr const char* name = "held_karp";
	static constexpr bool is_exact          = true;
	static constexpr bool symmetric_only    = false;
	static constexpr bool is_metaheuristic  = false;
	static constexpr int  max_tier          = 1;

	template <DistanceSource Dist, typename TC>
	void operator()(Solver<Dist, TC>& s, unsigned = 0) const { s.held_karp(); }

	template <DistanceSource Dist, typename TC, typename Variant>
	void operator()(Solver<Dist, TC>& s, const Variant& variant) const { s.held_karp(variant); }
};

// Master list -- add new algorithms here.
using AllAlgorithms = std::tuple<AlgoNearestNeighbor, AlgoHeldKarp>;

// ---------------------------------------------------------------------------
// Iteration helpers
// ---------------------------------------------------------------------------

namespace detail {

inline bool tag_matches(const char* tag, const char* filter) {
	if (!filter || !filter[0]) return true;
	auto len = std::strlen(tag);
	const char* pos = filter;
	while (*pos) {
		if (std::strncmp(pos, tag, len) == 0 &&
		    (pos[len] == ',' || pos[len] == '\0'))
			return true;
		pos = std::strchr(pos, ',');
		if (!pos) break;
		++pos;
	}
	return false;
}

} // namespace detail

// Call fn(algo) for each algorithm whose tag is in the comma-separated filter.
// A null or empty filter matches all algorithms.
// Returns the number of algorithms matched.
template <typename F>
int for_each_algorithm(const char* filter, F&& fn) {
	int count = 0;
	std::apply([&](auto... algos) {
		((detail::tag_matches(algos.tag, filter) ?
			(fn(algos), ++count, 0) : 0), ...);
	}, AllAlgorithms{});
	return count;
}

inline void print_algorithm_tags(std::FILE* out = stderr) {
	std::fprintf(out, "Available algorithm tags:");
	std::apply([out](auto... algos) {
		((std::fprintf(out, " %s(%s)", algos.tag, algos.name)), ...);
	}, AllAlgorithms{});
	std::fprintf(out, "\n");
}

} // namespace periple