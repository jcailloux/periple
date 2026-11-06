#pragma once

#include <periple/core/solver.hpp>
#include <periple/algorithms/nearest_neighbor.hpp>
#include <periple/algorithms/held_karp.hpp>
#include <periple/algorithms/two_opt.hpp>
#include <periple/algorithms/branch_and_bound.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <tuple>

namespace periple {

// ---------------------------------------------------------------------------
// Algorithm descriptors -- one struct per algorithm
// ---------------------------------------------------------------------------

// Options every descriptor forwards to its algorithm.
struct RunOptions {
	unsigned seed = 0;                        // metaheuristics
	std::chrono::nanoseconds time_limit{0};   // the algorithm's own time_limit (0 = no limit)
};

// Base of every descriptor. run is the algorithm under test; setup, optional,
// builds the tour an improvement algorithm starts from, outside its time limit.
template <typename Derived>
struct Algorithm {
	template <DistanceSource Dist, typename V>
	void operator()(Solver<Dist, V>& s, const RunOptions& options = {}) const {
		const auto& self = static_cast<const Derived&>(*this);
		if constexpr (requires { self.setup(s); }) self.setup(s);
		self.run(s, options);
	}
};

struct AlgoNearestNeighbor : Algorithm<AlgoNearestNeighbor> {
	static constexpr const char* tag  = "NN";
	static constexpr const char* name = "nearest_neighbor";
	static constexpr bool is_exact          = false;
	static constexpr bool symmetric_only    = false;
	static constexpr bool is_metaheuristic  = false;
	static constexpr int  max_tier          = 5;

	template <DistanceSource Dist, typename V>
	void run(Solver<Dist, V>& s, const RunOptions& o) const { s.nearest_neighbor({.time_limit = o.time_limit}); }
};

struct AlgoHeldKarp : Algorithm<AlgoHeldKarp> {
	static constexpr const char* tag  = "HK";
	static constexpr const char* name = "held_karp";
	static constexpr bool is_exact          = true;
	static constexpr bool symmetric_only    = false;
	static constexpr bool is_metaheuristic  = false;
	static constexpr int  max_tier          = 1;

	template <DistanceSource Dist, typename V>
	void run(Solver<Dist, V>& s, const RunOptions& o) const { s.held_karp({.time_limit = o.time_limit}); }
};

// Improvement pipeline: nearest neighbor construction followed by 2-opt.
// A strict variant can leave the construction partial or infeasible, in which
// case there is nothing to improve.
struct AlgoNNTwoOpt : Algorithm<AlgoNNTwoOpt> {
	static constexpr const char* tag  = "2O";
	static constexpr const char* name = "nn_two_opt";
	static constexpr bool is_exact          = false;
	static constexpr bool symmetric_only    = false;
	static constexpr bool is_metaheuristic  = false;
	static constexpr int  max_tier          = 5;

	template <DistanceSource Dist, typename V>
	void setup(Solver<Dist, V>& s) const { s.nearest_neighbor(); }

	template <DistanceSource Dist, typename V>
	void run(Solver<Dist, V>& s, const RunOptions& o) const {
		if (s.status() == SolutionStatus::feasible) s.two_opt({.time_limit = o.time_limit});
	}
};

struct AlgoBranchAndBound : Algorithm<AlgoBranchAndBound> {
	static constexpr const char* tag  = "BB";
	static constexpr const char* name = "branch_and_bound";
	static constexpr bool is_exact          = true;
	static constexpr bool symmetric_only    = false;
	static constexpr bool is_metaheuristic  = false;
	static constexpr int  max_tier          = 1;

	template <DistanceSource Dist, typename V>
	void run(Solver<Dist, V>& s, const RunOptions& o) const { s.branch_and_bound({.time_limit = o.time_limit}); }
};

// Master list -- add new algorithms here.
using AllAlgorithms = std::tuple<AlgoNearestNeighbor, AlgoHeldKarp, AlgoNNTwoOpt, AlgoBranchAndBound>;

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
