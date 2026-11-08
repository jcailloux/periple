#pragma once

// Nearest neighbor construction heuristic: greedy_construct with NearestSelector
// (greedy_construct/nearest_selector.hpp)

#include <periple/algorithms/greedy_construct/nearest_selector.hpp>

namespace periple {

// ---------------------------------------------------------------------------
// Solver::nearest_neighbor -- facade using NearestSelector
// ---------------------------------------------------------------------------

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::nearest_neighbor(NearestNeighborParams params) -> Solver& {
	return greedy_construct(NearestSelector{}, ConstructParams{.start_city = params.start_city, .resume_at = params.resume_at, .time_limit = params.time_limit});
}

} // namespace periple
