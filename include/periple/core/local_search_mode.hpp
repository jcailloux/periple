#pragma once

// LocalSearchMode -- how a local search evaluates and applies segment moves
//
// Deduced once per call from the solver itself, never configurable: a wrong
// mode gives a wrong cost, not a slower run. A variant forces replay, because
// its penalties depend on the traversal order; without one, the symmetric path
// is taken only when the caller declared the matrix symmetric (set_symmetric).
//
// Lives outside the algorithm headers because it parameterizes private member
// templates of Solver, whose type must be complete before the class.

namespace periple::detail {

enum class LocalSearchMode { symmetric, asymmetric, replay };

} // namespace periple::detail