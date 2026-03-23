#pragma once

#include <periple/core/callbacks.hpp>
#include <periple/core/traits.hpp>

#include <concepts>
#include <span>

namespace periple {

struct NearestSelector {
	template <DistanceSource Dist, typename Callbacks>
	auto evaluate(const Dist& dist,
	              std::span<const typename dist_traits<Dist>::city_type> tour,
	              typename dist_traits<Dist>::city_type candidate,
	              const Callbacks& cb) const
		-> typename dist_traits<Dist>::cost_type
	{
		using city_type = typename dist_traits<Dist>::city_type;
		using cost_type = typename dist_traits<Dist>::cost_type;

		if constexpr (requires(const Callbacks& c, std::span<const city_type> t,
		                       const AppendMove<city_type>& am) {
			{ c.move_score(t, am) } -> std::convertible_to<cost_type>;
		}) {
			return cb.move_score(tour, AppendMove<city_type>{candidate});
		} else if constexpr (requires(const Callbacks& c, std::span<const city_type> t,
		                              const AppendMove<city_type>& am) {
			{ c.move_eval(t, am) } -> std::convertible_to<cost_type>;
		}) {
			return cb.move_eval(tour, AppendMove<city_type>{candidate});
		} else {
			return dist(tour.back(), candidate);
		}
	}
};

} // namespace periple
