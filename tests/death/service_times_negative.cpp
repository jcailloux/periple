// A negative service time would make a departure come before its arrival,
// which the deadline test of branch_and_bound() rules out.
#include <periple/periple.hpp>

int main() {
	const double durations[] = {0.0, -1.0, 2.0};
	periple::service_times::ServiceTimes services(durations);
	(void)services;
}
