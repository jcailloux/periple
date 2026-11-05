#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace periple::detail {

// Wall-clock budget of one algorithm call: only its end time. Each loop polls
// it through its own Poller, since the right polling stride depends on the
// cost of that loop's units of work.
class Deadline {
public:
	using Clock = std::chrono::steady_clock;

	explicit Deadline(std::chrono::nanoseconds limit)
		: end_(Clock::now() + std::chrono::duration_cast<Clock::duration>(limit)) {}

	[[nodiscard]] Clock::time_point end() const { return end_; }

private:
	Clock::time_point end_;
};

// No limit: nothing to poll.
struct NoDeadline {};

// Polls a deadline once per unit of work of one loop, whose units cost about
// the same. The clock is read at the first poll, then about every `interval`:
// the stride between reads doubles while reads come faster and halves while
// they come slower. A read costs about 26 ns; a limit is noticed within about
// twice `interval` plus one unit of work. A stride grown on cheap units would
// skip reads over costly ones, so every loop opens its own Poller.
template <typename D>
class Poller;

template <>
class Poller<Deadline> {
public:
	using Clock = Deadline::Clock;

	explicit Poller(const Deadline& deadline) : end_(deadline.end()) {}

	// True once the limit has passed, and on every later poll.
	[[nodiscard]] bool operator()() {
		if (expired_) return true;
		if (--countdown_ > 0) return false;
		const auto now = Clock::now();
		if (now >= end_) {
			expired_ = true;
			return true;
		}
		stride_ = (now - last_ < interval) ? std::min(stride_ * 2, max_stride)
		                                   : std::max(stride_ / 2, std::uint32_t{1});
		last_ = now;
		countdown_ = stride_;
		return false;
	}

private:
	static constexpr auto interval = std::chrono::microseconds(10);
	static constexpr std::uint32_t max_stride = std::uint32_t{1} << 20;

	bool expired_ = false;
	std::uint32_t stride_ = 1;
	std::uint32_t countdown_ = 1;  // the first poll reads the clock
	Clock::time_point last_{};     // none yet: the first read keeps a stride of 1
	Clock::time_point end_;
};

// Never stops: this version of an algorithm holds no clock read.
template <>
class Poller<NoDeadline> {
public:
	explicit Poller(const NoDeadline&) {}
	[[nodiscard]] constexpr bool operator()() const { return false; }
};

template <typename D>
Poller(const D&) -> Poller<D>;

// Runs fn with a Deadline when a limit is set and with a NoDeadline otherwise:
// the choice is made once per call, outside every loop.
template <typename Fn>
decltype(auto) with_deadline(std::chrono::nanoseconds limit, Fn&& fn) {
	if (limit.count() > 0) {
		Deadline deadline(limit);
		return fn(deadline);
	}
	NoDeadline deadline;
	return fn(deadline);
}

} // namespace periple::detail
