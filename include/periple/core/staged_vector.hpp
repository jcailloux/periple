#pragma once

// StagedVector -- non-destructive evaluation buffer for cumulative dimensions
//
// Drop-in replacement for std::vector in dimension types. Three internal
// buffers: committed (ground truth), staged (current evaluation), record
// (saved best candidate). Conditional operator[] routes reads/writes to
// the staged buffer during evaluation, leaving committed untouched.
//
// Staging lifecycle:
//   begin_staging(from)  -- activate staged buffer for positions >= from
//   discard_staging()    -- deactivate staged buffer (no copy)
//   save_staging(to)     -- O(1) swap staged<->record, deactivate staging
//   commit(to)           -- copy record (or staging) into committed
//   discard_record()     -- drop saved record without committing

#include <algorithm>
#include <cstddef>
#include <vector>

namespace periple {

template <typename T>
class StagedVector {
public:
	T& operator[](std::size_t i) {
		return (staging_ && i >= stage_from_) ? staged_[i] : committed_[i];
	}
	const T& operator[](std::size_t i) const {
		return (staging_ && i >= stage_from_) ? staged_[i] : committed_[i];
	}

	// Ground truth even while staging is active; dimension code uses operator[].
	const T& committed(std::size_t i) const { return committed_[i]; }

	void resize(std::size_t n) {
		committed_.resize(n);
		staged_.resize(n);
		record_.resize(n);
		staging_ = false;
		has_record_ = false;
	}

	std::size_t size() const { return committed_.size(); }

	void fill(const T& v) { std::fill(committed_.begin(), committed_.end(), v); }

	void begin_staging(std::size_t from) {
		staging_ = true;
		stage_from_ = from;
	}

	void discard_staging() { staging_ = false; }

	void save_staging(std::size_t to) {
		record_from_ = stage_from_;
		record_to_ = to;
		std::swap(staged_, record_);
		has_record_ = true;
		staging_ = false;
	}

	void commit(std::size_t to) {
		if (has_record_) {
			std::copy(
			    record_.begin() + static_cast<std::ptrdiff_t>(record_from_),
			    record_.begin() + static_cast<std::ptrdiff_t>(record_to_),
			    committed_.begin() + static_cast<std::ptrdiff_t>(record_from_));
			has_record_ = false;
		} else if (staging_) {
			std::copy(
			    staged_.begin() + static_cast<std::ptrdiff_t>(stage_from_),
                staged_.begin() + static_cast<std::ptrdiff_t>(to),
                committed_.begin() + static_cast<std::ptrdiff_t>(stage_from_));
			staging_ = false;
		}
	}

	void discard_record() { has_record_ = false; }

	bool staging_active() const { return staging_ || has_record_; }

private:
	std::vector<T> committed_;
	std::vector<T> staged_;
	std::vector<T> record_;
	std::size_t stage_from_ = 0;
	std::size_t record_from_ = 0;
	std::size_t record_to_ = 0;
	bool staging_ = false;
	bool has_record_ = false;
};

} // namespace periple