#pragma once

#include <periple/core/traits.hpp>

#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <vector>

namespace periple {

// ---------------------------------------------------------------------------
// DistanceMatrix --full square storage (supports asymmetric instances)
// ---------------------------------------------------------------------------

template <typename CostT = double, typename CityT = std::size_t>
class DistanceMatrix {
public:
	using cost_type = CostT;
	using city_type = CityT;

	DistanceMatrix() = default;

	explicit DistanceMatrix(std::size_t n)
		: n_(n), data_(n * n, cost_type{}) {}

	DistanceMatrix(std::size_t n, std::initializer_list<cost_type> values)
		: n_(n), data_(values) {
		assert(data_.size() == n * n);
	}

	DistanceMatrix(std::size_t n, std::vector<cost_type> values)
		: n_(n), data_(std::move(values)) {
		assert(data_.size() == n * n);
	}

	explicit DistanceMatrix(const std::vector<std::vector<cost_type>>& rows)
		: n_(rows.size()), data_(rows.size() * rows.size()) {
		for (std::size_t i = 0; i < n_; ++i) {
			assert(rows[i].size() == n_);
			for (std::size_t j = 0; j < n_; ++j)
				data_[i * n_ + j] = rows[i][j];
		}
	}

	cost_type operator()(city_type i, city_type j) const {
		return data_[static_cast<std::size_t>(i) * n_ +
					 static_cast<std::size_t>(j)];
	}

	cost_type& operator()(city_type i, city_type j) {
		return data_[static_cast<std::size_t>(i) * n_ +
					 static_cast<std::size_t>(j)];
	}

	[[nodiscard]] std::size_t size() const { return n_; }

private:
	std::size_t n_ = 0;
	std::vector<cost_type> data_;
};

// ---------------------------------------------------------------------------
// SymmetricDistanceMatrix --lower-triangular storage (half memory)
// ---------------------------------------------------------------------------

template <typename CostT = double, typename CityT = std::size_t>
class SymmetricDistanceMatrix {
public:
	using cost_type = CostT;
	using city_type = CityT;

	SymmetricDistanceMatrix() = default;

	explicit SymmetricDistanceMatrix(std::size_t n)
		: n_(n), data_(n * (n - 1) / 2, cost_type{}) {}

	explicit SymmetricDistanceMatrix(
		const std::vector<std::vector<cost_type>>& rows)
		: n_(rows.size()), data_(rows.size() * (rows.size() - 1) / 2) {
		for (std::size_t i = 1; i < n_; ++i) {
			assert(rows[i].size() == n_);
			for (std::size_t j = 0; j < i; ++j)
				data_[tri(i, j)] = rows[i][j];
		}
	}

	cost_type operator()(city_type i, city_type j) const {
		auto a = static_cast<std::size_t>(i);
		auto b = static_cast<std::size_t>(j);
		if (a == b) return cost_type{};
		return data_[a > b ? tri(a, b) : tri(b, a)];
	}

	void set(city_type i, city_type j, cost_type v) {
		auto a = static_cast<std::size_t>(i);
		auto b = static_cast<std::size_t>(j);
		assert(a != b);
		data_[a > b ? tri(a, b) : tri(b, a)] = v;
	}

	[[nodiscard]] std::size_t size() const { return n_; }

private:
	// Strict lower triangle index: row > col
	static std::size_t tri(std::size_t row, std::size_t col) {
		return row * (row - 1) / 2 + col;
	}

	std::size_t n_ = 0;
	std::vector<cost_type> data_;
};

} // namespace periple