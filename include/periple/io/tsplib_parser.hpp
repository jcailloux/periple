#pragma once

#include <periple/distance/coordinate.hpp>
#include <periple/distance/functions.hpp>
#include <periple/distance/matrix.hpp>

#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace periple {
namespace detail {

enum class EdgeWeightType { euc_2d, geo, att, ceil_2d, explicit_matrix };
enum class EdgeWeightFormat { lower_diag_row, upper_row, full_matrix, none };

// Trim trailing whitespace / carriage returns
inline std::string trim(const std::string& s) {
	auto end = s.find_last_not_of(" \t\r\n");
	return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}

// Strip everything up to and including ':' then trim both ends
inline std::string parse_value(const std::string& line) {
	auto pos = line.find(':');
	std::string val = (pos == std::string::npos) ? line : line.substr(pos + 1);
	auto start = val.find_first_not_of(" \t\r\n");
	if (start == std::string::npos) return "";
	auto end = val.find_last_not_of(" \t\r\n");
	return val.substr(start, end - start + 1);
}

inline EdgeWeightType parse_weight_type(const std::string& s) {
	if (s == "EUC_2D") return EdgeWeightType::euc_2d;
	if (s == "GEO") return EdgeWeightType::geo;
	if (s == "ATT") return EdgeWeightType::att;
	if (s == "CEIL_2D") return EdgeWeightType::ceil_2d;
	if (s == "EXPLICIT") return EdgeWeightType::explicit_matrix;
	throw std::runtime_error("unsupported EDGE_WEIGHT_TYPE: " + s);
}

inline EdgeWeightFormat parse_weight_format(const std::string& s) {
	if (s == "LOWER_DIAG_ROW") return EdgeWeightFormat::lower_diag_row;
	if (s == "UPPER_ROW") return EdgeWeightFormat::upper_row;
	if (s == "FULL_MATRIX") return EdgeWeightFormat::full_matrix;
	if (s == "FUNCTION") return EdgeWeightFormat::none; // computed from coords
	throw std::runtime_error("unsupported EDGE_WEIGHT_FORMAT: " + s);
}

// Distance function dispatch. Both paths use periple::tsplib:: functions.
using CoordDistFn = int (*)(const double*, const double*);

inline CoordDistFn coord_distance_function(EdgeWeightType type) {
	switch (type) {
	case EdgeWeightType::euc_2d:  return tsplib::euc_2d;
	case EdgeWeightType::geo:     return tsplib::geo;
	case EdgeWeightType::att:     return tsplib::att;
	case EdgeWeightType::ceil_2d: return tsplib::ceil_2d;
	default:
		throw std::runtime_error("no distance function for EXPLICIT type");
	}
}

struct Coord {
	double x, y;
};

inline std::vector<Coord> read_node_coords(std::istream& in, std::size_t n) {
	std::vector<Coord> coords(n);
	for (std::size_t i = 0; i < n; ++i) {
		std::size_t index;
		in >> index >> coords[i].x >> coords[i].y;
		if (!in)
			throw std::runtime_error(
				"unexpected end of NODE_COORD_SECTION at node " +
				std::to_string(i));
	}
	return coords;
}

// Flatten Coord vector into [x0,y0, x1,y1, ...] for CoordinateDistance.
inline std::vector<double> flatten_coords(const std::vector<Coord>& coords) {
	std::vector<double> flat(coords.size() * 2);
	for (std::size_t i = 0; i < coords.size(); ++i) {
		flat[i * 2]     = coords[i].x;
		flat[i * 2 + 1] = coords[i].y;
	}
	return flat;
}

inline SymmetricDistanceMatrix<int> build_from_coords(
	const std::vector<double>& flat, std::size_t n, CoordDistFn dist_fn)
{
	SymmetricDistanceMatrix<int> mat(n);
	for (std::size_t i = 1; i < n; ++i)
		for (std::size_t j = 0; j < i; ++j)
			mat.set(i, j,
				dist_fn(flat.data() + i * 2, flat.data() + j * 2));
	return mat;
}

inline SymmetricDistanceMatrix<int> read_lower_diag_row(
	std::istream& in, std::size_t n)
{
	SymmetricDistanceMatrix<int> mat(n);
	for (std::size_t i = 0; i < n; ++i) {
		for (std::size_t j = 0; j <= i; ++j) {
			int v;
			if (!(in >> v))
				throw std::runtime_error(
					"unexpected end of EDGE_WEIGHT_SECTION (LOWER_DIAG_ROW)");
			if (i != j) mat.set(i, j, v);
		}
	}
	return mat;
}

inline SymmetricDistanceMatrix<int> read_upper_row(
	std::istream& in, std::size_t n)
{
	SymmetricDistanceMatrix<int> mat(n);
	for (std::size_t i = 0; i < n; ++i) {
		for (std::size_t j = i + 1; j < n; ++j) {
			int v;
			if (!(in >> v))
				throw std::runtime_error(
					"unexpected end of EDGE_WEIGHT_SECTION (UPPER_ROW)");
			mat.set(i, j, v);
		}
	}
	return mat;
}

inline SymmetricDistanceMatrix<int> read_full_matrix(
	std::istream& in, std::size_t n)
{
	SymmetricDistanceMatrix<int> mat(n);
	for (std::size_t i = 0; i < n; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			int v;
			if (!(in >> v))
				throw std::runtime_error(
					"unexpected end of EDGE_WEIGHT_SECTION (FULL_MATRIX)");
			if (i > j) mat.set(i, j, v);
		}
	}
	return mat;
}


// Shared header parsing: reads DIMENSION, EDGE_WEIGHT_TYPE, EDGE_WEIGHT_FORMAT,
// then stops at the data section keyword, returning what was found.
struct TsplibHeader {
	std::size_t dimension = 0;
	EdgeWeightType weight_type{};
	EdgeWeightFormat weight_format = EdgeWeightFormat::none;
	bool has_type = false;
	enum class DataSection { node_coord, edge_weight, none } data_section = DataSection::none;
};

inline TsplibHeader parse_tsplib_header(std::istream& in) {
	TsplibHeader h;
	std::string line;
	while (std::getline(in, line)) {
		line = trim(line);
		if (line.empty()) continue;

		if (line.starts_with("DIMENSION")) {
			h.dimension = std::stoul(parse_value(line));
		} else if (line.starts_with("EDGE_WEIGHT_TYPE")) {
			h.weight_type = parse_weight_type(parse_value(line));
			h.has_type = true;
		} else if (line.starts_with("EDGE_WEIGHT_FORMAT")) {
			h.weight_format = parse_weight_format(parse_value(line));
		} else if (line.starts_with("NODE_COORD_SECTION")) {
			h.data_section = TsplibHeader::DataSection::node_coord;
			break;
		} else if (line.starts_with("EDGE_WEIGHT_SECTION")) {
			h.data_section = TsplibHeader::DataSection::edge_weight;
			break;
		}
	}
	if (h.dimension == 0)
		throw std::runtime_error("DIMENSION missing or zero");
	if (!h.has_type)
		throw std::runtime_error("EDGE_WEIGHT_TYPE missing");
	if (h.data_section == TsplibHeader::DataSection::none)
		throw std::runtime_error(
			"no data section found (NODE_COORD_SECTION or EDGE_WEIGHT_SECTION)");
	return h;
}

inline SymmetricDistanceMatrix<int> load_tsplib_matrix_impl(std::istream& in) {
	auto h = parse_tsplib_header(in);
	if (h.data_section == TsplibHeader::DataSection::node_coord) {
		auto coords = read_node_coords(in, h.dimension);
		auto flat = flatten_coords(coords);
		return build_from_coords(flat, h.dimension,
			coord_distance_function(h.weight_type));
	}
	// EDGE_WEIGHT_SECTION
	if (h.weight_format == EdgeWeightFormat::none)
		throw std::runtime_error(
			"EDGE_WEIGHT_FORMAT missing for EXPLICIT type");
	switch (h.weight_format) {
	case EdgeWeightFormat::lower_diag_row:
		return read_lower_diag_row(in, h.dimension);
	case EdgeWeightFormat::upper_row:
		return read_upper_row(in, h.dimension);
	case EdgeWeightFormat::full_matrix:
		return read_full_matrix(in, h.dimension);
	default:
		throw std::runtime_error("unexpected weight format");
	}
}

using TsplibCoordDist = CoordinateDistance<CoordDistFn>;

inline TsplibCoordDist load_tsplib_coords_impl(std::istream& in) {
	auto h = parse_tsplib_header(in);
	if (h.data_section != TsplibHeader::DataSection::node_coord)
		throw std::runtime_error(
			"load_tsplib_coords requires coordinate-based instances, "
			"not EXPLICIT matrices");
	auto coords = read_node_coords(in, h.dimension);
	return TsplibCoordDist(
		2, flatten_coords(coords),
		coord_distance_function(h.weight_type));
}

} // namespace detail

// Load a TSPLIB95 file into a precomputed SymmetricDistanceMatrix<int>.
// Supports all EDGE_WEIGHT_TYPEs (EUC_2D, GEO, ATT, CEIL_2D, EXPLICIT).
// Uses O(N^2) memory -- suitable for small/medium instances.
inline SymmetricDistanceMatrix<int> load_tsplib(const std::string& filepath) {
	std::ifstream file(filepath);
	if (!file)
		throw std::runtime_error("cannot open file: " + filepath);
	return detail::load_tsplib_matrix_impl(file);
}

inline SymmetricDistanceMatrix<int> load_tsplib(std::istream& in) {
	return detail::load_tsplib_matrix_impl(in);
}

// Load a coordinate-based TSPLIB95 file into a CoordinateDistance.
// Computes distances on the fly -- O(N) memory, suitable for any instance size.
// Throws if the file uses EXPLICIT matrices (no coordinates available).
inline detail::TsplibCoordDist load_tsplib_coords(const std::string& filepath) {
	std::ifstream file(filepath);
	if (!file)
		throw std::runtime_error("cannot open file: " + filepath);
	return detail::load_tsplib_coords_impl(file);
}

inline detail::TsplibCoordDist load_tsplib_coords(std::istream& in) {
	return detail::load_tsplib_coords_impl(in);
}

} // namespace periple