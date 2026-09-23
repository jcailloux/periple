#pragma once

#include <periple/periple.hpp>
#include <periple/io/tsplib_parser.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

namespace bench {

// ---------------------------------------------------------------------------
// Instance definition
// ---------------------------------------------------------------------------

struct InstanceDef {
	const char* name;
	const char* subpath;
	int tier;
	bool is_explicit; // EXPLICIT instances have no coordinates
};

constexpr InstanceDef all_instances[] = {
	// Tier 1
	{"burma14",   "tier1/burma14.tsp",   1, false},
	{"ulysses16", "tier1/ulysses16.tsp", 1, false},
	{"gr17",      "tier1/gr17.tsp",      1, true},
	{"gr21",      "tier1/gr21.tsp",      1, true},
	{"ulysses22", "tier1/ulysses22.tsp", 1, false},
	{"gr24",      "tier1/gr24.tsp",      1, true},
	// Tier 2
	{"att48",     "tier2/att48.tsp",     2, false},
	{"berlin52",  "tier2/berlin52.tsp",  2, false},
	{"st70",      "tier2/st70.tsp",      2, false},
	{"eil76",     "tier2/eil76.tsp",     2, false},
	{"kroA100",   "tier2/kroA100.tsp",   2, false},
	{"kroB100",   "tier2/kroB100.tsp",   2, false},
	{"ch130",     "tier2/ch130.tsp",     2, false},
	{"ch150",     "tier2/ch150.tsp",     2, false},
	{"kroA200",   "tier2/kroA200.tsp",   2, false},
	// Tier 3
	{"a280",      "tier3/a280.tsp",      3, false},
	{"lin318",    "tier3/lin318.tsp",    3, false},
	{"pcb442",    "tier3/pcb442.tsp",    3, false},
	{"att532",    "tier3/att532.tsp",    3, false},
	{"rat575",    "tier3/rat575.tsp",    3, false},
	{"rat783",    "tier3/rat783.tsp",    3, false},
	// Tier 4
	{"pr1002",    "tier4/pr1002.tsp",    4, false},
	{"u1817",     "tier4/u1817.tsp",     4, false},
	{"pr2392",    "tier4/pr2392.tsp",    4, false},
	{"pcb3038",   "tier4/pcb3038.tsp",   4, false},
	{"fnl4461",   "tier4/fnl4461.tsp",   4, false},
	{"rl5934",    "tier4/rl5934.tsp",    4, false},
	// Tier 5
	{"usa13509",  "tier5/usa13509.tsp",  5, false},
	{"d18512",    "tier5/d18512.tsp",    5, false},
	{"pla33810",  "tier5/pla33810.tsp",  5, false},
	{"pla85900",  "tier5/pla85900.tsp",  5, false},
};

// ---------------------------------------------------------------------------
// Optima loading (minimal JSON parser for our rigid format)
// ---------------------------------------------------------------------------

struct Optimum {
	long long cost;
	bool proven;
};

inline std::unordered_map<std::string, Optimum> load_optima(const std::string& path) {
	std::ifstream f(path);
	if (!f) throw std::runtime_error("cannot open optima file: " + path);
	std::string content((std::istreambuf_iterator<char>(f)), {});

	std::unordered_map<std::string, Optimum> result;
	std::size_t pos = 0;
	while (pos < content.size()) {
		auto q1 = content.find('"', pos);
		if (q1 == std::string::npos) break;
		auto q2 = content.find('"', q1 + 1);
		std::string name = content.substr(q1 + 1, q2 - q1 - 1);

		auto cost_key = content.find("\"cost\"", q2);
		if (cost_key == std::string::npos) break;
		auto colon = content.find(':', cost_key + 6);
		long long cost = std::stoll(content.substr(colon + 1));

		auto proven_key = content.find("\"proven\"", colon);
		if (proven_key == std::string::npos) break;
		auto pcolon = content.find(':', proven_key + 8);
		auto pval = content.find_first_not_of(" \t\n\r", pcolon + 1);
		bool proven = content[pval] == 't';

		result[name] = {cost, proven};
		pos = content.find('}', pval) + 1;
	}
	return result;
}

// ---------------------------------------------------------------------------
// Tour validation
// ---------------------------------------------------------------------------

struct ValidationResult {
	bool valid = true;
	long long recomputed_cost = 0;
	std::string error;
};

template <periple::DistanceSource Dist>
ValidationResult validate_tour(
	const Dist& dist,
	std::span<const typename periple::dist_traits<Dist>::city_type> tour,
	typename periple::dist_traits<Dist>::cost_type reported_cost)
{
	using city_type = typename periple::dist_traits<Dist>::city_type;
	using cost_type = typename periple::dist_traits<Dist>::cost_type;

	ValidationResult r;
	std::size_t n = dist.size();

	if (tour.size() != n) {
		r.valid = false;
		r.error = "tour size " + std::to_string(tour.size())
				+ " != " + std::to_string(n);
		return r;
	}

	// Check Hamiltonian cycle: sort a copy, must be {0, 1, ..., n-1}
	std::vector<city_type> sorted(tour.begin(), tour.end());
	std::sort(sorted.begin(), sorted.end());
	for (std::size_t i = 0; i < n; ++i) {
		if (static_cast<std::size_t>(sorted[i]) != i) {
			r.valid = false;
			r.error = "not a valid Hamiltonian cycle";
			return r;
		}
	}

	// Recompute cost
	cost_type total{};
	for (std::size_t i = 0; i < n; ++i)
		total += dist(tour[i], tour[(i + 1) % n]);
	r.recomputed_cost = static_cast<long long>(total);

	if (total != reported_cost) {
		r.valid = false;
		r.error = "reported cost " + std::to_string(reported_cost)
				+ " != recomputed " + std::to_string(total);
	}
	return r;
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

struct TimingResult {
	double wall_time_ms;
	double cpu_time_ms;
	int repeats;
};

inline long peak_rss_kb() {
#ifdef __linux__
	std::ifstream f("/proc/self/status");
	std::string line;
	while (std::getline(f, line)) {
		if (line.starts_with("VmHWM:")) {
			return std::stol(line.substr(6));
		}
	}
#endif
	return 0;
}

// ---------------------------------------------------------------------------
// Algorithm registration
// ---------------------------------------------------------------------------

using MatrixDist = periple::SymmetricDistanceMatrix<int>;
using CoordDist  = periple::detail::TsplibCoordDist;

struct AlgorithmEntry {
	std::string name;
	bool is_exact;
	int max_tier;
	int runs;
	std::function<void(periple::Solver<MatrixDist>&, unsigned)> run_matrix;
	std::function<void(periple::Solver<CoordDist>&, unsigned)>  run_coords;
};

template <typename F>
AlgorithmEntry make_algorithm(
	std::string name, bool exact, int max_tier, int runs, F fn)
{
	return {
		std::move(name), exact, max_tier, runs,
		[fn](periple::Solver<MatrixDist>& s, unsigned seed) { fn(s, seed); },
		[fn](periple::Solver<CoordDist>& s, unsigned seed) { fn(s, seed); }
	};
}

// ---------------------------------------------------------------------------
// Benchmark result
// ---------------------------------------------------------------------------

struct BenchmarkResult {
	std::string algorithm;
	std::string instance;
	int tier;
	std::size_t n;
	long long optimum;
	long long cost;         // best cost (= only cost for single-run)
	double gap_pct;
	double wall_time_ms;
	double cpu_time_ms;
	double peak_rss_mb;
	std::string status;
	bool valid;
	std::string error;
	int runs = 1;
	double mean_cost = 0;
	double stddev_cost = 0;
};

inline const char* status_string(periple::SolutionStatus s) {
	switch (s) {
	case periple::SolutionStatus::optimal:  return "optimal";
	case periple::SolutionStatus::feasible: return "feasible";
	default: return "none";
	}
}

// ---------------------------------------------------------------------------
// Multi-run statistics
// ---------------------------------------------------------------------------

inline void fill_multi_run_stats(
	BenchmarkResult& r,
	const std::vector<long long>& costs,
	std::vector<double>& times)
{
	r.cost = *std::min_element(costs.begin(), costs.end());

	double sum = 0;
	for (auto c : costs) sum += static_cast<double>(c);
	r.mean_cost = sum / static_cast<double>(costs.size());

	double sq_sum = 0;
	for (auto c : costs) {
		double d = static_cast<double>(c) - r.mean_cost;
		sq_sum += d * d;
	}
	r.stddev_cost = std::sqrt(sq_sum / static_cast<double>(costs.size()));

	std::sort(times.begin(), times.end());
	r.wall_time_ms = times[times.size() / 2];
}

// ---------------------------------------------------------------------------
// Runner
// ---------------------------------------------------------------------------

class BenchmarkRunner {
public:
	BenchmarkRunner(
		std::string instances_dir,
		std::string optima_path,
		std::vector<AlgorithmEntry> algorithms)
		: instances_dir_(std::move(instances_dir))
		, algorithms_(std::move(algorithms))
		, optima_(load_optima(optima_path))
	{}

	// Filter by tier range (inclusive)
	void set_tier_range(int min_tier, int max_tier) {
		min_tier_ = min_tier;
		max_tier_ = max_tier;
	}

	std::vector<BenchmarkResult> run_all() {
		std::vector<BenchmarkResult> results;
		for (const auto& algo : algorithms_) {
			for (const auto& inst : all_instances) {
				if (inst.tier < min_tier_ || inst.tier > max_tier_)
					continue;
				if (inst.tier > algo.max_tier)
					continue;

				std::string path = instances_dir_ + "/" + inst.subpath;
				std::ifstream test(path);
				if (!test) {
					std::fprintf(stderr, "  SKIP %s (file not found)\n",
						inst.name);
					continue;
				}
				test.close();

				auto r = run_one(algo, inst, path);
				results.push_back(std::move(r));
			}
		}
		return results;
	}

	// --- Output ---

	static void print_table(const std::vector<BenchmarkResult>& results) {
		std::string current_algo;
		for (const auto& r : results) {
			if (r.algorithm != current_algo) {
				current_algo = r.algorithm;
				std::printf("\n%s\n", current_algo.c_str());
				std::printf("%-14s %5s %10s %10s %8s %10s %5s %s\n",
					"Instance", "N", "Optimum", "Cost",
					"Gap(%)", "Time(ms)", "Valid", "Status");
			}
			std::printf("%-14s %5zu %10lld %10lld %8.2f %10.2f %5s %s",
				r.instance.c_str(), r.n, r.optimum, r.cost,
				r.gap_pct, r.wall_time_ms,
				r.valid ? "yes" : "NO",
				r.status.c_str());
			if (r.runs > 1)
				std::printf("  [%d runs, mean %.0f, stddev %.1f]",
					r.runs, r.mean_cost, r.stddev_cost);
			if (!r.error.empty())
				std::printf("  [%s]", r.error.c_str());
			std::printf("\n");
		}
	}

	static void print_comparison(const std::vector<BenchmarkResult>& results) {
		// Collect unique algorithms and instances (in order)
		std::vector<std::string> algos, instances;
		for (const auto& r : results) {
			if (std::find(algos.begin(), algos.end(), r.algorithm) == algos.end())
				algos.push_back(r.algorithm);
			if (std::find(instances.begin(), instances.end(), r.instance) == instances.end())
				instances.push_back(r.instance);
		}

		// Build lookup: (algo, instance) -> gap
		std::unordered_map<std::string, double> gaps;
		std::unordered_map<std::string, long long> optima;
		for (const auto& r : results) {
			gaps[r.algorithm + ":" + r.instance] = r.gap_pct;
			optima[r.instance] = r.optimum;
		}

		// Header
		std::printf("\n%-14s", "Instance");
		for (const auto& a : algos)
			std::printf(" %12s", a.c_str());
		std::printf(" %10s\n", "Optimum");

		for (const auto& inst : instances) {
			std::printf("%-14s", inst.c_str());
			for (const auto& a : algos) {
				auto it = gaps.find(a + ":" + inst);
				if (it != gaps.end())
					std::printf(" %+11.2f%%", it->second);
				else
					std::printf(" %12s", "N/A");
			}
			std::printf(" %10lld\n", optima[inst]);
		}
	}

	static void write_json(
		const std::vector<BenchmarkResult>& results,
		const std::string& path,
		const std::string& commit)
	{
		std::ofstream f(path);
		if (!f) {
			std::fprintf(stderr, "Cannot write JSON to %s\n", path.c_str());
			return;
		}

		// Timestamp
		auto now = std::chrono::system_clock::now();
		auto t = std::chrono::system_clock::to_time_t(now);
		char ts[64];
		std::strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));

		f << "{\n";
		f << "  \"timestamp\": \"" << ts << "\",\n";
		f << "  \"commit\": \"" << commit << "\",\n";
		f << "  \"results\": [\n";

		for (std::size_t i = 0; i < results.size(); ++i) {
			const auto& r = results[i];
			f << "    {\n";
			f << "      \"algorithm\": \"" << r.algorithm << "\",\n";
			f << "      \"instance\": \"" << r.instance << "\",\n";
			f << "      \"tier\": " << r.tier << ",\n";
			f << "      \"n\": " << r.n << ",\n";
			f << "      \"optimum\": " << r.optimum << ",\n";
			f << "      \"cost\": " << r.cost << ",\n";
			f << "      \"gap_pct\": " << r.gap_pct << ",\n";
			f << "      \"wall_time_ms\": " << r.wall_time_ms << ",\n";
			f << "      \"cpu_time_ms\": " << r.cpu_time_ms << ",\n";
			f << "      \"peak_rss_mb\": " << r.peak_rss_mb << ",\n";
			f << "      \"status\": \"" << r.status << "\",\n";
			f << "      \"runs\": " << r.runs << ",\n";
			if (r.runs > 1) {
				f << "      \"mean_cost\": " << r.mean_cost << ",\n";
				f << "      \"stddev_cost\": " << r.stddev_cost << ",\n";
			}
			f << "      \"valid\": " << (r.valid ? "true" : "false") << "\n";
			f << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
		}

		f << "  ]\n";
		f << "}\n";
	}

private:
	// Use matrix path for Tier 1-4, coord path for Tier 5
	static constexpr int coord_threshold_ = 5;

	BenchmarkResult run_one(
		const AlgorithmEntry& algo,
		const InstanceDef& inst,
		const std::string& path)
	{
		BenchmarkResult r;
		r.algorithm = algo.name;
		r.instance = inst.name;
		r.tier = inst.tier;

		auto opt_it = optima_.find(inst.name);
		r.optimum = (opt_it != optima_.end()) ? opt_it->second.cost : -1;

		bool use_coords = (inst.tier >= coord_threshold_) && !inst.is_explicit;

		if (use_coords)
			run_with_coords(algo, path, r);
		else
			run_with_matrix(algo, path, r);

		// Compute gap
		if (r.optimum > 0)
			r.gap_pct = 100.0 * (r.cost - r.optimum)
						/ static_cast<double>(r.optimum);
		else
			r.gap_pct = -1.0;

		return r;
	}

	void run_with_matrix(
		const AlgorithmEntry& algo,
		const std::string& path,
		BenchmarkResult& r)
	{
		auto mat = periple::load_tsplib(path);
		r.n = mat.size();
		r.runs = algo.runs;

		periple::Solver solver(mat);
		// Every benchmark instance is symmetric (see BENCHMARKING.md); declaring
		// it gives local search its O(1) evaluation path. unchecked skips the
		// O(n^2) debug verification.
		solver.set_symmetric(true, periple::unchecked);
		long rss_before = peak_rss_kb();

		if (algo.runs > 1) {
			std::vector<long long> costs(algo.runs);
			std::vector<double> times(algo.runs);

			for (int run = 0; run < algo.runs; ++run) {
				solver.set_matrix(mat);
				unsigned seed = 42 + static_cast<unsigned>(run);
				auto t0 = std::chrono::steady_clock::now();
				algo.run_matrix(solver, seed);
				auto t1 = std::chrono::steady_clock::now();
				costs[run] = static_cast<long long>(solver.cost());
				times[run] = std::chrono::duration<double, std::milli>(
					t1 - t0).count();
			}

			long rss_after = peak_rss_kb();
			fill_multi_run_stats(r, costs, times);
			r.status = status_string(solver.status());
			r.cpu_time_ms = 0;
			r.peak_rss_mb = std::max(0.0, (rss_after - rss_before) / 1024.0);

			auto vr = validate_tour(mat, solver.tour(), solver.cost());
			r.valid = vr.valid;
			r.error = vr.error;
		} else {
			auto t0_wall = std::chrono::steady_clock::now();
			auto t0_cpu = std::clock();
			algo.run_matrix(solver, 0);
			auto t1_cpu = std::clock();
			auto t1_wall = std::chrono::steady_clock::now();
			long rss_after = peak_rss_kb();

			double wall_ms = std::chrono::duration<double, std::milli>(
				t1_wall - t0_wall).count();
			double cpu_ms = 1000.0 * (t1_cpu - t0_cpu) / CLOCKS_PER_SEC;

			// Repeat for short solves to get stable timing
			if (wall_ms < 10.0) {
				int repeats = std::max(5, static_cast<int>(100.0 / wall_ms));
				std::vector<double> times(repeats);
				for (int i = 0; i < repeats; ++i) {
					solver.set_matrix(mat);
					auto start = std::chrono::steady_clock::now();
					algo.run_matrix(solver, 0);
					auto end = std::chrono::steady_clock::now();
					times[i] = std::chrono::duration<double, std::milli>(
						end - start).count();
				}
				std::sort(times.begin(), times.end());
				wall_ms = times[repeats / 2]; // median
			}

			r.cost = static_cast<long long>(solver.cost());
			r.status = status_string(solver.status());
			r.wall_time_ms = wall_ms;
			r.cpu_time_ms = cpu_ms;
			r.peak_rss_mb = std::max(0.0, (rss_after - rss_before) / 1024.0);

			auto vr = validate_tour(mat, solver.tour(), solver.cost());
			r.valid = vr.valid;
			r.error = vr.error;
		}
	}

	void run_with_coords(
		const AlgorithmEntry& algo,
		const std::string& path,
		BenchmarkResult& r)
	{
		auto cd = periple::load_tsplib_coords(path);
		r.n = cd.size();
		r.runs = algo.runs;

		periple::Solver solver(cd);
		solver.set_symmetric(true, periple::unchecked);
		long rss_before = peak_rss_kb();

		if (algo.runs > 1) {
			std::vector<long long> costs(algo.runs);
			std::vector<double> times(algo.runs);

			for (int run = 0; run < algo.runs; ++run) {
				solver.set_matrix(cd);
				unsigned seed = 42 + static_cast<unsigned>(run);
				auto t0 = std::chrono::steady_clock::now();
				algo.run_coords(solver, seed);
				auto t1 = std::chrono::steady_clock::now();
				costs[run] = static_cast<long long>(solver.cost());
				times[run] = std::chrono::duration<double, std::milli>(
					t1 - t0).count();
			}

			long rss_after = peak_rss_kb();
			fill_multi_run_stats(r, costs, times);
			r.status = status_string(solver.status());
			r.cpu_time_ms = 0;
			r.peak_rss_mb = std::max(0.0, (rss_after - rss_before) / 1024.0);
		} else {
			auto t0_wall = std::chrono::steady_clock::now();
			auto t0_cpu = std::clock();
			algo.run_coords(solver, 0);
			auto t1_cpu = std::clock();
			auto t1_wall = std::chrono::steady_clock::now();
			long rss_after = peak_rss_kb();

			r.cost = static_cast<long long>(solver.cost());
			r.status = status_string(solver.status());
			r.wall_time_ms = std::chrono::duration<double, std::milli>(
				t1_wall - t0_wall).count();
			r.cpu_time_ms = 1000.0 * (t1_cpu - t0_cpu) / CLOCKS_PER_SEC;
			r.peak_rss_mb = std::max(0.0, (rss_after - rss_before) / 1024.0);
		}

		// Validate last run
		auto tour = solver.tour();
		std::size_t n = cd.size();
		r.valid = (tour.size() == n);
		if (r.valid) {
			std::vector<typename decltype(solver)::city_type> sorted(
				tour.begin(), tour.end());
			std::sort(sorted.begin(), sorted.end());
			for (std::size_t i = 0; i < n && r.valid; ++i)
				r.valid = (static_cast<std::size_t>(sorted[i]) == i);
			if (!r.valid)
				r.error = "not a valid Hamiltonian cycle";
		} else {
			r.error = "tour size mismatch";
		}
		// Cost recomputation is O(N) even for CoordinateDistance
		if (r.valid) {
			long long recomputed = 0;
			for (std::size_t i = 0; i < n; ++i)
				recomputed += cd(tour[i], tour[(i + 1) % n]);
			if (recomputed != r.cost) {
				r.valid = false;
				r.error = "cost mismatch: reported "
					+ std::to_string(r.cost)
					+ " != recomputed " + std::to_string(recomputed);
			}
		}
	}

	std::string instances_dir_;
	std::vector<AlgorithmEntry> algorithms_;
	std::unordered_map<std::string, Optimum> optima_;
	int min_tier_ = 1;
	int max_tier_ = 5;
};

} // namespace bench