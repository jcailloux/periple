#include "benchmark_runner.hpp"

#include <periple/algorithms/registry.hpp>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
	std::string instances_dir = PERIPLE_INSTANCES_DIR;
	std::string optima_path   = PERIPLE_OPTIMA_JSON;
	std::string output_path;
	const char* algo_filter = nullptr;
	int min_tier = 1, max_tier = 5;
	int meta_runs = 10;

	// Minimal CLI parsing
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--tier") == 0 && i + 1 < argc) {
			// Accept "1,2" or single "3"
			std::string arg = argv[++i];
			auto comma = arg.find(',');
			if (comma != std::string::npos) {
				min_tier = std::stoi(arg.substr(0, comma));
				max_tier = std::stoi(arg.substr(comma + 1));
			} else {
				min_tier = max_tier = std::stoi(arg);
			}
		} else if (std::strcmp(argv[i], "--algo") == 0 && i + 1 < argc) {
			algo_filter = argv[++i];
		} else if (std::strcmp(argv[i], "--runs") == 0 && i + 1 < argc) {
			meta_runs = std::stoi(argv[++i]);
		} else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
			output_path = argv[++i];
		} else if (std::strcmp(argv[i], "--help") == 0) {
			std::printf(
				"Usage: periple_bench [options]\n"
				"  --tier N      Run only tier N (e.g. --tier 1,2)\n"
				"  --algo TAGS   Run only listed algorithms (e.g. --algo NN,HK)\n"
				"  --runs N      Number of runs for metaheuristics (default: 10)\n"
				"  --output FILE Write JSON results to FILE\n"
				"  --help        Show this message\n\n");
			periple::print_algorithm_tags(stdout);
			return 0;
		}
	}

	// Build algorithm list from shared registry
	std::vector<bench::AlgorithmEntry> algorithms;
	periple::for_each_algorithm(algo_filter, [&](const auto& algo) {
		int runs = algo.is_metaheuristic ? meta_runs : 1;
		algorithms.push_back(bench::make_algorithm(
			algo.name, algo.is_exact, algo.max_tier, runs,
			[=](auto& s, unsigned seed) { algo(s, seed); }
		));
	});

	if (algorithms.empty()) {
		std::fprintf(stderr, "No algorithms matched filter '%s'\n", algo_filter);
		periple::print_algorithm_tags();
		return 1;
	}

	bench::BenchmarkRunner runner(instances_dir, optima_path, algorithms);
	runner.set_tier_range(min_tier, max_tier);

	auto results = runner.run_all();

	bench::BenchmarkRunner::print_table(results);
	bench::BenchmarkRunner::print_comparison(results);

	if (!output_path.empty()) {
		std::string commit;
#ifdef PERIPLE_GIT_COMMIT
		commit = PERIPLE_GIT_COMMIT;
#endif
		bench::BenchmarkRunner::write_json(results, output_path, commit);
		std::printf("\nJSON written to %s\n", output_path.c_str());
	}
}