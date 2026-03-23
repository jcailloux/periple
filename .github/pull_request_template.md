## Summary


## Checklist

- [ ] PR targets the `dev` branch
- [ ] Tests pass (`ctest --test-dir .build/debug --output-on-failure`)
- [ ] Follows [CONTRIBUTING.md](CONTRIBUTING.md) conventions
- [ ] Algorithm registered in `core/registry.hpp` (if new algorithm)
- [ ] Protocol tests with `LoggingCallbacks` pass (if new algorithm)
- [ ] Functional tests with at least one variant pass (if new algorithm)
- [ ] Benchmark results included below (if new/modified algorithm)

## Benchmark results
<!-- See CONTRIBUTING.md for what to report per algorithm category
     (exact, constructive, improvement, metaheuristic). -->