---
name: New algorithm
about: Propose or submit a new TSP algorithm
labels: algorithm
---

**Algorithm name**


**Category**
<!-- Construction / Local search / Exact / Metaheuristic -->

**Reference**
<!-- Paper, textbook, or description of the approach -->

**Expected complexity**
<!-- Time and space -->

**Move types**
<!-- Which move types does it pass to variant callbacks? AppendMove, DPMove, or a new one (see "Move types" in CALLBACKS.md) -->

**Variant callbacks**
<!-- Every algorithm runs each move through move_prepare and move_filter (see "Variant and strategy support" in CONTRIBUTING.md). Which variants, if any, can it genuinely not honor, and why? -->

**Strategy callbacks**
<!-- Which framework is it a strategy of (greedy_construct for a constructive heuristic)? None for an exact algorithm. -->

**Are you willing to implement it?**
<!-- Yes / No / Need guidance -->
<!-- If yes, see CONTRIBUTING.md for the step-by-step guide. -->