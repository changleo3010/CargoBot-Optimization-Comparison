# CargoRoute: Robot Path Planning and Optimization for Warehouse Logistics

Route planning for a single warehouse robot that collects all cargo and returns to a depot, with a focus on **when exact optimization is worth its runtime versus a fast heuristic**. The project pairs a heuristic solver against an exact mixed-integer solver on two problems — an unweighted single tour (TSP) and a weight-capacitated version (CVRP) — over the same obstacle-aware distance matrix.

ECE 594T course project. Authors: Zerui Wu, Chun Yu Chang (UC Santa Barbara, Dept. of ECE).

## Problem

A robot serves 39 fixed cargo points in a 23×28 warehouse from a depot at the origin. Shelves block straight-line travel, so pairwise distances must route around obstacles rather than through them. Two routing problems are studied:

- **TSP** — ignore weight, collect all cargo in one shortest closed tour.
- **CVRP** — add a weight capacity `Q`, which forces the work to split into the minimum number of capacity-feasible tours.

Both are combinatorial problems that become costly to solve exactly as the number of pickups grows, which is exactly the question the project probes: where does an exact solver still pay off, and where does a heuristic become the sensible choice?

## Method

### Obstacle-aware distance matrix (visibility graph + Dijkstra)

A straight segment between two pickups can cut through a shelf, so Euclidean distance is unusable. Each shelf is inflated by a safety margin into an obstacle rectangle, every cargo point is remapped to its nearest free aisle cell, and a visibility graph is built over the depot, pickups, and obstacle corners — keeping an edge only when it clears all shelf interiors. All-pairs Dijkstra produces the obstacle-aware distance matrix `d_ij^VG`, the single geometric input shared by both models.

### Solvers compared

| Tool | Type | Role |
|---|---|---|
| **NetworkX + 2-opt** | Heuristic | Fast initial TSP tour, refined by 2-opt local search (no optimality guarantee) |
| **Gurobi** | Exact MILP | Provable optimality via branch-and-bound, with lazy subtour-elimination constraints; used for both TSP and CVRP |
| **OR-Tools** (Google) | Heuristic | Metaheuristic routing for CVRP, the fast/scalable counterpart to Gurobi |

Lazy constraints matter here: subtour-elimination constraints are exponentially many, so they are left out of the initial model and added only when a candidate solution actually forms a subtour.

## Results

### TSP — exact optimization wins

The heuristic and exact solvers are nearly indistinguishable, and both finish in under a second, so the provably optimal Gurobi result is the clear choice.

| Solver | Distance | Time | Optimal? |
|---|---|---|---|
| NetworkX + 2-opt | 156.8 | < 1 s | No |
| Gurobi | 154.79 | < 1 s | Yes |

The heuristic lands within ~1.3% of optimal, but since the exact solve is just as fast, there is no reason not to take the certified optimum.

### CVRP — the heuristic wins in practice

Capacity flips the conclusion. OR-Tools reaches a good tour in seconds and stalls there; Gurobi takes over twelve hours to edge only ~1.2% below it, and for the first nine hours is actually worse than what OR-Tools produces in one second. Neither closes the gap to the best bound (292.50), leaving an optimality gap above 31% — so neither result is provably optimal.

| Solver | Incumbent distance | Best bound | Time | MIP gap |
|---|---|---|---|---|
| OR-Tools | 441.56 | 292.50 | 1 s | 33.8% |
| OR-Tools | 431.26 | 292.50 | 5 s | 32.2% |
| OR-Tools | 430.15 | 292.50 | 20 s | 32.0% |
| OR-Tools | 430.15 | 292.50 | 300 s | 32.0% |
| Gurobi | 444.63 | 292.50 | 8.73 hr | 34.2% |
| Gurobi | 438.91 | 292.50 | 10.99 hr | 33.4% |
| Gurobi | 437.87 | 292.50 | 11.31 hr | 33.2% |
| Gurobi | 425.07 | 292.50 | 12.59 hr | 31.2% |

At this scale the CVRP overwhelms the exact solver, so OR-Tools is the more sensible choice.

## Takeaways

The two cases point in opposite directions. For the single-tour TSP, exact optimization is cheap and a heuristic offers no real advantage, so Gurobi is ideal. For the capacitated CVRP, Gurobi cannot prove optimality even after twelve hours while OR-Tools reaches a close result in seconds, so a heuristic is the practical choice.

Future work: find efficient ways to reach the global minimum on larger-scale CVRP instances, or methods that produce a tighter lower bound (the loose 292.50 bound is what leaves the gap unclosed here).

## References

NetworkX (`traveling_salesman_problem`, Hagberg, Schult & Swart); Gurobi Optimizer; Google OR-Tools.

---

> Note: adjust the repository structure, setup, and run commands below to match the actual code layout before publishing.

## Repository structure

```
.
├── visibility_graph/   # Shelf inflation, visibility graph, all-pairs Dijkstra
├── tsp/                # NetworkX + 2-opt and Gurobi TSP solves
├── cvrp/               # OR-Tools and Gurobi capacitated routing
├── figures/            # Warehouse layout and solution-path plots
└── README.md
```

## Running

```bash
# example placeholders — replace with the project's actual entry points
pip install -r requirements.txt   # networkx, gurobipy, ortools, numpy, matplotlib
python tsp/run.py
python cvrp/run.py
```

A valid Gurobi license is required for the exact TSP/CVRP solves; NetworkX and OR-Tools are open source.
