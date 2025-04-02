# Fuel-Optimal Trajectory Generation for Powered Descent

This repository contains a Julia-based implementation for solving the **fuel-optimal and energy-optimal trajectory generation problem** in powered descent. The approach employs a **convex semi-infinite program** to solve the optimal control problem (OCP) in continuous time (uncountably many constraints) while staying in the finitary optimization regime, using **simulated annealing** for optimizing time samples and **nonlinear programming (NLP)** for solving the inner convex optimization problem.

## Installation

Ensure you have Julia (recommended version: 1.10.0) installed along with the required dependencies:

```julia
using Pkg
Pkg.add(["JuMP", "Ipopt", "CairoMakie", "LinearAlgebra", "StaticArrays", "Dates", "Serialization", "Optim", "Random", "LaTeXStrings"])
```

## Usage

Run the main script:

```julia
julia global_maximization.jl
```

## File Structure

- `main.jl` – Entry point; sets up and runs the optimization pipeline.
- `internal_minimization.jl` – Defines the inner control optimization problem using NLP.
- `objective_function.jl` – Implements the cost function to minimize fuel usage.
- `constraints.jl` – Defines constraints on dynamics, control inputs, and terminal conditions.
- `terminal_conditions.jl` – Enforces final state constraints.

## References

This implementation is based on ideas from convex semi-infinite programming as discussed in:

- **S. Das, A. Aravind, A. Cherukuri, and D. Chatterjee**, *Near-optimal solutions of convex semi-infinite programs via targeted sampling*, Annals of Operations Research, vol. 318, no. 1, pp. 129–146, 2022.

```bibtex
@article{ref:SDAAACDC-22,
    AUTHOR = {S. Das and A. Aravind and A. Cherukuri and D. Chatterjee},
    TITLE = {Near-optimal solutions of convex semi-infinite programs via targeted sampling},
    JOURNAL = {Annals of Operations Research},
    VOLUME = {318},
    NUMBER = {1},
    YEAR = {2022},
    PAGES = {129--146},
}
```
