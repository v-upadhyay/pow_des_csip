# Fuel-Optimal Trajectory Generation for Powered Descent

This repository contains a Julia-based implementation for solving the **fuel-optimal trajectory generation problem** in atmospheric powered descent. The approach employs **simulated annealing** for optimizing time samples and **nonlinear programming (NLP)** for solving the inner control optimization problem.

## Features

- **Outer-loop optimization:** Uses simulated annealing (`SAMIN`) to optimize the time samples.
- **Inner-loop control optimization:** Nonlinear programming (`JuMP` + `Ipopt`) to solve the fuel-optimal control problem.
- **Dynamic constraints:** Implements vehicle dynamics under gravitational acceleration.
- **Control constraints:** Enforces acceleration and slack variable bounds.
- **Mars Descent Scenario:** Default parameters consider an entry from an altitude of 300m with an initial velocity of (-10, -10, -75) m/s.

## Installation

Ensure you have Julia installed along with the required dependencies:

```julia
using Pkg
Pkg.add(["JuMP", "Ipopt", "Plots", "LinearAlgebra", "StaticArrays", "Dates", "Serialization", "Optim", "Random", "LaTeXStrings"])
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
