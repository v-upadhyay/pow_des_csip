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
- `internal_minimization.jl` – Defines the inner convex optimization problem using NLP (IPOPT).
- `objective_function.jl` – Implements the cost function to minimize fuel/ energy usage.
- `constraints.jl` – Defines constraints on dynamics, control inputs, and terminal conditions.
- `state_trajectory.jl` – Visual demonstration of state constraints ssatisfaction.
