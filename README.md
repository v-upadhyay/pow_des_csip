# **Fuel-Optimal Trajectory Generation for Powered Descent**

This repository contains a Julia implementation for solving the **fuel-optimal trajectory generation problem** in atmospheric powered descent. The approach leverages **simulated annealing** to optimize the time-sampled trajectory while using **nonlinear programming (NLP)** to solve the inner control optimization problem.

## **Features**
- **Outer-loop optimization:** Simulated annealing (`SAMIN`) for optimizing the time samples.
- **Inner-loop control optimization:** Nonlinear programming (`JuMP` + `Ipopt`) to solve the fuel-optimal control problem.
- **Dynamic constraints:** Implements vehicle dynamics under gravitational acceleration.
- **Control constraints:** Enforces acceleration and slack variable bounds.
- **Mars Descent Scenario:** Default parameters consider an entry from an altitude of 300m with an initial velocity of (-10, -10, -75) m/s.

## **Installation**
Ensure you have Julia installed along with the required dependencies:
```julia
using Pkg
Pkg.add(["JuMP", "Ipopt", "Plots", "LinearAlgebra", "StaticArrays", "Dates", "Serialization", "Optim", "Random", "LaTeXStrings"])
