using JuMP
# using Ipopt
using Clarabel
using LinearAlgebra
using StaticArrays
using Dates
using Serialization
using Optim
using Random
using CairoMakie
using LaTeXStrings
using BlackBoxOptim
using BenchmarkTools


println("Reading the files...")

include("objective_function.jl")
include("constraints.jl")
include("internal_minimization.jl")

println("Files read successfully")


global optimum_cost = 0.0
global cost_vector = []
global num_params = 200
global terminal_time = 22
global num_samples = 800
global initial_state = [400, 400, 500, -10, -10, -105]

function maximization()

    time_sample = rand(num_samples) * terminal_time
    initial_state = [400, 400, 500, -10, -10, -105]
    cost = internal_minimization(num_samples, num_params, time_sample, terminal_time, initial_state)
    # control_trajectory(num_samples, num_params, time_sample, terminal_time, initial_state)

    global optimum_cost = max(optimum_cost, cost)
    global cost_vector = vcat(cost_vector, cost)

    println("Current Optimum Cost: ", optimum_cost)
    println("Cost Vector Length: ", length(cost_vector))

end

###############################################
############### Maximization ##################
###############################################

b = @benchmark maximization() samples=200 evals=1 seconds=1200
display(b)

times_ns = b.times
times_s = times_ns ./ 1e9      # convert to milliseconds


fig = Figure(resolution = (800, 500))

ax = Axis(fig[1, 1],
    xlabel = "CPU time (in secs)",
    ylabel = "Frequency",
    xlabelsize = 32,
    ylabelsize = 32
)


ax.xticklabelsize = 28
ax.yticklabelsize = 28


hist!(
    ax,
    times_s;
    bins = 100,
    normalization = :none
)
save("scenario_histogram_comp.eps", fig)
fig



fig = Figure(size = (600, 400))
ax = Axis(
    fig[1, 1],
    xlabel = L"\text{Value of Objective Function}",
    ylabel = L"\text{Frequency}",
    # title = "Histogram of sample data"
)

hist!(
    ax,
    cost_vector;
    bins = 100,
    normalization = :none
)
save("scenario_histogram_max.png", fig)
fig