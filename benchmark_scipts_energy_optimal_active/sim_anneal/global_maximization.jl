using JuMP
using Ipopt
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

# Container to store best values
best_vals = Float64[]

# Wrapped objective function that tracks best-so-far
mutable struct Tracker
    best::Float64
end

tracker = Tracker(Inf)
global global_optimum_cost = 0.0
global cost_vector = Float64[]
num_params = 200
num_samples = 4*num_params
terminal_time = 22
epsilon = 1e-6
sip_lower_bound = vcat([0 for _ in 1:num_samples])
sip_upper_bound = vcat([terminal_time for _ in 1:num_samples])
initial_guess = vcat([(i-1) * (terminal_time) / num_samples for i in 1:num_samples])
sip_lower_bound = float(sip_lower_bound)
sip_upper_bound = float(sip_upper_bound)
global initial_guess = float(initial_guess)
global max_anneling_iterations = 30
initial_state = [400, 400, 500, -10, -10, -105]

function maximization_cost(time_sample)
    global num_params
    global num_samples
    global terminal_time
    initial_state = [400, 400, 500, -10, -10, -105]
    cost = -internal_minimization(num_samples, num_params, time_sample, terminal_time, initial_state)
    tracker.best = cost
    push!(best_vals, cost)
    return cost
end

###############################################
############### Maximization ##################
###############################################



function annealing_optimization()

    global initial_guess
    global max_anneling_iterations

    # start_time = time()
    options = Optim.Options(iterations = max_anneling_iterations, store_trace = true, show_trace = true)
    result = optimize(maximization_cost, sip_lower_bound, sip_upper_bound, initial_guess, SAMIN(nt = 5, ns = 10, rt = 0.9), options)
    
    # end_time = time()
    # println(end_time - start_time)

    # optimal_time_samples = result.minimizer
    trace = result.trace
    objective_values = [trace[i].value for i in 1:length(trace)]
    
    
    # slack_commands, control_commands = control_trajectory(num_samples, num_params, optimal_time_samples, terminal_time)
    
    # control_norm = sqrt.(control_commands[1,:].^2 + control_commands[2,:].^2 + control_commands[3,:].^2)
    
    optimal_value = maximum(-objective_values)
    global cost_vector = vcat(cost_vector, optimal_value)
    println("Local optimal value: ", optimal_value)
    global global_optimum_cost = max(global_optimum_cost, optimal_value)

    println("Current best objective value: ", global_optimum_cost)

end

b = @benchmark annealing_optimization() samples=200 evals=1 seconds=10000
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
save("annealing_histogram_comp.eps", fig)
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
save("annealing_histogram_max.eps", fig)
fig
