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

num_params = 200
num_samples = 4*num_params
terminal_time = 22
epsilon = 1e-6
sip_lower_bound = vcat([0 for _ in 1:num_samples])
sip_upper_bound = vcat([terminal_time for _ in 1:num_samples])
initial_guess = vcat([(i-1) * (terminal_time) / num_samples for i in 1:num_samples])
sip_lower_bound = float(sip_lower_bound)
sip_upper_bound = float(sip_upper_bound)
initial_guess = float(initial_guess)
initial_state = [400, 400, 500, -10, -10, -105]

function maximization_cost(time_sample)
    num_params = 100
    num_samples = 4*num_params
    terminal_time = 22
    initial_state = [400, 400, 500, -10, -10, -105]
    cost = -internal_minimization(num_samples, num_params, time_sample, terminal_time, initial_state)
    tracker.best = cost
    push!(best_vals, cost)
    return cost
end

###############################################
############### Maximization ##################
###############################################


# Define bounds
lower_bounds = 0.0
upper_bounds = terminal_time

# Run DE optimization
result = bboptimize(maximization_cost;
    SearchRange = (lower_bounds, upper_bounds),
    NumDimensions = num_samples,
    Method = :de_rand_1_bin,  # DE variant
    MaxSteps = 100
)


best_sol = best_candidate(result)
best_val = best_fitness(result)

println("Best solution: ", best_sol)
println("Objective value: ", best_val)


###############################################
########### Optimizers Collection #############
###############################################

optimal_time_samples = best_sol

slack_commands, control_commands = control_trajectory(num_samples, num_params, optimal_time_samples, terminal_time, initial_state)

control_norm = sqrt.(control_commands[1,:].^2 + control_commands[2,:].^2 + control_commands[3,:].^2)


###############################################
############### Visualization #################
###############################################

# f = Figure(resolution = (800, 600))
# ax = Axis(f[1, 1], xlabel = "Iteration", ylabel = "Best Objective Value", title = "Objective vs Iteration")

# lines!(ax, 1:length(best_vals), -best_vals, color = :blue, linewidth = 2)
# f

# Define time vector based on data length
N = num_params
Δt = terminal_time / (N - 1)  
t = collect(0:Δt:terminal_time)  

@assert length(t) == length(control_norm) "Mismatch in length of time vector and data"

# Define color for grid (same as previous)
grid_color = RGBAf(1.0, 0.0, 0.0, 0.3)  # Red with 30% opacity
label_size = 30  # Consistent label size

# Create Figure
fig = Figure(size=(900, 600))

ax = Axis(fig[1, 1],
    xlabel="Time (s)", ylabel="Magnitude",
    xlabelsize=label_size, ylabelsize=label_size, titlesize=18,
    xticklabelsize=24, yticklabelsize=24,

    # Major Grid
    xgridvisible=true, xgridcolor=grid_color, xgridstyle=:dash,
    ygridvisible=true, ygridcolor=grid_color, ygridstyle=:dash,

    # Minor Grid
    # xminorgridvisible=true, xminorgridcolor=grid_color,
    # yminorgridvisible=true, yminorgridcolor=grid_color,

    # Correct way to define minor tick density
    # This creates 10 intervals (9 minor ticks) between major ticks
    # xminorticks = IntervalsBetween(10),
    # yminorticks = IntervalsBetween(10),

    # Major ticks
    xticks = 0:2:22,
    yticks = 0:0.7:maximum(control_norm)
)
# Use `stairs!` for stepwise constant plots
stairs!(ax, t, control_norm,
    color=:dodgerblue,
    linewidth=4,
    label=L"\text{Control norm (Energy Optimal)}")

# A thinner, dashed orange line for the secondary data
stairs!(ax, t .+ 0.1Δt, slack_commands,
    color=:darkorange,
    linewidth=6,
    linestyle=:dash,
    label=L"\text{Slack Trajectory (Energy Optimal)}")
# Add control bounds with dotted lines
hlines!(ax, [6.3, 10], label=L"\text{Control bounds}", linestyle=:dot, color=:black, linewidth=3)

# Add legend with box and increased text size
legend = axislegend(ax, position=:rt, 
    framevisible=true, 
    framecolor=:black,  # Black border
    linewidth=1.5,      # Thicker border
    patchsize=(30, 15), 
    textsize=25,  
    labelsize=25
)

legend.halign = :left  # Align legend to the left
legend.valign = :bottom   # Align legend to the top

# Save Figure
# save("control_trajectory.png", fig)

# Show Figure
display(fig)
