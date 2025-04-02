using JuMP
using Ipopt
using CairoMakie
using LinearAlgebra
using LaTeXStrings
# using Colors  
using StaticArrays
using Dates
using Serialization
using Optim
using Random

println("Reading the files...")

include("objective_function.jl")
include("constraints.jl")
include("internal_minimization.jl")

println("Files read successfully")

num_params = 100
num_samples = 4*num_params
terminal_time = 22
sip_lower_bound = vcat([0 for _ in 1:num_samples])
sip_upper_bound = vcat([terminal_time for _ in 1:num_samples])
initial_guess = vcat([(i-1) * (terminal_time) / num_samples for i in 1:num_samples])
sip_lower_bound = float(sip_lower_bound)
sip_upper_bound = float(sip_upper_bound)
initial_guess = float(initial_guess)

function maximization_cost(time_sample)
    num_params = 100
    num_samples = 4*num_params
    terminal_time = 22
    return -internal_minimization(num_samples, num_params, time_sample, terminal_time)
end

start_time = time()
max_anneling_iterations = 30
options = Optim.Options(iterations = max_anneling_iterations, store_trace = true, show_trace = true)
result = optimize(maximization_cost, sip_lower_bound, sip_upper_bound, initial_guess, SAMIN(nt = 5, ns = 10, rt = 0.7), options)

end_time = time()
println(end_time - start_time)

###############################################
########### Optimizers Collection #############
###############################################

optimal_time_samples = result.minimizer
trace = result.trace
objective_values = [trace[i].value for i in 1:length(trace)]


slack_commands, control_commands = control_trajectory(num_samples, 100, optimal_time_samples, 22)

control_norm = sqrt.(control_commands[1,:].^2 + control_commands[2,:].^2 + control_commands[3,:].^2)

optimal_value = maximum(-objective_values)
println("Optimal value: ", optimal_value)

###############################################
############### Visualization #################
###############################################

# Define time vector based on data length
N = num_params
Δt = terminal_time / (N - 1)  
t = collect(0:Δt:terminal_time)  

@assert length(t) == length(control_norm) "Mismatch in length of time vector and data"

# Define color for grid (same as previous)
grid_color = RGBAf(1.0, 0.0, 0.0, 0.3)  # Red with 30% opacity
label_size = 40  # Consistent label size

# Create Figure
fig = Figure(size=(900, 600))

ax = Axis(fig[1, 1], 
    xlabel="Time (s)", ylabel="Magnitudes",
    xlabelsize=label_size, ylabelsize=label_size, titlesize=18,
    xticklabelsize=24, yticklabelsize=24,

    # Major Grid - Semi-transparent red
    xgridvisible=true, xgridcolor=grid_color, xgridstyle=:dash,  
    ygridvisible=true, ygridcolor=grid_color, ygridstyle=:dash,

    # Minor Grid - Same Red
    xminorgridvisible=true, xminorgridcolor=grid_color,
    yminorgridvisible=true, yminorgridcolor=grid_color,

    # Minor tick density
    xminorticks=10, yminorticks=10,  

    # Major ticks
    xticks=0:2:22,  # Major ticks every 2 seconds
    yticks=0:2:maximum(control_norm)  # Major ticks every 2 units
)

# Define gradient colormap for better aesthetics
colormap = cgrad([
    RGBf(0.85, 0.95, 0.3),   # Soft Greenish Yellow
    RGBf(0.65, 0.85, 0.45),  # Muted Lime-Green
    RGBf(0.45, 0.75, 0.65),  # Soft Teal
    RGBf(0.35, 0.65, 0.85),  # Desaturated Sky Blue
    RGBf(0.3, 0.45, 0.85),   # Deep Cool Blue
    RGBf(0.25, 0.25, 0.75)   # Indigo-Navy Blend
], 6)

# Use `stairs!` for stepwise constant plots
stairs!(ax, t, control_norm, color=colormap[2], linewidth=4, label=L"\text{Control norm (Energy Optimal)}")  
stairs!(ax, t .+ 0.1Δt, slack_commands, color=colormap[5], linewidth=3, linestyle=:dashdot, label=L"\text{Slack Trajectory (Energy Optimal)}")

# Add control bounds with dotted lines
hlines!(ax, [2, 10], label=L"\text{Control bounds}", linestyle=:dot, color=:black, linewidth=3)

# Add legend with box and increased text size
legend = axislegend(ax, position=:rt, 
    framevisible=true, 
    framecolor=:black,  # Black border
    linewidth=1.5,      # Thicker border
    patchsize=(30, 15), 
    textsize=25,  
    labelsize=25
)

legend.halign = :right  # Align legend to the left
legend.valign = :bottom   # Align legend to the top

# Show Figure
display(fig)
