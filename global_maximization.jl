using JuMP
using Ipopt
using Plots
using LinearAlgebra
using StaticArrays
using Dates
using Serialization
using Optim
using Random
using LaTeXStrings

println("Reading the files...")

include("objective_function.jl")
include("constraints.jl")
include("internal_minimization.jl")

println("Files read successfully")

num_params = 100
num_samples = 4*num_params
terminal_time = 22
epsilon = 1e-6
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
max_anneling_iterations = 10
options = Optim.Options(iterations = max_anneling_iterations, store_trace = true, show_trace = true)
result = optimize(maximization_cost, sip_lower_bound, sip_upper_bound, initial_guess, SAMIN(nt = 5, ns = 10, rt = 0.7), options)

end_time = time()
println(end_time - start_time)

optimal_time_samples = result.minimizer
trace = result.trace
objective_values = [trace[i].value for i in 1:length(trace)]


slack_commands, control_commands = control_trajectory(num_samples, 100, optimal_time_samples, 22)

control_norm = sqrt.(control_commands[1,:].^2 + control_commands[2,:].^2 + control_commands[3,:].^2)

optimal_value = maximum(-objective_values)
println("Optimal value: ", optimal_value)

# # Plot the norm of control with increased plot size, axis label font sizes, grids, and curve thickness as a step plot
# plot(control_norm, label=L"\|\mathbf{u^{\mathcal{D}}_{\beta}}(t)\|", xlabel="Time", ylabel="Magnitudes", legend=:bottomright, legendfontsize=12, size=(800, 600), xlabelfontsize=20, ylabelfontsize=20, grid=true, gridalpha=0.8, gridcolor=:indigo, gridlinewidth=0.5, linewidth=3, seriestype=:steppost)

# # Add the slack control to the plot as a step plot
# plot!(slack_commands, label=L"s^{\mathcal{D}}_{\alpha}(t)", legendfontsize=12, linewidth=3, linestyle=:dash, seriestype=:steppost)

# # Add horizontal lines at y = 2 and y = 10
# hline!([2, 10], label="Control bounds", legendfontsize=12, linewidth=3, linestyle=:dot)
