using JuMP
using Ipopt
using LinearAlgebra
using StaticArrays
using Dates
using Serialization
using Zygote
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
include("gradol_function.jl")

println("Files read successfully")

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
MAX_OUTER_EPOCHS = 20

function safe_log(x)
    return log(max(x, 1e-10))
end

function log_barrier(control_commands, time_sample, alpha)
    global num_params
    num_samples = 4*num_params
    terminal_time = 22
    initial_state = [400, 400, 500, -10, -10, -105]

    barrier = 0.0
    
    constraint = constraints_value(control_commands, num_samples, time_sample, terminal_time/num_params, num_params, initial_state)
    barrier += sum(-safe_log.(constraint))

    barrier += alpha * objective_function(norm.(control_commands), terminal_time/num_params)

    return barrier
end

function gradient(time_sample)
    grad = zeros(length(time_sample))
    alpha = 8 * 10^3
    global num_params  
    num_samples = 4*num_params
    terminal_time = 22
    initial_state = [400, 400, 500, -10, -10, -105]
    slack_commands, control_commands = control_trajectory(num_samples, num_params, time_sample, terminal_time, initial_state)
    obj_value = objective_function(slack_commands, terminal_time/num_params)
    println(" Objective value: ", obj_value)
    grad = Zygote.gradient(log_barrier, control_commands, time_sample, alpha)

    return alpha, grad[2]
    
end

###############################################
############### Maximization ##################
###############################################


function maximization()
    global num_params
    global epsilon
    global num_samples
    global terminal_time
    # terminal_eps = terminal_time - epsilon
    # t_sample = range(0, stop= terminal_eps, length=num_samples) |> collect
    t_sample = terminal_time * rand(num_samples)  # Random initial u
    t_sample_prev = copy(t_sample)  # Initialize t_sample_prev as a copy of t_sample

    for epoch in 1:MAX_OUTER_EPOCHS
        lr_t = 0.4  # Update outer learning rate here

        alpha, grad_t = gradient(t_sample)

        t_sample .= t_sample .+ lr_t .* grad_t / alpha
        t_sample .= clamp.(t_sample, 0.0, terminal_time - 1e-5)  # Ensure time samples are within bounds

        if norm(t_sample .- t_sample_prev) < 1e-3
            # println("Final time samples: ", t_sample)
            # println("Final maximization gradient: ", grad_t)
            println("Converged")

            break
        else
            t_sample_prev .= t_sample  # Update t_sample_prev for the next iteration
        end
    end
end



# maximization()
# time_start = now()
# maximization()
# time_end = now()
# println("Total Time taken: ", time_end - time_start)
b = @benchmark maximization() samples=30 evals=1
display(b)