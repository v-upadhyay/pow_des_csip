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

global num_params
global num_samples
global terminal_time
global initial_state
global epsilon
global MAX_OUTER_EPOCHS
global optimum_value = 0.0

# N = [50, 100, 150, 200, 250, 300, 350, 400]
# N = [50, 100]
N = [100]
time = [0.0 for _ in 1:length(N)]


for i in 1:length(N)
    println(" Number of parameters: ", i)

    global num_params        = N[i]
    global num_samples       = 4 * num_params
    global terminal_time     = 22.0
    global epsilon           = 1e-6
    global MAX_OUTER_EPOCHS  = 10
    global initial_state     = [400.0, 400.0, 500.0, -10.0, -10.0, -105.0]

    safe_log(x) = log(max(x, 1e-10))


    function log_barrier(control_commands, time_sample, alpha)
        constraint = constraints_value(
            control_commands,
            num_samples,
            time_sample,
            terminal_time / num_params,
            num_params,
            initial_state
        )
    
        barrier = sum(-safe_log.(constraint))
        barrier += alpha * objective_function(
            norm.(control_commands),
            terminal_time / num_params
        )
    
        return barrier
    end
    
    function gradient(time_sample)
        alpha = 8e3
    
        slack_commands, control_commands =
            control_trajectory(
                num_samples,
                num_params,
                time_sample,
                terminal_time,
                initial_state
            )

        obj_value = objective_function(
                slack_commands, 
                terminal_time / num_params
            )

        global optimum_value = max(optimum_value, obj_value)

        println(" Best Objective value: ", optimum_value)
    
        grad = Zygote.gradient(
            (cc, ts) -> log_barrier(cc, ts, alpha),
            control_commands,
            time_sample
        )
    
        return alpha, grad[2]
    end
    
    
    ###############################################
    ############### Maximization ##################
    ###############################################
    
    
    function maximization()
        t_sample = terminal_time .* rand(num_samples)
        t_prev = copy(t_sample)
    
        for epoch in 1:MAX_OUTER_EPOCHS
            lr_t = 0.4
    
            alpha, grad_t = gradient(t_sample)
    
            t_sample .+= lr_t .* grad_t ./ alpha
            t_sample .= clamp.(t_sample, 0.0, terminal_time - epsilon)
    
            if norm(t_sample .- t_prev) < 1e-2
                println("Converged")
                break
            end
    
            t_prev .= t_sample
        end
    end
    

    # time_start = now()
    # maximization()
    # time_end = now()
    # println("Total Time taken: ", time_end - time_start)
    # time[i] = (time_end - time_start) / Millisecond(1000)

    b = @benchmark maximization() samples=10 evals=1 seconds=60
    display(b)

end