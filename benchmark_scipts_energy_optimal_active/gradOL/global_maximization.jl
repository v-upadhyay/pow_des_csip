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

global local_optimum_cost = 0.0
global global_optimum_cost = 0.0
global cost_vector = []
global num_params        = 200
global num_samples       = 4 * num_params
global terminal_time     = 22.0
global epsilon           = 1e-6
global MAX_OUTER_EPOCHS  = 5
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
    alpha = 1e4

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

    global local_optimum_cost = max(local_optimum_cost, obj_value)

    println("Best local objective cost: ", local_optimum_cost)

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
    global local_optimum_cost = 0.0

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
    global cost_vector = vcat(cost_vector, local_optimum_cost)
    global global_optimum_cost = max(global_optimum_cost, local_optimum_cost)
    println("Best global optimum cost: ", global_optimum_cost)
end

b = @benchmark maximization() samples=200 evals=1 seconds=2400
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
save("gradOL_histogram_comp.eps", fig)
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
save("gradOL_histogram_max.png", fig)
fig

