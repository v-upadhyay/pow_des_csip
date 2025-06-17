using JuMP
using Ipopt
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
num_samples = num_params
terminal_time = 21
epsilon = 1e-6
sip_lower_bound = vcat([0 for _ in 1:num_samples])
sip_upper_bound = vcat([terminal_time for _ in 1:num_samples])
initial_guess = vcat([(i-1) * (terminal_time) / num_samples for i in 1:num_samples])
sip_lower_bound = float(sip_lower_bound)
sip_upper_bound = float(sip_upper_bound)
initial_guess = float(initial_guess)

function maximization_cost(time_sample)
    global num_samples, num_params, terminal_time
    return -internal_minimization(num_samples, num_params, sort(time_sample), terminal_time)
end

start_time = time()
max_anneling_iterations = 50
options = Optim.Options(iterations = max_anneling_iterations, store_trace = true, show_trace = true)
result = optimize(maximization_cost, sip_lower_bound, sip_upper_bound, initial_guess, SAMIN(nt = 5, ns = 10, rt = 0.5), options)

end_time = time()
println(end_time - start_time)

optimal_time_samples = sort(result.minimizer)

println("Optimal time samples: ", optimal_time_samples)