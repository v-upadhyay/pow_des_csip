
function internal_minimization(num_samples, num_params, sample_time, terminal_time)

# Record the start time
start_time = now()

dT = terminal_time / num_params

# Define the optimization model
model = Model(Ipopt.Optimizer)

# Set the maximum number of iterations
set_optimizer_attribute(model, "print_level", 0) 
set_optimizer_attribute(model, "max_iter", 1000)
set_optimizer_attribute(model, "tol", 1e-5)
set_optimizer_attribute(model, "acceptable_tol", 1e-5)
set_optimizer_attribute(model, "hessian_approximation", "limited-memory")

                       

########################################
######### Control Variables ############
########################################
@variable(model, acceleration[1:3, 1:num_params])                   # acceleration at each time step
@variable(model, slack_control[1:num_params])                       # slack variable for control constraints


# println("Adding the constraints...")

# Define the objective function to minimize the total distance
cost_function(model, acceleration, slack_control, dT)

# Add constraints to the model
dynamics_constraints(model, acceleration, num_samples, sample_time, dT, num_params)
control_constraints(model, acceleration, slack_control, num_params, dT)

# println("Constraints added successfully")



# Solve the optimization problem
optimize!(model)

# Record the end time
end_time = now()

# Calculate the duration
duration = end_time - start_time


# print("Optimization completed successfully in ", duration, " seconds")

status = termination_status(model)
println("Solution status: ", status)

status = termination_status(model)
println("Solution status: ", status)

if status == MOI.LOCALLY_INFEASIBLE
    serialize("Infeasible_time_samples.dat", sample_time)
end

return objective_value(model)

end





function control_trajectory(num_samples, num_params, sample_time, terminal_time)

    # Record the start time
    start_time = now()
    
    dT = terminal_time / num_params
    
    # Define the optimization model
    model = Model(Ipopt.Optimizer)
    
    # Set the maximum number of iterations
    # set_optimizer_attribute(model, "print_level", 0) 
    set_optimizer_attribute(model, "max_iter", 1000)
    set_optimizer_attribute(model, "tol", 1e-5)
    set_optimizer_attribute(model, "acceptable_tol", 1e-5)
    set_optimizer_attribute(model, "hessian_approximation", "limited-memory")
    
                           
    
    ########################################
    ######### Control Variables ############
    ########################################
    @variable(model, acceleration[1:3, 1:num_params])                   # acceleration at each time step
    @variable(model, slack_control[1:num_params])                       # slack variable for control constraints
    
    
    # println("Adding the constraints...")
    
    # Define the objective function to minimize the total distance
    cost_function(model, acceleration, slack_control, dT)
    
    # Add constraints to the model
    dynamics_constraints(model, acceleration, num_samples, sample_time, dT, num_params)
    control_constraints(model, acceleration, slack_control, num_params, dT)
    
    # println("Constraints added successfully")
    
    
    
    # Solve the optimization problem
    optimize!(model)
    
    # Record the end time
    end_time = now()
    
    # Calculate the duration
    duration = end_time - start_time
    
    
    print("Optimization completed successfully in ", duration, " seconds")

    U = zeros(4, num_params)

    U[1:3, :] = value.(acceleration)
    U[4, :] = value.(slack_control)
    serialize("Control_Data.dat", U)
    
    return value.(slack_control), value.(acceleration)
    
end