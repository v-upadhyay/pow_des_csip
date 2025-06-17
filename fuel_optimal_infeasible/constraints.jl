using Ipopt
using LinearAlgebra
using StaticArrays

function disturbance_integral(time, A)
    disturbance = [0, 0, 0, (1 - cos(time)), 0, sin(time)] - A * [0, 0, 0, (sin(time) - time * cos(time)), 0, (cos(time) + time * sin(time))]
    return disturbance
end

function control_constraints(model, acceleration, slack_control, num_params, dT)
    
    for i in 1:num_params
        @constraint(model, 2 <= slack_control[i] <= 10)
        @constraint(model, acceleration[1, i]^2 + acceleration[2, i]^2 + acceleration[3, i]^2  <=  slack_control[i]^2)
        # @constraint(model, sum(slack_control) * dT <=  200)
    end
end

function dynamics_constraints(model, acceleration, num_samples, sample_time, dT, num_params)
    A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]
    B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1]
    x0 = [400, 400, 300, -10, -10, -75]
    I = Diagonal(ones(6))
    g = [0, -3.71, 0]

    

    for i in 2:num_samples+1

        time_step = Int(floor(sample_time[i-1] / dT))
        time_left = sample_time[i-1] - time_step*dT
        psuedo_state = x0
        for j in 1:time_step
            # Define the matrix and vector terms separately for clarity
            matrix_term = (I - A * (j - 0.5) * dT) * B
            vector_term = acceleration[:, j] + g

            # Calculate the matrix-vector product row-by-row using a comprehension
            # This avoids the ambiguity of the full `*` operator
            num_rows = size(A, 1) # This is 6 in your case
            update = [dot(matrix_term[k, :], vector_term) for k in 1:num_rows] * dT

            psuedo_state = psuedo_state + update
        end
        # Define the matrix and vector terms
        matrix_term = (I - A * (time_step * dT + time_left / 2)) * B
        vector_term = acceleration[:, time_step+1] + g

        # Perform the explicit row-by-row dot product
        num_rows = size(A, 1)
        update = [dot(matrix_term[k, :], vector_term) for k in 1:num_rows] * time_left

        psuedo_state = psuedo_state + update
        state = (I + A * sample_time[i-1]) * (psuedo_state + disturbance_integral(sample_time[i-1], A))

        @constraint(model, state[1] >= state[2])
        # @constraint(model, state[1] == state[2])
        @constraint(model, state[1] <= state[2])
        # @constraint(model, state[2] >= 0)
        # @constraint(model, state[2] >= 0.5 * sqrt(state[1]^2 + state[2]^2 + state[3]^2))
        # @constraint(model, state[2] <= 400)

    end

    time_step = num_params
    psuedo_state = x0
    for j in 1:time_step
        # Define the matrix and vector terms separately for clarity
        matrix_term = (I - A * (j - 0.5) * dT) * B
        vector_term = acceleration[:, j] + g

        # Calculate the matrix-vector product row-by-row using a comprehension
        # This avoids the ambiguity of the full `*` operator
        num_rows = size(A, 1) # This is 6 in your case
        update = [dot(matrix_term[k, :], vector_term) for k in 1:num_rows] * dT

        psuedo_state = psuedo_state + update
    end
    state = (I + A * num_params * dT) * (psuedo_state + disturbance_integral(num_params * dT, A))
    @constraint(model, state .== zeros(6))
    

end



