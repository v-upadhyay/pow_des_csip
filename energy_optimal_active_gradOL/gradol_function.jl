using LinearAlgebra
using Zygote

function constraints_value(acceleration, num_samples, sample_time, dT, num_params, initial_state)

    A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]
    B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1]
    x0 = initial_state
    I = Diagonal(ones(6))
    g = [0, -3.71, 0]

    error_x34 = Zygote.Buffer(sample_time)
    error_x43 = Zygote.Buffer(sample_time)
    
    
    for i in 2:num_samples+1
        current_time = sample_time[i-1]
        time_step = Zygote.dropgrad(Int(floor(current_time / dT)))
        
        time_left = current_time - time_step*dT
        
        psuedo_state = x0
        
        for j in 1:time_step
            psuedo_state = psuedo_state + (I - A * (j - 0.5) * dT) * B * (acceleration[:, j] + g) * dT
        end
        
        psuedo_state = psuedo_state + (I - A * (time_step * dT + time_left / 2)) * B * (acceleration[:, time_step+1] + g) * time_left
        
        state = (I + A * current_time) * (psuedo_state)
        
        # Store in buffer
        error_x34[i-1] = state[3] - state[4] + 1e-6
        error_x43[i-1] = state[4] - state[3] + 1e-6
    end

    # Terminal state calculation
    final_step = num_params
    psuedo_state_term = x0
    for j in 1:final_step
        psuedo_state_term = psuedo_state_term + (I - A * (j - 0.5) * dT) * B * (acceleration[:, j] + g) * dT
    end
    state_term = (I + A * num_params * dT) * (psuedo_state_term)

    error_terminal_1 = state_term - zeros(6) 
    error_terminal_2 = zeros(6) - state_term

    return vcat(copy(error_x34), copy(error_x43), error_terminal_1, error_terminal_2)
end

function objective_function(slack_control, dT)
    return sum(abs2, slack_control) * dT 
end