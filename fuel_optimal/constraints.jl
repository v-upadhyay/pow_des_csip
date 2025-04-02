using Ipopt
using LinearAlgebra
using StaticArrays

function control_constraints(model, acceleration, slack_control, num_params, dT)
    
    for i in 1:num_params
        @constraint(model, 2 <= slack_control[i] <= 10)
        @constraint(model, acceleration[1, i]^2 + acceleration[2, i]^2 + acceleration[3, i]^2  <=  slack_control[i]^2)
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
            psuedo_state = psuedo_state + (I - A * (j - 0.5) * dT) * B * (acceleration[:, j] + g) * dT
        end
        psuedo_state = psuedo_state + (I - A * (time_step * dT + time_left / 2)) * B * (acceleration[:, time_step+1] + g) * time_left
        state = (I + A * sample_time[i-1]) * (psuedo_state)

        @constraint(model, state[1] >= state[2])
        @constraint(model, state[1] <= state[2])

    end

    time_step = num_params
    psuedo_state = x0
    for j in 1:time_step
        psuedo_state = psuedo_state + (I - A * (j - 0.5) * dT) * B * (acceleration[:, j] + g) * dT
    end
    state = (I + A * num_params * dT) * (psuedo_state)
    @constraint(model, state .== zeros(6))
    

end



