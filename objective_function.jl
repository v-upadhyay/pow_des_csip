using Ipopt
using LinearAlgebra
using StaticArrays

function cost_function(model, acceleration, slack_control, dT)
    # epsilon = (1e-1)/3125
    epsilon = 0
    @objective(model, Min, sum(slack_control) * dT + epsilon * (sum(acceleration.^2) + sum(slack_control.^2)) * dT)
end