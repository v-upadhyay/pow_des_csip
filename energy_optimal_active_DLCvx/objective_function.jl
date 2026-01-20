using Ipopt
using LinearAlgebra
using StaticArrays

function cost_function(model, acceleration, slack_control, dT)

    @objective(model, Min, sum(slack_control.^2) * dT)
    
end