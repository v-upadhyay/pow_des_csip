using Ipopt
using LinearAlgebra
using StaticArrays

function terminal_conditions(model, spatial_coord, spatial_vel)

    @constraint(model, spatial_coord[1, end] == 0)
    @constraint(model, spatial_coord[2, end] == 0)
    @constraint(model, spatial_coord[3, end] == 0)
    @constraint(model, spatial_vel[1, end] == 0)
    @constraint(model, spatial_vel[2, end] == 0)
    @constraint(model, spatial_vel[3, end] == 0)
    
end