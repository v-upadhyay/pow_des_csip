using LinearAlgebra, Serialization
using CairoMakie

function state_trajectory(acceleration, N, T)
    dT = T/N
    A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]
    B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1]
    x0 = [400, 400, 300, -10, -10, -75]
    I = Diagonal(ones(6))
    X = zeros(6, N)

    for i in 1:N-1
        g = [0, -3.711, 0]
        x = (I + A*i*dT) * x0
        t = i*dT 
        for j in 1:i
            x = x + dT*(I + A*(t - dT*(j-0.5)))*B*(acceleration[:, j] + g)
        end
        X[:, i] = x
    end
    return X
end

# Load control data
control = deserialize("Control_Data.dat")
X = state_trajectory(control[1:3, :], 100, 22)

println("Trajectory Computed")

# Create figure
fig = Figure(size = (800, 610), backgroundcolor = :white)

label_size = 30
axis_letter_size = 18
# Create a 3D axis
ax = Axis3(fig[1, 1],  
           xlabel = "Range", ylabel = "Cross", zlabel = "Altitude",
           xlabelsize=label_size , ylabelsize=label_size , zlabelsize = label_size,
           titlesize=18,
           xticklabelsize=axis_letter_size, yticklabelsize=axis_letter_size, zticklabelsize = axis_letter_size,
           xticklabelcolor = :black, yticklabelcolor = :black, zticklabelcolor = :black,
           xlabelcolor = :black, ylabelcolor = :black, zlabelcolor = :black,
           xgridvisible = true, ygridvisible = true, zgridvisible = true,
           xgridcolor = (:red, 0.3), ygridcolor = (:red, 0.3), zgridcolor = (:red, 0.3),
           xgridwidth = 1, ygridwidth = 1, zgridwidth = 1)

ax.xlabelcolor = :black
ax.ylabelcolor = :black
ax.zlabelcolor = :black

# Set axis background color
ax.backgroundcolor = :white

# Plot fuel optimal trajectory as scatter plot
traj_plot = lines!(ax, X[1, :], X[3, :], X[2, :], 
                   color = :blue, linewidth = 3, label = "Energy Optimal Trajectory")


# Define grid points for the plane x = y
x_range = LinRange(minimum(X[1, :]) - 3, maximum(X[1, :]) + 3, 30)
z_range = LinRange(minimum(X[3, :]) - 3, maximum(X[3, :]) + 3, 30)

X_plane = [x for x in x_range, z in z_range]
Y_plane = [z for x in x_range, z in z_range]
Z_plane = [x for x in x_range, z in z_range]

# Plot the plane x = y
plane_plot = surface!(ax, X_plane, Y_plane, Z_plane, color=:cyan, transparency=true, alpha=0.3)


# Add a large marker at the target (0,0,0)
target_marker = scatter!(ax, [0], [0], [0], 
                         color = :green, markersize = 15, label = "Target Point")

# Add a large marker at the initial point (400, 300, 400)
initial_marker = scatter!(ax, [X[1,1]], [X[3,1]], [X[2,1]], 
                          color = :black, markersize = 15, label = "Initial Point")

legend = Legend(fig[1, 1], ax, framevisible = true, tellwidth = false, tellheight = false,
       halign = :right, valign = :top, padding = (5, 5, 5, 5), patchsize = (20, 10), backgroundcolor = :white, labelcolor = :black,textsize=18, labelsize=19)

# Align the legend to the top
legend.halign = :left  # Center horizontally
legend.valign = :top      # Move to the top

# Display the figure
fig