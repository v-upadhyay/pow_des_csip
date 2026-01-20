using Serialization
using LinearAlgebra
using CairoMakie
using Colors

U = deserialize("Control_Data.dat")

slack_trajectory = U[4, :]
acceleration = U[1:3, :]

N = length(slack_trajectory)
terminal_time = 22

Δt = terminal_time / (N - 1)  
t = collect(0:Δt:terminal_time)  

@assert length(t) == length(control_norm) "Mismatch in length of time vector and data"

# Define color for grid (same as previous)
grid_color = RGBAf(1.0, 0.0, 0.0, 0.3)  # Red with 30% opacity
label_size = 30  # Consistent label size

# light_purple = RGBA(0.6, 0.4, 0.8, 0.4)
light_purple = "#63bff0"
# dark_orange = RGBA(0.7, 0.2, 0.0, 1.0)
dark_orange = "#FFB347"

# Create Figure
fig = Figure(size=(900, 600))

ax = Axis(fig[1, 1],
    xlabel="Time (s)", ylabel="Magnitude",
    xlabelsize=label_size, ylabelsize=label_size, titlesize=18,
    xticklabelsize=24, yticklabelsize=24,

    # Major Grid
    xgridvisible=true, xgridcolor=grid_color,
    ygridvisible=true, ygridcolor=grid_color,

    # Major ticks
    xticks = 0:2:22,
    yticks = 0:0.7:maximum(control_norm)
)


# # Use `stairs!` for stepwise constant plots
# stairs!(ax, t, control_norm,
#     color=:dodgerblue,
#     linewidth=6,
#     label=L"\text{Control norm (Energy Optimal)}")

# # A thinner, dashed orange line for the secondary data
# stairs!(ax, t .+ 0.1Δt, slack_commands,
#     color=:darkorange,
#     linewidth=6,
#     linestyle=:dash,
#     label=L"\text{Slack Trajectory (Energy Optimal)}")


# --- Option 3: Red & Teal ---

# # Secondary Line — Cool academic navy
# stairs!(ax, t .+ 0.1Δt, slack_commands,
#     color = "#F28C8C",      # Dark blue with a scholarly tone
#     linewidth = 8,
#     linestyle = :dash,
#     label = L"\text{Slack Trajectory (Energy Optimal)}")

# # Primary Line — Refined deep crimson
# stairs!(ax, t, control_norm,
#     color = :purple,      # Deep academic red
#     linewidth = 2,
#     # linestyle = :dash,
#     label = L"\text{Control norm (Energy Optimal)}")

# Secondary Line — Cool academic navy
stairs!(ax, t .+ 0.1Δt, slack_commands,
    color = light_purple,      # Dark blue with a scholarly tone
    linewidth = 8,
    # linestyle = :dash,
    label = L"\text{Slack Trajectory (Energy Optimal)}")

# Primary Line — Refined deep crimson
stairs!(ax, t, control_norm,
    color = dark_orange,      # Deep academic red
    linewidth = 2,
    # linestyle = :dash,
    label = L"\text{Control norm (Energy Optimal)}")


# Add control bounds with dotted lines
hlines!(ax, [6.3, 10], label=L"\text{Control bounds}", linestyle=:dot, color=:black, linewidth=3)

# Add legend with box and increased text size
legend = axislegend(ax, position=:rt, 
    framevisible=true, 
    framecolor=:black,  # Black border
    linewidth=1.5,      # Thicker border
    patchsize=(30, 15), 
    textsize=25,  
    labelsize=25
)

legend.halign = :left  # Align legend to the left
legend.valign = :bottom   # Align legend to the top

# Save Figure
save("control_trajectory.png", fig)

# Show Figure
display(fig)
