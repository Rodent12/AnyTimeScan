import numpy as np
import plotly.graph_objs as go

# Assuming you have an array of shape (8583, 3)
array_shape = (8583, 3)

k = 7
# Create a 3D scatter plot
trace = go.Scatter3d(
    x=all_points[k][0],
    y=all_points[k][1],
    z=all_points[k][2],
    mode='markers',
    marker=dict(
        size=4,
        color=np.arange(array_shape[0]),  # Color points based on index
        colorbar=dict(title='Index'),
        colorscale='Viridis'
    )
)

# Create layout
layout = go.Layout(scene=dict(aspectmode="data"))

# Create figure and add trace
fig = go.Figure(data=[trace], layout=layout)

# Show the plot
fig.show()