import numpy as np
import matplotlib.pyplot as plt

# Function to generate a grid of points
def generate_grid(x_range, y_range, step):
    x, y = np.meshgrid(np.arange(x_range[0], x_range[1], step), np.arange(y_range[0], y_range[1], step))
    return x, y

# Functions to define the different potential fields
def repulsive_force(x, y):
    u = -x / (x**2 + y**2)
    v = -y / (x**2 + y**2)
    return u, v

def vortex_force(x, y):
    u = y / (x**2 + y**2)
    v = -x / (x**2 + y**2)
    return u, v

def safe_apf(x, y, d_safe, d_vort):
    u = np.zeros_like(x)
    v = np.zeros_like(y)
    
    safe_zone = x**2 + y**2 < d_safe**2
    vortex_zone = (x**2 + y**2 >= d_safe**2) & (x**2 + y**2 < d_vort**2)
    
    u[safe_zone] = -x[safe_zone]
    v[safe_zone] = -y[safe_zone]
    
    u[vortex_zone] = y[vortex_zone]
    v[vortex_zone] = -x[vortex_zone]
    
    return u, v

# Set up the grid
x_range = [-2, 2]
y_range = [-2, 2]
step = 0.2
x, y = generate_grid(x_range, y_range, step)

# Calculate vector fields
u_rep, v_rep = repulsive_force(x, y)
u_vort, v_vort = vortex_force(x, y)
u_safe1, v_safe1 = safe_apf(x, y, d_safe=0.1, d_vort=0.3)
u_safe2, v_safe2 = safe_apf(x, y, d_safe=0.4, d_vort=0.5)

# Plot the vector fields
fig, axes = plt.subplots(2, 2, figsize=(10, 10))

# Repulsive force
Q = axes[0, 0].quiver(x, y, u_rep, v_rep)
qk = axes[0, 0].quiverkey(Q, 0.9, 0.9, 2, r'Test', labelpos='E',
                   coordinates='figure')
axes[0, 0].set_title('Repulsive force')
axes[0, 0].set_xlim(x_range)
axes[0, 0].set_ylim(y_range)

# Vortex force
axes[1, 0].quiver(x, y, u_vort, v_vort)
axes[1, 0].set_title('Vortex force')
axes[1, 0].set_xlim(x_range)
axes[1, 0].set_ylim(y_range)

# Safe APF d_safe=0.1, d_vort=0.3
axes[0, 1].quiver(x, y, u_safe1, v_safe1)
axes[0, 1].set_title('Safe APF d_safe=0.1, d_vort=0.3')
axes[0, 1].set_xlim(x_range)
axes[0, 1].set_ylim(y_range)

# Safe APF d_safe=0.4, d_vort=0.5
axes[1, 1].quiver(x, y, u_safe2, v_safe2)
axes[1, 1].set_title('Safe APF d_safe=0.4, d_vort=0.5')
axes[1, 1].set_xlim(x_range)
axes[1, 1].set_ylim(y_range)

# Show the plot
plt.tight_layout()
plt.show()

