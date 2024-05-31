import matplotlib.pyplot as plt
import numpy as np
import math

from field import rep_field, att_field

px, py, sx, sy, mx, my = 300, 300, 50, 50, 500, 500

posRbX, posRbY = [50, 250]
dis = math.sqrt((posRbX-py)*(posRbX-py)+(posRbY-px)*(posRbY-px))
dis = (1/math.log(dis))*750

rep_field = rep_field(px, py, dis, dis, mx, my)
# att_field = att_field(px, py, mx, my)

U, V = np.gradient(rep_field)

# Normalize the U and V components
magnitude = np.sqrt(U**2 + V**2)
U = U / magnitude
V = V / magnitude

arrow_length = 0.00015
U = U * arrow_length
V = V * arrow_length

# Create a grid of points
x, y = np.meshgrid(range(mx), range(my))

# Sample the grid points
n = 15
x = x[::n, ::n]
y = y[::n, ::n]
U = U[::n, ::n]
V = V[::n, ::n]

M = np.hypot(U, V)

# Plot the vector field using quiver (Magic :))))))
fig1, ax1 = plt.subplots()
Q = ax1.quiver(x, y, U, V, M, scale = 0.01, units = 'width', linewidths=0.5)
qk = ax1.quiverkey(Q, 0.1, 0.1, 2, r'APF test', labelpos='E',
                   coordinates='figure')

# Display the plot
plt.show()

