import numpy as np
import matplotlib.pyplot as plt

# -------------------------------------------------
# Artificial Potential Field parameters
# -------------------------------------------------
goal = np.array([8.0, 8.0])
obstacle = np.array([4.0, 4.0])

k_att = 1.0
k_rep = 20.0          # stronger repulsion
Q_star = 2.5          # larger influence radius
step_size = 0.05
max_iters = 800

eps = 1e-6            # avoid division by zero

# -------------------------------------------------
# Force definitions
# -------------------------------------------------
def attractive_force(x):
    return -k_att * (x - goal)

def repulsive_force(x):
    d = np.linalg.norm(x - obstacle)
    if d <= Q_star:
        return (
            k_rep
            * (1.0 / d - 1.0 / Q_star)
            * (1.0 / d**2)
            * (x - obstacle)
        )
    return np.zeros(2)

def total_force(x):
    return attractive_force(x) + repulsive_force(x)

def normalize(v):
    n = np.linalg.norm(v)
    if n < eps:
        return np.zeros_like(v)
    return v / n

# -------------------------------------------------
# Simulate agent motion (normalized direction)
# -------------------------------------------------
x = np.array([0.5, 0.5])
trajectory = [x.copy()]

for _ in range(max_iters):
    f = normalize(total_force(x))
    x = x + step_size * f
    trajectory.append(x.copy())
    if np.linalg.norm(x - goal) < 0.1:
        break

trajectory = np.array(trajectory)

# -------------------------------------------------
# Vector field grid (normalized vectors)
# -------------------------------------------------
X, Y = np.meshgrid(np.linspace(0, 10, 30),
                   np.linspace(0, 10, 30))
U = np.zeros_like(X)
V = np.zeros_like(Y)

for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        p = np.array([X[i, j], Y[i, j]])
        f = normalize(total_force(p))
        U[i, j] = f[0]
        V[i, j] = f[1]

# -------------------------------------------------
# Plot
# -------------------------------------------------
plt.figure(figsize=(8, 8))
plt.quiver(X, Y, U, V, alpha=0.6)

plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='Trajectory')
plt.scatter(goal[0], goal[1], c='g', s=120, marker='*', label='Goal')
plt.scatter(obstacle[0], obstacle[1], c='r', s=120, label='Obstacle')

circle = plt.Circle(obstacle, Q_star, color='r', fill=False, linestyle='--')
plt.gca().add_patch(circle)

plt.xlim(0, 10)
plt.ylim(0, 10)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Artificial Potential Field (Normalized Vectors, Strong Repulsion)')
plt.legend()
plt.grid(True)
plt.show()
