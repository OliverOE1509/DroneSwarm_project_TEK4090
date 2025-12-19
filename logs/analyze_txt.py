import re
import math
import numpy as np
import matplotlib.pyplot as plt
import itertools


# =========================
# Configuration
# =========================

LOGFILE = "output1.txt"
FLAG_JUMP_EPS = 0.01
DRONES = [1, 2, 3]
plt.style.use("seaborn-v0_8-muted")



# =========================
# Regex patterns
# =========================

TIME_RE = re.compile(r"\[(\d+)\.(\d+)\]")
GPS_RE = re.compile(
    r"id:\s*(\d+),\s*gps:\s*\(([^,]+),\s*([^,]+),\s*([^)]+)\)"
)
FLAG_RE = re.compile(
    r"flag:\s*\(([^,]+),\s*([^,]+),\s*([^)]+)\)"
)


# =========================
# Helpers
# =========================

def normalize_time(series, t0):
    return [t - t0 for t in series]


def segment_dist_by_flag(t, dist, respawn_times):
    """Split distance time series into flag episodes."""
    episodes = []
    start = 0

    for rt in respawn_times[1:]:
        end = start
        while end < len(t) and t[end] < rt:
            end += 1
        episodes.append(dist[start:end])
        start = end

    if start < len(dist):
        episodes.append(dist[start:])

    return episodes

def full_inter_drone_distances(agents):
    drone_ids = sorted(agents.keys())
    pairs = list(itertools.combinations(drone_ids, 2))

    T = min(len(agents[d]["t"]) for d in drone_ids)
    time = agents[drone_ids[0]]["t"][:T]

    distances = {}

    for i, j in pairs:
        xi, yi, zi = agents[i]["x"][:T], agents[i]["y"][:T], agents[i]["z"][:T]
        xj, yj, zj = agents[j]["x"][:T], agents[j]["y"][:T], agents[j]["z"][:T]

        distances[(i, j)] = [
            math.sqrt(
                (xi[k] - xj[k])**2 +
                (yi[k] - yj[k])**2 +
                (zi[k] - zj[k])**2
            )
            for k in range(T)
        ]

    return time, distances
# =========================
# Parse log file
# =========================

agents = {}
flag = {"t": [], "x": [], "y": [], "z": []}

current_time = None
current_flag = None

with open(LOGFILE) as f:
    for line in f:
        m_time = TIME_RE.search(line)
        if m_time:
            sec, nsec = map(int, m_time.groups())
            current_time = sec + nsec * 1e-9

        m_flag = FLAG_RE.search(line)
        if m_flag and current_time is not None:
            fx, fy, fz = map(float, m_flag.groups())
            current_flag = (fx, fy, fz)

            flag["t"].append(current_time)
            flag["x"].append(fx)
            flag["y"].append(fy)
            flag["z"].append(fz)

        m_gps = GPS_RE.search(line)
        if m_gps and current_time is not None and current_flag is not None:
            drone_id = int(m_gps.group(1))
            x, y, z = map(float, m_gps.groups()[1:])

            if drone_id not in agents:
                agents[drone_id] = {
                    "t": [], "x": [], "y": [], "z": [], "dist": []
                }

            fx, fy, fz = current_flag
            dist = math.sqrt((x - fx)**2 + (y - fy)**2 + (z - fz)**2)

            agents[drone_id]["t"].append(current_time)
            agents[drone_id]["x"].append(x)
            agents[drone_id]["y"].append(y)
            agents[drone_id]["z"].append(z)
            agents[drone_id]["dist"].append(dist)


# =========================
# Normalize time
# =========================

t0 = min(min(a["t"]) for a in agents.values())

for a in agents.values():
    a["t"] = normalize_time(a["t"], t0)

flag["t"] = normalize_time(flag["t"], t0)


# =========================
# Detect flag respawns
# =========================

flag_respawn_times = []
prev_pos = None

for t, x, y, z in zip(flag["t"], flag["x"], flag["y"], flag["z"]):
    pos = (x, y, z)
    if prev_pos is None:
        flag_respawn_times.append(t)
        prev_pos = pos
        continue

    dx, dy, dz = (pos[i] - prev_pos[i] for i in range(3))
    if math.sqrt(dx*dx + dy*dy + dz*dz) > FLAG_JUMP_EPS:
        flag_respawn_times.append(t)
        prev_pos = pos

flag_times = [
    flag_respawn_times[i+1] - flag_respawn_times[i]
    for i in range(len(flag_respawn_times) - 1)
]

print("Flag episode durations:", flag_times)


# =========================
# Segment distances by flag
# =========================

flag_episodes_distances = {}

for i in DRONES:
    flag_episodes_distances[f"drone{i}"] = segment_dist_by_flag(
        agents[i]["t"],
        agents[i]["dist"],
        flag_respawn_times
    )


# =========================
# Compute per-episode distance-per-second table
# =========================

n_episodes = min(
    len(flag_times),
    *(len(flag_episodes_distances[f"drone{i}"]) for i in DRONES)
)

table = np.full((n_episodes, len(DRONES)), np.nan)

for k in range(n_episodes):
    dt = flag_times[k]

    for j, drone in enumerate(DRONES):
        d = flag_episodes_distances[f"drone{drone}"][k]

        if len(d) < 2 or dt <= 0:
            continue

        distance_closed = d[0] - min(d)
        if distance_closed > 0:
            table[k, j] = distance_closed / dt


# =========================
# Density visualization
# =========================

t, pair_distances = full_inter_drone_distances(agents)

plt.figure(figsize=(9, 5), dpi=120)

for (i, j), d in pair_distances.items():
    plt.plot(t, d, linewidth=2, label=f"Drone {i}–{j}")

for rt in flag_respawn_times:
    plt.axvline(
        rt,
        color="k",
        linestyle="--",
        linewidth=1,
        alpha=0.4
    )

ymax = plt.ylim()[1]
for k in range(len(flag_respawn_times) - 1):
    mid = 0.5 * (flag_respawn_times[k] + flag_respawn_times[k+1])
    plt.text(
        mid,
        0.95 * ymax,
        f"E{k+1}",
        ha="center",
        va="top",
        fontsize=9,
        alpha=0.7
    )

plt.xlabel("Time [s]")
plt.ylabel("Inter-drone distance [m]")
plt.title("Inter-Drone Distances with Target Respawn Events")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.tight_layout()
plt.show()

# =========================
# Print table
# =========================

print("\nAverage distance per second [m/s]\n")
print("Flag\tDrone 1\t\tDrone 2\t\tDrone 3")

for k in range(n_episodes):
    row = table[k]
    print(
        f"{k+1}\t"
        f"{row[0]:.3f}\t\t"
        f"{row[1]:.3f}\t\t"
        f"{row[2]:.3f}"
    )


# =========================
# Optional plot
# =========================

plt.figure(figsize=(8, 5), dpi=120)

# Plot distance-to-flag for each drone
for drone_id, data in agents.items():
    plt.plot(
        data["t"],
        data["dist"],
        linewidth=2,
        label=f"Drone {drone_id}"
    )

# Mark episode boundaries
for rt in flag_respawn_times:
    plt.axvline(
        rt,
        color="k",
        linestyle="--",
        linewidth=1,
        alpha=0.4
    )

# Optional: label episodes
ymax = plt.ylim()[1]
for k in range(len(flag_respawn_times) - 1):
    mid = 0.5 * (flag_respawn_times[k] + flag_respawn_times[k + 1])
    plt.text(
        mid,
        0.95 * ymax,
        f"E{k+1}",
        ha="center",
        va="top",
        fontsize=9,
        alpha=0.7
    )

plt.xlabel("Time [s]")
plt.ylabel("Distance to flag [m]")
plt.title("Distance to Target with Episode Boundaries")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.tight_layout()
plt.show()
