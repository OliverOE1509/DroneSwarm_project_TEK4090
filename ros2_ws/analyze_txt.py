import re
import math
import matplotlib.pyplot as plt

agents = {}
flag = {"t": [], "x": [], "y": [], "z": []}

current_time = None
current_flag = None

time_re = re.compile(r"\[(\d+)\.(\d+)\]")
gps_re = re.compile(
    r"id:\s*(\d+),\s*gps:\s*\(([^,]+),\s*([^,]+),\s*([^)]+)\)"
)
flag_re = re.compile(
    r"flag:\s*\(([^,]+),\s*([^,]+),\s*([^)]+)\)"
)

with open("output.txt") as f:
    for line in f:
        m_time = time_re.search(line)
        if m_time:
            sec = int(m_time.group(1))
            nsec = int(m_time.group(2))
            current_time = sec + nsec * 1e-9

        m_flag = flag_re.search(line)
        if m_flag and current_time is not None:
            fx = float(m_flag.group(1))
            fy = float(m_flag.group(2))
            fz = float(m_flag.group(3))
            current_flag = (fx, fy, fz)

            flag["t"].append(current_time)
            flag["x"].append(fx)
            flag["y"].append(fy)
            flag["z"].append(fz)

        m_gps = gps_re.search(line)
        if m_gps and current_time is not None and current_flag is not None:
            drone_id = int(m_gps.group(1))
            x = float(m_gps.group(2))
            y = float(m_gps.group(3))
            z = float(m_gps.group(4))

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


t0 = min(min(a["t"]) for a in agents.values())
for a in agents.values():
    a["t"] = [t - t0 for t in a["t"]]

flag["t"] = [t - t0 for t in flag["t"]]   

plt.style.use("seaborn-v0_8-muted")
plt.figure(figsize=(8, 5), dpi=120)

for drone_id, data in agents.items():
    plt.plot(
        data["t"],
        data["dist"],
        linewidth=2,
        label=f"Drone {drone_id}"
    )

plt.xlabel("Time [s]", fontsize=12)
plt.ylabel("Distance to flag [m]", fontsize=12)
plt.title("Distance to Flag Over Time", fontsize=14)

plt.grid(True, which="both", linestyle="--", linewidth=0.5, alpha=0.7)
plt.legend(frameon=True, fontsize=10)

plt.tight_layout()
plt.show()