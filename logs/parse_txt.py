import ast
from numpy.linalg import norm
import numpy as np
import matplotlib.pyplot as plt

def piecewise_ranges(lst):
    if not lst:
        return []

    ranges = []
    start = 0
    current = lst[0]

    for i in range(1, len(lst)):
        if lst[i] != current:
            ranges.append((current, start, i - 1))
            current = lst[i]
            start = i

    ranges.append((current, start, len(lst) - 1))
    return ranges


with open("logs/output.txt") as infile:
    txt = infile.readlines()

txt1 = [i for i in txt if f"[Mavic_2_PRO_{1}_Controller]" in i] 
txt2 = [i for i in txt if f"[Mavic_2_PRO_{2}_Controller]" in i] 
txt3 = [i for i in txt if f"[Mavic_2_PRO_{3}_Controller]" in i] 
    
txt1.sort(key=lambda x:x.split()[2].strip("[").strip("]"))
txt2.sort(key=lambda x:x.split()[2].strip("[").strip("]"))
txt3.sort(key=lambda x:x.split()[2].strip("[").strip("]"))

dt = 0.1
txt_1_dt = []
txt_2_dt = []
txt_3_dt = []

for i in range(0,len(txt1),6):
    txt_1_dt.append(txt1[i:i+6])

for i in range(0,len(txt2),6):
    txt_2_dt.append(txt2[i:i+6])

for i in range(0,len(txt3),6):
    txt_3_dt.append(txt3[i:i+6])

txt_1_dt = txt_1_dt[0:764]
txt_2_dt = txt_2_dt[0:764]
txt_3_dt = txt_3_dt[0:764]

flags = []
for i in range(len(txt_1_dt)):
    for j in range(0,6):
        if "flag" in txt_1_dt[i][j]:
            flags.append(ast.literal_eval(txt_1_dt[i][j].split(":")[-1].strip()))

episodes = piecewise_ranges(flags)
episode_time_intervals = [[episodes[0][1]*dt,episodes[0][2]*dt]]
for ep in episodes[1:]:
    episode_time_intervals.append([ep[1]*dt-0.1, ep[2]*dt])
    
#Average velocity per drone per episode
def get_avg_v(episodes, txt_id_dt):
    averages = []
    for ep in episodes:
        ep_txt = txt_id_dt[ep[1]: ep[2]]
        velocities = []
        for block in ep_txt:
            for i in block:
                if 'vel' in i:
                    velocities.append(norm(ast.literal_eval(i.split("vel")[-1].strip(":").strip())))
        if len(velocities) == 0:
            continue
        averages.append(round(sum(velocities)/len(velocities),4))
    return averages

avg1 = get_avg_v(episodes, txt_1_dt)
avg2 = get_avg_v(episodes, txt_2_dt)
avg3 = get_avg_v(episodes, txt_3_dt)


def get_dist_to_flag(episodes, txt_id_dt):
    dist_to_flag = []
    for ep in episodes:
        fg = np.asarray(ep[0])
        ep_txt = txt_id_dt[ep[1]: ep[2]]
        for block in ep_txt:
            for i in block:
                if "gps" in i:
                    gps = np.asarray(ast.literal_eval(i.split(":")[-1].strip()))
                    dist_to_flag.append(norm(gps-fg))
        
    return dist_to_flag

d2f_1 = get_dist_to_flag(episodes, txt_1_dt)
d2f_2 = get_dist_to_flag(episodes, txt_2_dt)
d2f_3 = get_dist_to_flag(episodes, txt_3_dt)

def plot_dist_to_flag(episode_time_intervals, d2f_1, d2f_2, d2f_3, dt=0.1, figsize=(10,4.5)):
    # create time vectors for each series
    t1 = np.arange(len(d2f_1)) * dt
    t2 = np.arange(len(d2f_2)) * dt
    t3 = np.arange(len(d2f_3)) * dt

    # prepare figure and axis
    fig, ax = plt.subplots(figsize=figsize)

    # plot series (order matches legend left->right in the reference image)
    ax.plot(t3, d2f_3, label='Drone 3', linewidth=1.8)  # plotted first to match the sample figure coloring/stacking
    ax.plot(t2, d2f_2, label='Drone 2', linewidth=1.8)
    ax.plot(t1, d2f_1, label='Drone 1', linewidth=1.8)

    # vertical lines for episode boundaries and episode labels
    # draw a vertical dashed line at each episode start (except time 0 if you prefer)
    for idx, (start, end) in enumerate(episode_time_intervals, start=1):
        # only draw lines inside the plotted x-limits
        ax.axvline(start, color='k', linestyle='--', linewidth=1, alpha=0.6)
        # label episodes by placing "E1", "E2", ... centered between start and end
        midpoint = 0.5 * (start + end)
        # determine vertical placement slightly below the top of the plotted data
        y_min, y_max = ax.get_ylim()
        y_pos = y_max - 0.06 * (y_max - y_min)
        ax.text(midpoint, y_pos, f'E{idx}', ha='center', va='bottom', fontsize=9, bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.0))

    # also draw a vertical line at the final end of the last episode for symmetry (optional)
    final_end = episode_time_intervals[-1][1]
    ax.axvline(final_end, color='k', linestyle='--', linewidth=1, alpha=0.6)

    # formatting
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Distance to flag [m]')
    ax.set_title('Distance to Target with Episode Boundaries')
    ax.grid(linestyle=':', linewidth=0.8, alpha=0.8)
    ax.legend(loc='upper left')
    ax.set_xlim(left=0, right=max(t1.max() if len(t1) else 0,
                                 t2.max() if len(t2) else 0,
                                 t3.max() if len(t3) else 0,
                                 final_end))

    # improve layout and return handles
    fig.tight_layout()
    return fig, ax

fig, ax = plot_dist_to_flag(episode_time_intervals, d2f_1, d2f_2, d2f_3)
fig.savefig('distance_to_flag.png', dpi=200)


def get_positions(txt_id_dt):
    pos_id = []
    for block in txt_id_dt: 
        for entry in block:
            if "gps" in entry:
                    gps = np.asarray(ast.literal_eval(entry.split(":")[-1].strip()))
                    pos_id.append(gps)
    return pos_id

pos_1 = get_positions(txt_1_dt)
pos_2 = get_positions(txt_2_dt)
pos_3 = get_positions(txt_3_dt)

def get_distances(pos_i, pos_j):
    dist_i_to_j = []
    for i,j in zip(pos_i,pos_j):
        dist_i_to_j.append(norm(i-j))
    return dist_i_to_j

dist_1_to_2 = get_distances(pos_1, pos_2)
dist_1_to_3 = get_distances(pos_1, pos_3)
dist_2_to_3 = get_distances(pos_2, pos_3)

def plot_dist_pairwise(episode_time_intervals, dist_1_to_2, dist_1_to_3, dist_2_to_3, dt=0.1, figsize=(10,4.5)):
        # time vectors
    t12 = np.arange(len(dist_1_to_2)) * dt
    t13 = np.arange(len(dist_1_to_3)) * dt
    t23 = np.arange(len(dist_2_to_3)) * dt

    # figure
    fig, ax = plt.subplots(figsize=figsize)

    # plot pairwise distances
    ax.plot(t12, dist_1_to_2, label='Drone 1-2', linewidth=1.8)
    ax.plot(t13, dist_1_to_3, label='Drone 1-3', linewidth=1.8)
    ax.plot(t23, dist_2_to_3, label='Drone 2-3', linewidth=1.8)

    # episode boundaries and labels
    for idx, (start, end) in enumerate(episode_time_intervals, start=1):
        ax.axvline(start, color='k', linestyle='--', linewidth=1, alpha=0.6)

        midpoint = 0.5 * (start + end)
        y_min, y_max = ax.get_ylim()
        y_pos = y_max - 0.06 * (y_max - y_min)

        ax.text(midpoint,
                y_pos,
                f'E{idx}',
                ha='center',
                va='bottom',
                fontsize=9)

    # final boundary
    final_end = episode_time_intervals[-1][1]
    ax.axvline(final_end, color='k', linestyle='--', linewidth=1, alpha=0.6)

    # formatting
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Inter-drone distance [m]')
    ax.set_title('Inter-Drone Distances with Target Respawn Events')
    ax.grid(linestyle=':', linewidth=0.8, alpha=0.8)
    ax.legend(loc='upper left')

    ax.set_xlim(
        left=0,
        right=max(t12.max() if len(t12) else 0,
                  t13.max() if len(t13) else 0,
                  t23.max() if len(t23) else 0,
                  final_end)
    )

    fig.tight_layout()
    return fig, ax

fig, ax = plot_dist_pairwise(episode_time_intervals, dist_1_to_2, dist_1_to_3, dist_2_to_3)
fig.savefig('distance_between_agents.png', dpi=200)