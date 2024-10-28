import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

results_file = "MAPF_Benchmark_Results.csv"
df = pd.read_csv(results_file)

df = df[df["Path Cost"] != "Abandoned"]
df["Path Cost"] = pd.to_numeric(df["Path Cost"], errors="coerce")

df["Instance"] = np.arange(1, len(df) + 1)

plt.figure(figsize=(15, 8))

jitter_strength = 0.1
markers = {'Prioritized': 'o', 'CBS': 's', 'CBS_Disjoint': 'D'}  # Different markers for each algorithm
for algo in df["Algorithm"].unique():
    subset = df[df["Algorithm"] == algo]
    jittered_x = subset["Instance"] + np.random.uniform(-jitter_strength, jitter_strength, size=len(subset))
    plt.scatter(jittered_x, subset["Path Cost"], label=algo, alpha=0.7, marker=markers[algo])

plt.xlabel("Instance Number", fontsize=14)
plt.ylabel("Path Cost", fontsize=14)
plt.title("Path Cost for Each Instance by Algorithm", fontsize=16)
plt.xticks(fontsize=10)
plt.yticks(fontsize=10)
plt.legend(title="Algorithm", fontsize=12)
plt.grid(True)
plt.tight_layout()

plt.savefig("Path_Cost_Per_Instance_Detailed.png")
plt.show()



import plotly.express as px

output_maps_folder = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/code/output_maps"
df = pd.read_csv(results_file)

def get_num_agents(map_file_path):
    with open(map_file_path, 'r') as file:
        lines = file.readlines()
        num_agents = int(lines[33].strip())  # Line 34 in Python index 33
    return num_agents

df["Number of Agents"] = df["Map"].apply(lambda map_file: get_num_agents(os.path.join(output_maps_folder, map_file)))

df = df[df["CPU Time"] != "Abandoned"]
df["CPU Time"] = pd.to_numeric(df["CPU Time"], errors="coerce")

avg_runtime = df.groupby(["Algorithm", "Number of Agents"])["CPU Time"].mean().reset_index()

avg_runtime["CPU Time"] = avg_runtime["CPU Time"].apply(lambda x: x if x > 0 else 0.001)

fig = px.line(
    avg_runtime,
    x="Number of Agents",
    y="CPU Time",
    color="Algorithm",
    log_y=True,  
    title="Average CPU Time by Number of Agents",
    labels={"CPU Time": "Average CPU Time (s)", "Number of Agents": "Number of Agents"},
    hover_data={"Algorithm": True, "Number of Agents": True, "CPU Time": ':.6f'},  # Precise hover values
)

fig.update_traces(marker=dict(size=6), line=dict(width=2))
fig.update_layout(template="simple_white")

fig.show()