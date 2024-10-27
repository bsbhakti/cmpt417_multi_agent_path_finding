import os
import subprocess
import pandas as pd
import matplotlib.pyplot as plt

# Folder containing map instances
output_maps_folder = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/code/output_maps"
# List of algorithms to benchmark
algorithms = ["Prioritized", "CBS", "CBS --disjoint"]
# Store results in a list
results = []

# Iterate over all map files in the output_maps folder
for map_file in os.listdir(output_maps_folder):
    map_path = os.path.join(output_maps_folder, map_file)
    if map_file.endswith(".map"):
        for algo in algorithms:
            # Run the experiment for each algorithm
            cmd = [
                "python", "run_experiments.py", 
                "--instance", map_path, 
                "--solver", algo
            ]
            try:
                # Capture the output
                output = subprocess.check_output(cmd, universal_newlines=True)
                if "no solution" in output:
                    nodes_expanded = "N/A"
                    path_cost = "N/A"
                else:
                    # Parse nodes expanded and path cost from output
                    nodes_expanded = int(output.split("Nodes expanded:")[1].split()[0])
                    path_cost = float(output.split("Path cost:")[1].split()[0])
            except Exception as e:
                nodes_expanded = "Error"
                path_cost = "Error"

            # Append results
            results.append({
                "Map": map_file,
                "Algorithm": algo,
                "Nodes Expanded": nodes_expanded,
                "Path Cost": path_cost
            })

# Convert results to a DataFrame for easier analysis
df = pd.DataFrame(results)

# Display the table to the user
import ace_tools as tools; tools.display_dataframe_to_user(name="MAPF Benchmark Results", dataframe=df)

# Plotting
plt.figure(figsize=(10, 6))
for algo in algorithms:
    # Filter results for each algorithm
    algo_df = df[(df["Algorithm"] == algo) & (df["Nodes Expanded"] != "N/A")]
    
    # Plot nodes expanded
    plt.plot(algo_df["Map"], algo_df["Nodes Expanded"], marker="o", label=f"{algo} - Nodes Expanded")
    
plt.xlabel("Map Instances")
plt.ylabel("Nodes Expanded")
plt.title("MAPF Benchmark: Nodes Expanded Comparison")
plt.xticks(rotation=90)
plt.legend()
plt.tight_layout()
plt.show()

# Plotting path cost for each algorithm
plt.figure(figsize=(10, 6))
for algo in algorithms:
    algo_df = df[(df["Algorithm"] == algo) & (df["Path Cost"] != "N/A")]
    
    # Plot path cost
    plt.plot(algo_df["Map"], algo_df["Path Cost"], marker="x", label=f"{algo} - Path Cost")

plt.xlabel("Map Instances")
plt.ylabel("Path Cost")
plt.title("MAPF Benchmark: Path Cost Comparison")
plt.xticks(rotation=90)
plt.legend()
plt.tight_layout()
plt.show()
