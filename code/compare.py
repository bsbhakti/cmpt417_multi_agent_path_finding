import os
import subprocess
import pandas as pd
import matplotlib.pyplot as plt

output_maps_folder = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/code/output_maps"
algorithms = ["Prioritized", "CBS", "CBS_DISJOINT"]
results = []

timeout_duration = 1200

for map_file in os.listdir(output_maps_folder):
    map_path = os.path.join(output_maps_folder, map_file)
    if map_file.endswith(".map"):
        print("Running ", map_file)
        for algo in algorithms:
            if(algo == "CBS_DISJOINT"):
                algo = "CBS"
                cmd = [
                "python", "run_experiments.py", 
                "--instance", map_path, 
                "--solver", algo,
                "--batch", "--disjoint"]
                algo = "CBS_Disjoint"
            else:
                cmd = [
                "python", "run_experiments.py", 
                "--instance", map_path, 
                "--solver", algo,
                "--batch"]
            try:
                output = subprocess.check_output(cmd, universal_newlines=True, timeout=timeout_duration)
                if "no solution" in output:
                    path_cost = -1
                    cpu_time = -1
                else:
                    sum_cost_line = next((line for line in output.splitlines() if "Sum of costs:" in line), None)
                    if sum_cost_line:
                        path_cost = float(sum_cost_line.split("Sum of costs:")[1].strip())
                    else:
                        path_cost = -1
                    
                    cpu_time_line = next((line for line in output.splitlines() if "CPU time (s):" in line), None)
                    if cpu_time_line:
                        cpu_time = float(cpu_time_line.split("CPU time (s):")[1].strip())
                    else:
                        cpu_time = -1  

            except subprocess.TimeoutExpired:
                print(f"Abandoned {map_file} for algorithm {algo} due to timeout.")
                path_cost = "Abandoned"
                cpu_time = "Abandoned"
            except Exception as e:
                path_cost = "Error"
                cpu_time = "Error"

            results.append({
                "Map": map_file,
                "Algorithm": algo,
                "Path Cost": path_cost,
                "CPU Time": cpu_time
            })

df = pd.DataFrame(results)

df.to_csv("MAPF_Benchmark_Results.csv", index=False)
print("Results saved to MAPF_Benchmark_Results.csv")

df["Path Cost"] = pd.to_numeric(df["Path Cost"], errors="coerce")
df["CPU Time"] = pd.to_numeric(df["CPU Time"], errors="coerce")

plt.figure(figsize=(12, 6))
df_bar = df.dropna(subset=["Path Cost"])
df_bar = df_bar.pivot(index="Map", columns="Algorithm", values="Path Cost")
df_bar.plot(kind="bar", width=0.8)
plt.title("Path Cost Comparison for Each Algorithm Across Maps")
plt.ylabel("Path Cost")
plt.xlabel("Map Instances")
plt.xticks(rotation=90)
plt.legend(title="Algorithm")
plt.tight_layout()
plt.savefig("Bar.png")

plt.figure(figsize=(12, 6))
for algo in algorithms:
    algo_df = df[(df["Algorithm"] == algo) & (df["CPU Time"] > 0)]  # Filter out invalid CPU times
    plt.scatter(algo_df["Map"], algo_df["CPU Time"], label=f"{algo} - CPU Time", alpha=0.7)

plt.xlabel("Map Instances")
plt.ylabel("CPU Time (s)")
plt.title("CPU Time Comparison for Each Algorithm Across Maps")
plt.xticks(rotation=90)
plt.legend()
plt.tight_layout()
plt.savefig("CPU_Time_Scatter.png")
