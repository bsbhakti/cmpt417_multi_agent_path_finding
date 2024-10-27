import os
from collections import defaultdict

# Folder path containing .scen files
folder_path = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/scen-even"  # Replace with your actual folder path

# Dictionary to store scenarios for each file separately
all_scenarios = {}

# Iterate over each .scen file in the folder
for filename in os.listdir(folder_path):
    if filename.endswith(".scen"):
        file_path = os.path.join(folder_path, filename)
        
        # Initialize a dictionary to store grouped scenarios for the current file
        scenario_instances = defaultdict(list)
        
        # Open the current file and process each line
        with open(file_path, "r") as file:
            for line in file:
                data = line.strip().split()
                
                # Parse each field
                scenario_id = int(data[0])  # Use scenario_id as the key
                map_name = data[1]
                map_width = int(data[2])
                map_height = int(data[3])
                start_x = int(data[4])
                start_y = int(data[5])
                goal_x = int(data[6])
                goal_y = int(data[7])
                optimal_distance = float(data[8])
                
                # Store information for each agent within the same scenario_id
                agent_info = {
                    "start": (start_x, start_y),
                    "goal": (goal_x, goal_y),
                    "optimal_distance": optimal_distance
                }
                
                # Group by scenario_id within this file
                scenario_instances[scenario_id].append(agent_info)
        
        # Store the grouped scenarios for the current file
        all_scenarios[filename] = scenario_instances

# Display the scenarios for each file
for filename, scenarios in all_scenarios.items():
    print(f"File: {filename}")
    for scenario_id, agents in scenarios.items():
        print(f"  Scenario ID: {scenario_id}")
        print(f"  Agents: {agents}")
    print()
