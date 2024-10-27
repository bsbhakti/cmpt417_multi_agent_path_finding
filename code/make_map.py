import os
from collections import defaultdict

# Folder paths for .scen files and output map files
scen_folder_path = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/scen-even"  # Replace with your actual folder path
output_maps_folder = "output_maps"
os.makedirs(output_maps_folder, exist_ok=True)

# Path to the base map file
base_map_path = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/maze-32-32-4.map"  # Replace with your actual base map file path

# Load the base map layout
with open(base_map_path, "r") as f:
    base_map_lines = f.readlines()
    map_width, map_height = map(int, base_map_lines[0].strip().split())
    base_map_grid = [list(line.strip()) for line in base_map_lines[1:]]

# Function to create a map instance file with agent start and goal positions
def create_map_instance(agents, base_map_grid, output_path):
    # Write the output in the specified format
    with open(output_path, "w") as f:
        # Write the map dimensions
        f.write(f"{map_width} {map_height}\n")
        
        # Write the grid
        for row in base_map_grid:
            f.write("".join(row) + "\n")
        
        # Write the number of agents
        f.write(f"{len(agents)}\n")
        
        # Write each agent's start and goal positions without marking them on the map
        for agent_id, (start, goal) in enumerate(agents):
            start_x, start_y = start
            goal_x, goal_y = goal
            f.write(f"{start_x} {start_y} {goal_x} {goal_y}\n")

# Process each .scen file and create map instances
for filename in os.listdir(scen_folder_path):
    if filename.endswith(".scen"):
        file_path = os.path.join(scen_folder_path, filename)
        
        # Read agents' start and goal positions for each scenario in the .scen file
        scenario_instances = defaultdict(list)
        with open(file_path, "r") as file:
            for line in file:
                data = line.strip().split()
                if(data[0] == 'version'):
                    continue
                
                # Parse each field
                scenario_id = int(data[0])  # Use scenario_id to group agents
                start_x = int(data[4])
                start_y = int(data[5])
                goal_x = int(data[6])
                goal_y = int(data[7])
                
                # Store agent's start and goal position within the same scenario_id
                scenario_instances[scenario_id].append(((start_x, start_y), (goal_x, goal_y)))

        # Generate a map file for each scenario_id in the current .scen file
        for scenario_id, agents in scenario_instances.items():
            # Define a unique output path for each scenario instance
            output_map_path = os.path.join(output_maps_folder, f"{filename.replace('.scen', f'_{scenario_id}.map')}")
            create_map_instance(agents, base_map_grid, output_map_path)
            print(f"Created map instance: {output_map_path}")
