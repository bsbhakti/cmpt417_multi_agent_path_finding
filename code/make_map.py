import os
from collections import defaultdict

scen_folder_path = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/code/scen-even"  # Replace with your actual folder path
output_maps_folder = "custominstances"
os.makedirs(output_maps_folder, exist_ok=True)

base_map_path = "/Users/bhakti/Desktop/bhakti/year_4_sem_1/cmpt_417/cmpt417_multi_agent_path_finding/maze-32-32-4.map"  # Replace with your actual base map file path

with open(base_map_path, "r") as f:
    base_map_lines = f.readlines()
    map_width, map_height = map(int, base_map_lines[0].strip().split())
    base_map_grid = [list(line.strip()) for line in base_map_lines[1:]]
def create_map_instance(agents, base_map_grid, output_path):
    with open(output_path, "w") as f:
        map_height = len(base_map_grid)
        map_width = len(base_map_grid[0])
        f.write(f"{map_width} {map_height}\n")
        
        for row in base_map_grid:
            f.write("".join(row) + "\n")
        
        f.write(f"{len(agents)}\n")
        
        for start, goal in agents:
            start_x, start_y = start
            goal_x, goal_y = goal
            f.write(f"{start_x} {start_y} {goal_x} {goal_y}\n")

for filename in os.listdir(scen_folder_path):
    if filename.endswith(".scen"):
        file_path = os.path.join(scen_folder_path, filename)
        
        scenario_instances = defaultdict(list)
        with open(file_path, "r") as file:
            for line in file:
                data = line.strip().split()
                if data[0] == 'version':
                    continue
                
                scenario_id = int(data[0])  
                start_x = int(data[4])
                start_y = int(data[5])
                goal_x = int(data[6])
                goal_y = int(data[7])
                
                if base_map_grid[start_x][start_y] == '@' or base_map_grid[goal_x][goal_y] == '@':
                    print(f"Discarding agent with start ({start_x}, {start_y}) or goal ({goal_x}, {goal_y}) on obstacle in {filename} scenario {scenario_id}")
                    continue
                
                scenario_instances[scenario_id].append(((start_x, start_y), (goal_x, goal_y)))

        for scenario_id, agents in scenario_instances.items():
            output_map_path = os.path.join(output_maps_folder, f"{filename.replace('.scen', f'_{scenario_id}.map')}")
            create_map_instance(agents, base_map_grid, output_map_path)
            print(f"Created map instance: {output_map_path}")
