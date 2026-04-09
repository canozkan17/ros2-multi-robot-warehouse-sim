import yaml
import json

path = "/home/canozkan/thesis_ws/src/mini_scripts/"

file_name = "shelf_waypoints.yaml" 
with open(f'{path}/{file_name}') as f:
    data = yaml.safe_load(f)

waypoints = []
for shelf in data['shelves']: 
    waypoints.append({
        'shelf_id': shelf['id'],
        'pose': {
            'x': shelf['position']['x'],
            'y': shelf['position']['y'],
            'z': shelf['position']['z'],
            'roll': shelf['orientation']['roll'],
            'pitch': shelf['orientation']['pitch'],
            'yaw': shelf['orientation']['yaw']
        }
    })

# waypoint distirubiton per robot/drone (3 robot, load-balanced)
robot_waypoints = {f'robot{i+1}': [] for i in range(3)}
for i, wp in enumerate(waypoints):
    robot_id = f'robot{(i % 3) + 1}'
    robot_waypoints[robot_id].append(wp)

file_name = "robot_assignments.json"
with open(f'{path}/{file_name}','w') as f:
    json.dump(robot_waypoints, f, indent=2)

print("Waypoint distibution complete:")
for robot, wps in robot_waypoints.items():
    print(f"  {robot}: {len(wps)} raf")