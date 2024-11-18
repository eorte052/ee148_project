#!/usr/bin/env python3
import rospy, heapq
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

# Grid dimensions
GRID_WIDTH = 7
GRID_HEIGHT = 10

# Define start and end positions
start_pos = (GRID_HEIGHT - 1, GRID_WIDTH // 2)  # Middle of the bottom row
end_pos = (0, GRID_WIDTH // 2)  # Middle of the top row
i = 0

def spawn_wall(x, y, spawn_model_prox):
    global i
    i = i + 1

    # XML for the built-in unit box model, scaled to look like a wall
    wall_xml = f"""
    <sdf version="1.6">
      <model name="wall">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size> <!-- Width, Depth, Height -->
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>1 1 1</size> <!-- Width, Depth, Height -->
              </box>
            </geometry>
            <material>
              <ambient>0.7 0.7 0.7 1</ambient>
              <diffuse>0.7 0.7 0.7 1</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """

    # Set the pose for where the wall will be placed
    wall_pose = Pose()
    wall_pose.position.x = x  # Adjust as needed
    wall_pose.position.y = y
    wall_pose.position.z = 0.5  # Half the wall height

    try:
        spawn_model_prox(f'wall_{i}', wall_xml, '', wall_pose, 'world')
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn wall: {e}")

def a_star(grid, start, end):
    rows, cols = len(grid), len(grid[0])

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, end), 0, start))

    came_from = {}
    g_score = {start: 0}

    closed_set = set()

    while open_set:
        current_f_score, current_g_score, current = heapq.heappop(open_set)

        if current == end:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        if current in closed_set:
            continue

        closed_set.add(current)

        y, x = current
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            neighbor = (ny, nx)
            # If in bounds
            if 0 <= ny < rows and 0 <= nx < cols and grid[ny][nx] == 0:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))

    # No path
    return None

def create_obstacles():
    global i
    i = 0
    is_valid = False
    while not is_valid:
        grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

        # Two obstacles in each row, allowing repeats
        for y in range(1, GRID_HEIGHT - 1):
            x = np.random.randint(0, GRID_WIDTH, size=3)
            for x_pos in x:
                grid[y][x_pos] = 1
        
        path = a_star(grid, start_pos, end_pos)
        # Make sure path is valid
        if path is not None:
            is_valid = True
        else:
            grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
    
    return grid, path

if __name__ == '__main__':
    try:
        rospy.init_node('spawn_wall_node', anonymous=True)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        grid, path = create_obstacles()
        
        i = 0
        for y in range(GRID_HEIGHT):
            for x in range (GRID_WIDTH):
            	if grid[y][x]:
                    spawn_wall(x, y, spawn_model_prox)
                    
        #spawn borders
        for y in range(-1, GRID_HEIGHT+1):
            spawn_wall(-1, y, spawn_model_prox)
            spawn_wall(GRID_WIDTH, y,spawn_model_prox)
            
        for x in range(GRID_WIDTH):
            spawn_wall(x, -1, spawn_model_prox)
            spawn_wall(x, GRID_HEIGHT,spawn_model_prox)
    except rospy.ROSInterruptException:
        pass

