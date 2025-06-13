#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, Polygon, Point32, PolygonStamped
import math, signal, sys, termios
from alphabot_driver.AlphaBot import AlphaBot
from alphabot_driver.PCA9685 import PCA9685
from POMDP_simple_solver import Maze, MDP, QMDPController
import numpy as np

CELL_SIZE     = rospy.get_param('~cell_size', 0.30)      # m per cell
LINEAR_SPEED  = 0.1       # m/s
ANGULAR_SPEED = math.pi/2*1.4 # rad/s for 90°
CELL_TIME     = CELL_SIZE / LINEAR_SPEED
TURN_TIME_90  = (math.pi/2) / ANGULAR_SPEED * 0.9
#MOTOR_PWM     = 0       # wheel PWM
MOTOR_PWM     = 9.5       # wheel PWM
CORRECTION_FACTOR = 1.12 # correction factor for motor PWM to match speed

NUM_PROTECTED_MARKERS = 2

current_orientation = 0  # 0=east,1=north,2=west,3=south
current_marker = 0  # 0=right side of the square, 1=above the square, 2=left side of the square, 3=below the square
current_z = 0.0  # z coordinate of the current marker

# Hardware
Ab  = AlphaBot()
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

global wait_variable
wait_variable = True 

# Maze and checkpoints
"""
grid = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

start, goal = (1,1), (7,20)
checkpoints = [(1,6,0), (19,5,3), (17,13,0), (15,21,0), (7,21,1)] # Row, Column, Orientation (0: right side of the square, 1: above the square, 2: left side of the square, 3: below the square)
"""

grid = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
 [1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1],
 [1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1],
 [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
 [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
 [1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1],
 [1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1],
 [1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1],
 [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]
start, goal = (1,1), (1,10)
checkpoints = [(1,4,0), (3,3,2), (5,3,3), (4,6,0), (4,10,0), (1,10,1)] # Row, Column, Orientation (0: right side of the square, 1: above the square, 2: left side of the square, 3: below the square)

marker_orientation_dictionary = {0: (0.5, 1), 1: (0, 0.5), 2: (0.5, 0), 3: (1, 0.5)} # Orientation to (x/row, y/column) offset for marker position or {0: (0.5, 0), 1: (0, -0.5), 2: (-0.5, 0), 3: (0, 0.5)}


# Strip orientation for the solver
checkpoints_for_maze = [tuple(cp[:2]) for cp in checkpoints]

global length_belief

# Build MDP & controller
maze       = Maze(grid, start, goal, checkpoints=checkpoints_for_maze)
length_belief = {}
for i in range (len(grid)):
    for j in range (len(grid[0])):
        if grid[i][j] == 0:
            length_belief[(i,j)] = 0.0

mdp        = MDP(maze, slip_prob=0.1, step_cost=-1,
                    goal_reward=100, gamma=0.95)
mdp.value_iteration()
controller = QMDPController(mdp)
controller.init_belief()

# Track heading for correct rotations
heading = 0  # 0=east,1=north,2=west,3=south

THRESH = controller.entropy_thresh

marker_exists = False
new_belief_updater = None


def send_action(a_idx):
    global heading, current_orientation, wait_variable
    action = controller.mdp.actions[a_idx]
    wait_variable = True
    # 1) rotate to desired heading
    desired = {'right':0,'up':1,'left':2,'down':3}[action]
    diff = (desired - heading) % 4

    if diff == 1:
        current_orientation = (current_orientation + 1) % 4
        Ab.setPWMA(MOTOR_PWM*CORRECTION_FACTOR); Ab.setPWMB(MOTOR_PWM)
        Ab.left(); rospy.sleep(TURN_TIME_90); Ab.stop()
    elif diff == 2:
        current_orientation = (current_orientation + 2) % 4
        Ab.setPWMA(MOTOR_PWM*CORRECTION_FACTOR); Ab.setPWMB(MOTOR_PWM)
        Ab.left(); rospy.sleep(TURN_TIME_90)
        Ab.left(); rospy.sleep(TURN_TIME_90); Ab.stop()
    elif diff == 3:
        current_orientation = (current_orientation - 1) % 4
        Ab.setPWMA(MOTOR_PWM*CORRECTION_FACTOR); Ab.setPWMB(MOTOR_PWM)
        Ab.right(); rospy.sleep(TURN_TIME_90); Ab.stop()
    heading = desired

    # 2) pause, then move forward one cell
    rospy.sleep(1.0)
    Ab.setPWMA(MOTOR_PWM*CORRECTION_FACTOR); Ab.setPWMB(MOTOR_PWM)
    Ab.forward(); rospy.sleep(CELL_TIME); Ab.stop()

    # 3) pause before next decision
    rospy.loginfo("Pausing for 2 s")
    rospy.sleep(2.0)

    wait_variable = False
    print(f"[INFO] Wait variable set to {wait_variable}")

    # return the *actual* coordinate (for checkpoint/goal checks)
    # here we assume perfect odometry: map heading+movement to grid:
    #   convert believed_position + action → new coord
    bp = controller.get_believed_position()
    dr, dc = {'up':(-1,0),'down':(1,0),
                'left':(0,-1),'right':(0,1)}[action]
    if grid[bp[0]+dr][bp[1]+dc] == 1:
        # if we hit a wall, stay in place
        rospy.logwarn("Bumped into wall at %s, staying in place", bp)
        return bp
        
    return (bp[0]+dr, bp[1]+dc)

def detect_checkpoint(coord):
    return coord in maze.checkpoints

def check_goal(coord):
    return coord == maze.goal

def pick_waypoint():
    if controller.belief.max() < THRESH:
        mp = controller.get_believed_position()
        cps_sorted = sorted(
            maze.checkpoints,
            key=lambda x: abs(x[0]-mp[0]) + abs(x[1]-mp[1])
        )
        return cps_sorted[0]
    return maze.goal

def belief_to_grid(belief):
    belief_grid = np.zeros((len(grid), len(grid[0])), dtype=float)
    lcounter = 0
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:
                belief_grid[i][j] = belief[lcounter]
                lcounter += 1
            else:
                belief_grid[i][j] = -1.0  # Mark walls with -1
                
    return belief_grid

def update_grid_probabilities(grid_probabilities):
    print("[INFO] Wait:", wait_variable)

    if not wait_variable:
        global marker_exists, new_belief_updater, current_marker, current_z
        current_marker = int(grid_probabilities.header.frame_id)
        print(f"[INFO] Received grid probabilities for marker {current_marker}")
        # Extract z position from timestamp (stored as nanoseconds)
        current_z = grid_probabilities.header.stamp.nsecs / 1e9
        belief_updater = length_belief.copy()
        new_belief_updater = np.zeros(len(length_belief), dtype=float)
        for idx, cell in enumerate(grid_probabilities.polygon.points):
            row = cell.x
            column = cell.y
            probability = cell.z
            if (int(row), int(column)) in belief_updater:
                belief_updater[(int(row), int(column))] = probability
        counter= 0
        for coordinate, value in belief_updater.items():
            new_belief_updater[counter] = value
            counter += 1

        if np.sum(new_belief_updater) == 0.0:
            rospy.logwarn("No valid belief updater found, using uniform distribution")
            new_belief_updater = np.ones(len(length_belief), dtype=float)
        new_belief_updater /= np.sum(new_belief_updater)
        
        marker_exists = True

# 
# def angle_correction(believed_position):
#     global current_orientation, current_marker, current_z
# 
#     marker_x = checkpoints[current_marker - NUM_PROTECTED_MARKERS][0]
#     marker_y = checkpoints[current_marker - NUM_PROTECTED_MARKERS][1]
#     marker_ori = checkpoints[current_marker - NUM_PROTECTED_MARKERS][2]
# 
#     distance = math.sqrt((believed_position[0] - marker_x) ** 2 + (believed_position[1] - marker_y) ** 2)
#     x_global = believed_position[0] - marker_x - 0.5
# 
#     print(f"Current_z: {current_z}, Distance to marker: {distance}, x_global: {x_global}, Current orientation: {current_orientation}, Marker orientation: {marker_ori}")
# 
#     theta_1 = math.acos(current_z / distance) if distance != 0 else 0
#     theta_2 = math.acos(x_global / distance) if distance != 0 else 0
# 
#     theta = theta_2 - theta_1
# 
#     # Convert theta to degrees
#     theta = math.degrees(theta)
#     theta = ((current_orientation - marker_ori) % 4) * 90 + theta
# 
#     if theta > 180:
#         theta -= 360
#     elif theta < -180:
#         theta += 360
# 
#     print(f"Theta correction: {theta} degrees, current orientation: {current_orientation}, marker orientation: {marker_ori}. Theta1: {math.degrees(theta_1)}, Theta2: {math.degrees(theta_2)}")
# 
#     if abs(theta) > 5:
#         rospy.logwarn("Angle correction needed: %f degrees", theta)
#         if theta > 0:
#             # Rotate right
#             Ab.setPWMA(MOTOR_PWM*CORRECTION_FACTOR); Ab.setPWMB(MOTOR_PWM)
#             Ab.right(); rospy.sleep(TURN_TIME_90 * (theta / 90)); Ab.stop()
#         else:
#             # Rotate left
#             Ab.setPWMA(MOTOR_PWM*CORRECTION_FACTOR); Ab.setPWMB(MOTOR_PWM)
#             Ab.left(); rospy.sleep(TURN_TIME_90 * (-theta / 90)); Ab.stop()
# 

def main():
    global marker_exists, new_belief_updater
    rospy.init_node('qmdp_controller')
    
    # Open files for writing
    belief_file = open("belief_positions.txt", "w")
    most_likely_file = open("most_likely_positions.txt", "w")
    actions_file = open("actions_taken.txt", "w")
    entropy_file = open("entropy_values.txt", "w")
    
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Publish checkpoint poses
    marker_pub = rospy.Publisher(
        'global_locations/marker_pose', PoseArray, queue_size=10
    )
    pose_array = PoseArray()
    pose_array.header.stamp    = rospy.Time.now()
    pose_array.header.frame_id = "map"

    for x, y, ori in checkpoints:
        pose = PoseStamped().pose
        pose.position.x = x + marker_orientation_dictionary[ori][0]
        pose.position.y = y + marker_orientation_dictionary[ori][1]
        pose.position.z = ori
        pose.orientation.w = 1.0
        pose_array.poses.append(pose)
    
    # wait for subscribers
    while marker_pub.get_num_connections()==0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    marker_pub.publish(pose_array)

    rospy.Subscriber('global_locations/grid_probabilities', PolygonStamped, update_grid_probabilities)

    # ——— replannable path loop ————————————————————————————————————
    # Shutdown handler
    settings = termios.tcgetattr(sys.stdin)

    def shutdown(signum=None, frame=None):
        rospy.loginfo('Shutting down')
        Ab.stop()
        cmd_pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.signal_shutdown('exit')
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    believed_path = []

    waypoint = goal
    path     = maze.shortest_path(controller.get_believed_position(), waypoint)
    actions  = maze.coords_to_actions(path)
    coord = start
    believed_position = start
    entropy_bot = controller.belief_entropy()

    # Wait for camera
    rospy.sleep(5.0)

    while believed_position != goal:
        # Debug: Check if believed position is valid
        current_believed_pos = controller.get_believed_position()
        if grid[current_believed_pos[0]][current_believed_pos[1]] == 1:
            rospy.logerr("ERROR: Believed position %s is in a WALL! This should never happen!", current_believed_pos)
            rospy.loginfo("Current belief distribution: %s", controller.belief)
            # Force belief to start position to recover
            controller.init_belief()
            current_believed_pos = controller.get_believed_position()
            rospy.loginfo("Reset belief to start position: %s", current_believed_pos)
        
        # Plan path from current believed position
        waypoint = goal
        path = maze.shortest_path(current_believed_pos, waypoint)
        actions = maze.coords_to_actions(path)
        
        # Debug: Check if path planning failed
        if not path:
            rospy.logerr("ERROR: No path found from %s to %s!", current_believed_pos, waypoint)
            rospy.loginfo("Maze start: %s, goal: %s", maze.start, maze.goal)
            break
        if not actions:
            rospy.logerr("ERROR: No actions generated from path %s!", path)
            break
            
        # log current belief and planned target
        belief_list = controller.belief
        belief_grid = belief_to_grid(belief_list)
        belief_file.write(f"{belief_grid}\n\n")
        most_likely_file.write(f"{controller.get_believed_position()}\n")
        actions_file.write(f"{actions[0]}\n")
        entropy_file.write(f"{entropy_bot}\n")
        rospy.loginfo("Believed pos = %s → waypoint %s",
                      current_believed_pos, waypoint)
        rospy.loginfo("Executing action = %s", actions[0])

        print("Marker exists:", marker_exists)
        # if at a checkpoint, relocalise & replan
        if marker_exists == True:
            idx = maze.coord_to_state(coord)
            rospy.loginfo("Previous belief: \n%s\n", belief_grid)
            rospy.loginfo("Marker belief: %s",belief_to_grid(new_belief_updater))
            believed_position = controller.get_believed_position()
            controller.relocalise(new_belief_updater)
            rospy.loginfo("Relocalised to %s with belief %s", believed_position, controller.belief)
            believed_path.append(believed_position)
            rospy.loginfo("[INFOOOOO] Correcting angle based on marker position")
            # angle_correction(believed_position)
            a_idx = controller.mdp.actions.index(actions[0])
            coord = send_action(a_idx)
            path     = maze.shortest_path(coord, waypoint)
            actions  = maze.coords_to_actions(path)
            entropy_bot = controller.belief_entropy()

            marker_exists = False
            rospy.sleep(1.0)

        # otherwise predict belief forward
        else:
            a_idx = controller.mdp.actions.index(actions[0])
            coord = send_action(a_idx)
            controller.predict_belief(a_idx)
            
            # Update coord to match the updated belief position
            coord = controller.get_believed_position()
            believed_path.append(coord)
            
            waypoint = pick_waypoint()
            path     = maze.shortest_path(coord, waypoint)
            actions  = maze.coords_to_actions(path)
            entropy_bot = controller.belief_entropy()
            rospy.sleep(1.0)
            rospy.sleep(1.0)

        
    rospy.loginfo("Arrived at goal %s", goal)
    rospy.loginfo("Final believed path: %s", believed_path)
    belief_file.close()
    most_likely_file.close()
    actions_file.close()
    entropy_file.close()
    shutdown()

if __name__ == '__main__':
    main()
