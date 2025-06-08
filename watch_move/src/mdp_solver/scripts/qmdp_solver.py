#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, Polygon, Point32
import math, signal, sys, termios
from alphabot_driver.AlphaBot import AlphaBot
from alphabot_driver.PCA9685 import PCA9685
from POMDP_simple_solver import Maze, MDP, QMDPController
import numpy as np

CELL_SIZE     = rospy.get_param('~cell_size', 0.25)      # m per cell
LINEAR_SPEED  = 0.1       # m/s
ANGULAR_SPEED = math.pi/2*1.4 # rad/s for 90°
CELL_TIME     = CELL_SIZE / LINEAR_SPEED
TURN_TIME_90  = (math.pi/2) / ANGULAR_SPEED
MOTOR_PWM     = 10       # wheel PWM
CORRECTION_FACTOR = 1.00 # correction factor for motor PWM to match speed

current_orientation = 0  # 0=east,1=north,2=west,3=south

# Hardware
Ab  = AlphaBot()
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

# Maze and checkpoints
grid = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
    [1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
    [1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

start, goal = (1,1), (7,20)
checkpoints = [(1,6,0), (19,5,3), (17,13,0), (15,21,0), (7,21,1)] # Row, Column, Orientation (0: right side of the square, 1: above the square, 2: left side of the square, 3: below the square)

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
    global heading, current_orientation
    action = controller.mdp.actions[a_idx]

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

def update_grid_probabilities(grid_probabilities):
    global marker_exists, new_belief_updater
    belief_updater = length_belief.copy()
    new_belief_updater = np.zeros(len(length_belief), dtype=float)
    print(grid_probabilities)
    for idx, cell in enumerate(grid_probabilities.points):
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


def main():
    global marker_exists, new_belief_updater
    rospy.init_node('qmdp_controller')

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

    rospy.Subscriber('global_locations/grid_probabilities', Polygon, update_grid_probabilities)

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

    waypoint = pick_waypoint()
    path     = maze.shortest_path(controller.get_believed_position(), waypoint)
    actions  = maze.coords_to_actions(path)
    coord = start
    believed_position = start

    # Wait for camera
    rospy.sleep(5.0)

    while believed_position != goal:
        # log current belief and planned target
        rospy.loginfo("Believed pos = %s → waypoint %s",
                      controller.get_believed_position(), waypoint)
        rospy.loginfo("Executing action = %s", actions[0])
        
        # if at a checkpoint, relocalise & replan
        if marker_exists == True:
            idx = maze.coord_to_state(coord)
            controller.relocalise(new_belief_updater)
            a_idx = controller.mdp.actions.index(actions[0])
            coord = send_action(a_idx)
            believed_position = controller.get_believed_position()
            rospy.loginfo("Relocalised to %s with belief %s", believed_position, controller.belief)
            believed_path.append(believed_position)
            path     = maze.shortest_path(coord, waypoint)
            actions  = maze.coords_to_actions(path)
            marker_exists = False
            rospy.sleep(2.0)
            continue

        # otherwise predict belief forward
        else:
            a_idx = controller.mdp.actions.index(actions[0])
            coord = send_action(a_idx)
            controller.predict_belief(a_idx)
            believed_path.append(controller.get_believed_position())
            path     = maze.shortest_path(coord, waypoint)
            actions  = maze.coords_to_actions(path)

        
    rospy.loginfo("Arrived at goal %s", goal)
    rospy.loginfo("Final believed path: %s", believed_path)
    shutdown()

if __name__ == '__main__':
    main()
