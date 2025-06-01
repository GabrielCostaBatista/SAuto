#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, Polygon, Point32
import math, signal, sys, termios
from alphabot_driver.AlphaBot import AlphaBot
from alphabot_driver.PCA9685 import PCA9685
from POMDP_simple_solver import Maze, MDP, QMDPController
import numpy as np

CELL_SIZE     = rospy.get_param('~cell_size', 0.25)      # m per cell
LINEAR_SPEED  = 0.2       # m/s
ANGULAR_SPEED = math.pi/2 # rad/s for 90°
CELL_TIME     = CELL_SIZE / LINEAR_SPEED
TURN_TIME_90  = (math.pi/2) / ANGULAR_SPEED
MOTOR_PWM     = 0#10       # wheel PWM

# Hardware
Ab  = AlphaBot()
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

# Maze and checkpoints
grid = [
    [0,0,0,1,0],
    [1,1,0,1,0],
    [0,0,0,0,0],
    [0,1,1,1,0],
    [0,0,0,0,0]
]

start, goal = (0,0), (4,0)
checkpoints = [(0,0,1), (2,0,1), (3,4,2), (4,2,3)] # Row, Column, Orientation (0: right side of the square, 1: above the square, 2: left side of the square, 3: below the square)

marker_orientation_dictionary = {0: (1, 0.5), 1: (0.5, 0), 2: (0, 0.5), 3: (0.5, 1)} # Orientation to (x, y) offset for marker position or {0: (0.5, 0), 1: (0, -0.5), 2: (-0.5, 0), 3: (0, 0.5)}


# Strip orientation for the solver
checkpoints_for_maze = [tuple(cp[:2]) for cp in checkpoints]

# Build MDP & controller
maze       = Maze(grid, start, goal, checkpoints=checkpoints_for_maze)
mdp        = MDP(maze, slip_prob=0.1, step_cost=-1,
                    goal_reward=100, gamma=0.95)
mdp.value_iteration()
controller = QMDPController(mdp)
controller.init_belief()

# Track heading for correct rotations
heading = 0  # 0=east,1=north,2=west,3=south

THRESH = controller.entropy_thresh



def send_action(a_idx):
    global heading
    action = controller.mdp.actions[a_idx]

    # 1) rotate to desired heading
    desired = {'right':0,'up':1,'left':2,'down':3}[action]
    diff = (desired - heading) % 4
    if diff == 1:
        Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
        Ab.left(); rospy.sleep(TURN_TIME_90); Ab.stop()
    elif diff == 2:
        Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
        Ab.left(); rospy.sleep(TURN_TIME_90)
        Ab.left(); rospy.sleep(TURN_TIME_90); Ab.stop()
    elif diff == 3:
        Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
        Ab.right(); rospy.sleep(TURN_TIME_90); Ab.stop()
    heading = desired

    # 2) pause, then move forward one cell
    rospy.sleep(1.0)
    Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
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
    for idx, cell in enumerate(grid_probabilities.points):
        x = cell.x
        y = cell.y
        probability = cell.z
        new_belief_updater = np.zeros((len(grid), len(grid[0])))
        new_belief_updater[int(x)][int(y)] = probability
    
    marker_exists = True


def main():
    rospy.init_node('qmdp_controller')

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Publish checkpoint poses
    marker_pub = rospy.Publisher(
        'global_locations/marker_pose', PoseArray, queue_size=10
    )
    pose_array = PoseArray()
    pose_array.header.stamp    = rospy.Time.now()
    pose_array.header.frame_id = "map"
    ori_offsets = {0:(1,0.5),1:(0.5,0),2:(0,0.5),3:(0.5,1)}
    for r,c,ori in checkpoints:
        pose = PoseStamped().pose
        pose.position.x = c + marker_orientation_dictionary[ori][0]
        pose.position.y = r + marker_orientation_dictionary[ori][1]
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

    global marker_exists, new_belief_updater
    marker_exists = False
    believed_position = start

    while believed_position != goal:
        # log current belief and planned target
        rospy.loginfo("Believed pos = %s → waypoint %s",
                      controller.get_believed_position(), waypoint)
        rospy.loginfo("Executing action = %s", actions[0])
        update_grid_probabilities
        
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

        
    rospy.loginfo("Arrived at goal %s", coord)
    rospy.loginfo("Final believed path: %s", believed_path)
    shutdown()

if __name__ == '__main__':
    main()
