#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseArray, PoseStamped
import math, signal, sys, termios
from alphabot_driver.AlphaBot import AlphaBot
from alphabot_driver.PCA9685 import PCA9685
from POMDP_simple_solver import Maze, MDP, QMDPController
import numpy as np

def main():
    # ——— ROS & robot setup —————————————————————————————————————
    rospy.init_node('qmdp_controller')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    CELL_SIZE     = 0.25      # m per cell
    LINEAR_SPEED  = 0.2       # m/s
    ANGULAR_SPEED = math.pi/2 # rad/s for 90°
    CELL_TIME     = CELL_SIZE / LINEAR_SPEED
    TURN_TIME_90  = (math.pi/4) / ANGULAR_SPEED
    MOTOR_PWM     = 10        # wheel PWM (0–100%)

    Ab = AlphaBot()
    pwm = PCA9685(0x40)
    pwm.setPWMFreq(50)

    # ——— Maze + policy ————————————————————————————————————————
    grid = [
        [0,0,0,1,0],
        [1,1,0,1,0],
        [0,0,0,0,0],
        [0,1,1,1,0],
        [0,0,0,0,0]
    ]
    start, goal = (0,0), (4,0)
    checkpoints = [(0,0,0), (1,4,1), (3,4,2), (4,2,3)]

    marker_orientation_dictionary = {
        0: (1, 0.5), 1: (0.5, 0), 2: (0, 0.5), 3: (0.5, 1)
    }

    # Publish checkpoint poses
    marker_pub = rospy.Publisher(
        'global_locations/marker_pose', PoseArray, queue_size=10
    )
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "map"
    for r, c, ori in checkpoints:
        pose = PoseStamped().pose
        pose.position.x = c * CELL_SIZE + marker_orientation_dictionary[ori][0]
        pose.position.y = r * CELL_SIZE + marker_orientation_dictionary[ori][1]
        pose.position.z = ori
        pose.orientation.w = 1.0
        pose_array.poses.append(pose)

    # Wait for subscribers then publish
    while marker_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    marker_pub.publish(pose_array)

    # Strip orientation for the solver
    checkpoints = [tuple(cp[:2]) for cp in checkpoints]

    # Build solver
    maze       = Maze(grid, start, goal, checkpoints=checkpoints)
    mdp        = MDP(maze, slip_prob=0.1, step_cost=-1,
                     goal_reward=100, gamma=0.95)
    mdp.value_iteration()
    controller = QMDPController(mdp)
    controller.init_belief()                        # ← initialise belief

    # ——— shutdown handler —————————————————————————————————————
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

    # ——— state & heading —————————————————————————————————————
    true_state = maze.start_idx
    heading    = 0   # 0=east,1=north,2=west,3=south

    def send_action(a_idx):
        nonlocal true_state, heading
        action = controller.mdp.actions[a_idx]
        # compute desired heading
        desired = {'right':0,'up':1,'left':2,'down':3}[action]
        diff = (desired - heading) % 4

        # rotate
        if diff == 1:
            Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
            Ab.left();   rospy.sleep(TURN_TIME_90); Ab.stop()
        elif diff == 2:
            Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
            Ab.left();   rospy.sleep(TURN_TIME_90)
            Ab.left();   rospy.sleep(TURN_TIME_90); Ab.stop()
        elif diff == 3:
            Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
            Ab.right();  rospy.sleep(TURN_TIME_90); Ab.stop()

        heading = desired

        # move forward one cell
        Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
        Ab.forward(); rospy.sleep(CELL_TIME); Ab.stop()

        # update true_state (simulation or replace with odometry)
        true_state = np.random.choice(
            range(mdp.n),
            p=mdp.P[true_state, a_idx]
        )

        return maze.state_to_coord(true_state)

    def detect_checkpoint(coord):
        return coord in maze.checkpoints

    def check_goal(coord):
        return coord == maze.goal

    # ——— run until goal ————————————————————————————————————————
    max_steps      = 200
    true_path      = []
    believed_path  = []

    for step in range(max_steps):
        a_idx = controller.select_action()
        coord = send_action(a_idx)

        if detect_checkpoint(coord):
            controller.relocalise(maze.coord_to_state(coord))
        else:
            controller.predict_belief(a_idx)

        true_path.append(coord)
        believed_path.append(controller.get_believed_position())

        if check_goal(coord):
            rospy.loginfo("Goal reached at %s in %d steps", coord, step+1)
            break

    rospy.loginfo("True path:     %s", true_path)
    rospy.loginfo("Believed path: %s", believed_path)
    shutdown()

if __name__ == '__main__':
    main()
