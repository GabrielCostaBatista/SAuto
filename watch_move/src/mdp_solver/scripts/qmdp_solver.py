#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math, signal, sys, termios
from alphabot_driver.AlphaBot import AlphaBot
from alphabot_driver.PCA9685 import PCA9685
from POMDP_simple_solver import Maze, MDP, QMDPController
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped

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
    checkpoints = [(0,2), (1,4), (3,4), (4,2)]

    # Send message to topic indicating the markers position
    marker_pub = rospy.Publisher('global_locations/marker_pose', PoseArray, queue_size=10)
    rospy.loginfo("Publishing marker positions to topic 'global_locations/marker_pose'")

    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "map"
    for checkpoint in checkpoints:
        pose = PoseStamped().pose  # Only the Pose part
        pose.position.x = checkpoint[1] * CELL_SIZE
        pose.position.y = checkpoint[0] * CELL_SIZE
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        pose_array.poses.append(pose)

    rospy.loginfo(f"Publishing {len(checkpoints)} checkpoints as a PoseArray. Waiting for subscribers...")

    # Wait for subscribers to connect before publishing
    while marker_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    marker_pub.publish(pose_array)
    rospy.loginfo(f"Published {len(checkpoints)} checkpoints as a PoseArray")

    # Create the maze and MDP
    maze       = Maze(grid, start, goal, checkpoints=checkpoints)
    mdp        = MDP(maze, slip_prob=0.1, step_cost=-1, goal_reward=100, gamma=0.95)
    mdp.value_iteration()
    controller = QMDPController(mdp)

    # ——— shutdown handler —————————————————————————————————————
    settings = termios.tcgetattr(sys.stdin)
    def shutdown(signum=None, frame=None):
        rospy.loginfo('Shutting down')
        Ab.stop()
        cmd_pub.publish(Twist())     # stop cmd_vel
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.signal_shutdown('exit')
        sys.exit(0)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ——— keep track of true state & paths ————————————————————————
    true_state = maze.start_idx

    def send_action(a_idx):
        nonlocal true_state
        action = controller.mdp.actions[a_idx]
        rospy.loginfo('Action → %s', action)
        rospy.loginfo('True state → %s', true_state)
        rospy.loginfo('Believed state → %s', controller.belief)

        # rotate + move one cell
        if action == 'up':
            Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
            Ab.forward(); rospy.sleep(CELL_TIME); Ab.stop()
        elif action == 'down':
            Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
            Ab.backward(); rospy.sleep(CELL_TIME); Ab.stop()
        elif action == 'left':
            Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
            Ab.left(); rospy.sleep(TURN_TIME_90)
            Ab.forward(); rospy.sleep(CELL_TIME); Ab.stop()
        elif action == 'right':
            Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
            Ab.right(); rospy.sleep(TURN_TIME_90)
            Ab.forward(); rospy.sleep(CELL_TIME); Ab.stop()

        # update true_state via MDP transition model (or via odometry in real robot)
        true_state = np.random.choice(
            range(mdp.n),
            p=mdp.P[true_state, a_idx]
        )
        return true_state

    def detect_checkpoint(s_idx):
        return s_idx in maze.checkpoint_idxs

    def check_goal(s_idx):
        return s_idx == maze.goal_idx

    # ——— run until goal ————————————————————————————————————————
    true_path, believed_path = controller.control_loop(
        send_action,
        check_goal,
        detect_checkpoint,
        max_steps=200
    )

    # ——— log and exit ————————————————————————————————————————
    rospy.loginfo('True path:     %s', true_path)
    rospy.loginfo('Believed path: %s', believed_path)
    shutdown()

if __name__ == '__main__':
    main()