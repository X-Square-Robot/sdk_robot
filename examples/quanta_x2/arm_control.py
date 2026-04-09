import time
from typing import Annotated
import typer
from x2robot import Robot, connect
from x2robot.sdk import RobotModeParam, RobotWorkMode
from x2robot.sdk import ManipulatorControlModeParam, ManipulatorControlMode
from x2robot.geometry_msgs import Pose, Point, Quaternion
from x2robot.sdk import JointPositions


import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from toppra.constraint import JointVelocityConstraint, JointAccelerationConstraint


def move_arm_joints_toppra(arm, target_positions: list, v_max=1.0, a_max=3):
    # 1. Define joint limits
    lower_limits = np.array([-3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708])
    upper_limits = np.array([3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708])

    # 2. Get current state
    current_state = arm.get_joint_states()
    q_start = np.array(current_state.position)
    q_end = np.array(target_positions)
    
    # --- Manual position validity check ---
    # Check target point
    if np.any(q_end < lower_limits) or np.any(q_end > upper_limits):
        print("Error: Target position out of limits!")
        return
    # Check current point (prevent sensor drift)
    q_start = np.clip(q_start, lower_limits, upper_limits)

    num_joints = len(q_start)

    # 3. Trajectory planning
    waypoints = np.stack([q_start, q_end])
    path = ta.SplineInterpolator([0, 1], waypoints)

    # 4. Build dynamics constraints (Velocity & Acceleration)
    pc_vel = JointVelocityConstraint([v_max] * num_joints)
    pc_acc = JointAccelerationConstraint([a_max] * num_joints)

    # 5. Solve
    # Note: If there is no JointPositionConstraint in the environment,
    # We mainly rely on pc_vel and pc_acc to ensure smooth motion, and the position is guaranteed by the path
    instance = algo.TOPPRA([pc_vel, pc_acc], path)
    traj = instance.compute_trajectory(0, 0)

    if traj is None:
        print("TOPP-RA planning failed.")
        return

    # 6. Execute (200Hz)
    duration = traj.duration
    dt = 0.002
    ts = np.arange(0, duration, dt)
    
    for t in ts:
        q_t = traj(t)
        # Again double insurance: clip at the command level
        q_t_safe = np.clip(q_t, lower_limits, upper_limits)
        
        joint_cmd = JointPositions()
        joint_cmd.positions = q_t_safe.tolist()
        arm.set_joint_positions(joint_cmd)
        time.sleep(dt)

    # Final forced alignment with target for precision
    final_cmd = JointPositions()
    final_cmd.positions = q_end.tolist()
    arm.set_joint_positions(final_cmd)
    print("Movement Finished.")


def move_by_joint_positions(robot: Robot, arm):
    # 1. Initialization and configuration
    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
    # Set to joint position control mode
    robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS))

    # --- Task A: Joint Reset ---
    print("Starting joint reset...")
    
    if (arm == "left"):
        zero_positions = [1.57, -1.57, -1.57, -1.57, 0, 0, 0]
        move_arm_joints_toppra(robot.left_arm, zero_positions)
    else:
        zero_positions = [-1.57, -1.57, 1.57, -1.57, 0, 0, 0]
        move_arm_joints_toppra(robot.right_arm, zero_positions)

    time.sleep(1)

    # --- Task B: Move to specific joint angles ---
    # Convert angles to radians: deg * (pi/180)
    # Example: Rotate the 4th joint by 30 degrees, the 6th joint by 45 degrees
    if (arm == "left"):
       target_q = [1.57, -1.57, -1.57, -1.07, 0, 0, 0.7854]
       print(f"Moving left arm to specific joint angles: {target_q}")
       move_arm_joints_toppra(robot.left_arm, target_q)
    else:
      target_q = [-1.57, -1.57, 1.57, -1.07, 0, 0, 0.7854]
      print(f"Moving right arm to specific joint angles: {target_q}")
      move_arm_joints_toppra(robot.right_arm, target_q)

    time.sleep(2)
    print("Demo finished.")


# move by end pose
def move_arm_endpose_toppra(arm, target_pose, other_arm=None, waist=None, v_max=2.2, a_max=0.3):
    # 1. Get current real pose as the absolute starting point
    start_pose = arm.get_end_pose()
    
    p_start = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])
    p_end = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    
    q_start = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, 
               start_pose.pose.orientation.z, start_pose.pose.orientation.w]
    q_end = [target_pose.orientation.x, target_pose.orientation.y, 
             target_pose.orientation.z, target_pose.orientation.w]

    if other_arm is not None:
        other_pose = other_arm.get_end_pose()

    if waist is not None:
        waist_pose = waist.get_end_pose()

    # 2. TOPP-RA Trajectory Planning
    dist = np.linalg.norm(p_end - p_start)
    path_len = dist if dist > 1e-6 else 1.0
    
    # Establish a 1D geometric path: moving from 0 to path_len
    path = ta.SplineInterpolator([0, 1], np.array([[0], [path_len]]))
    pc_vel = constraint.JointVelocityConstraint([v_max])
    pc_acc = constraint.JointAccelerationConstraint([a_max])
    
    instance = algo.TOPPRA([pc_vel, pc_acc], path)
    traj = instance.compute_trajectory(0, 0)
    
    if traj is None:
        print("TOPP-RA planning failed")
        return

    # 3. Prepare Orientation Interpolator (SLERP)
    # SLERP is based on the time axis [0, duration]
    key_rots = R.from_quat([q_start, q_end])
    slerp_func = Slerp([0, traj.duration], key_rots)

    # 4. Execute Movement
    duration = traj.duration
    interval = 0.005  # 200Hz
    ts = np.arange(0, duration, interval)
    
    print(f"Executing TOPP-RA smooth trajectory: Estimated duration {duration:.2f}s")

    for t in ts:
        # 1. Get the planned position s_t for the current time point (0 <= s_t <= path_len)
        s_t = traj(t)[0]
        
        # 2. Calculate nonlinear alpha ratio (this coefficient satisfies smooth accel/decel)
        # Even though the path is a straight line, the movement speed is now governed by TOPP-RA
        alpha = np.clip(s_t / path_len, 0, 1)

        pose = Pose()
        pose.position = Point()      
        pose.orientation = Quaternion() 
        
        # --- Position Interpolation: alpha now follows the velocity curve ---
        pose.position.x = p_start[0] + (p_end[0] - p_start[0]) * alpha
        pose.position.y = p_start[1] + (p_end[1] - p_start[1]) * alpha
        pose.position.z = p_start[2] + (p_end[2] - p_start[2]) * alpha

        # --- Orientation Interpolation (SLERP) ---
        curr_q = slerp_func(t).as_quat()
        pose.orientation.x = curr_q[0]
        pose.orientation.y = curr_q[1]
        pose.orientation.z = curr_q[2]
        pose.orientation.w = curr_q[3]

        # Send target command
        arm.set_end_pose(pose)

        if other_arm is not None:
            other_arm.set_end_pose(other_pose.pose)
        if waist is not None:
            waist.set_end_pose(waist_pose.pose)
        time.sleep(interval)

    # 5. Print joint angles after movement finishes
    arm.set_end_pose(target_pose) # Ensure precise alignment with the endpoint
    time.sleep(0.1)
    
    # Read and print joint angles
    cur_pose = arm.get_end_pose()
    print(f"\nMovement finished:\n")
    print(f"Current end pose: position={cur_pose.pose.position}, orientation={cur_pose.pose.orientation}")
    print(f"Target end pose: position={target_pose.position}, orientation={target_pose.orientation}")

def move_by_end_pose(robot: Robot, arm):
    # 1. Initialization and configuration
    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
    # Set to end pose control mode, need to set waist and dual arms simultaneously
    robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE))

    time.sleep(1)
    # --- Task A: End Pose Reset
    print("Starting end pose reset...")
    if (arm == "left"):
        zero_pose = Pose(position=Point(x=0.400, y=0.220, z=0.877), orientation=Quaternion(x=0.000, y=-0.001, z=0.001, w=1.000))
        move_arm_endpose_toppra(robot.left_arm, zero_pose, other_arm=robot.right_arm, waist=robot.waist)
    else:
        # !Note: Y axis is negative for right arm
        zero_pose = Pose(position=Point(x=0.400, y=-0.220, z=0.877), orientation=Quaternion(x=-0.000, y=0.000, z=0.000, w=1.000))
        move_arm_endpose_toppra(robot.right_arm, zero_pose, other_arm=robot.left_arm, waist=robot.waist)
    
    time.sleep(2)

    # --- Task B: Move to specific end pose ---
    # move up 20 cm
    print("Executing TOPP-RA trajectory control...")
    if (arm == "left"):
        target = Pose(position=Point(x=0.400, y=0.220, z=1.080), orientation=Quaternion(x=0.000, y=0.000, z=0.000, w=1.000))
        move_arm_endpose_toppra(robot.left_arm, target, other_arm=robot.right_arm, waist=robot.waist)
    else:
        target = Pose(position=Point(x=0.400, y=-0.220, z=1.080), orientation=Quaternion(x=0.000, y=0.000, z=0.000, w=1.000))
        move_arm_endpose_toppra(robot.right_arm, target, other_arm=robot.left_arm, waist=robot.waist)

    time.sleep(2)
    print("Demo finished.")

def stream_arm_joint_states(robot: Robot, arm):
    print("Starting arm joint states streaming...")
    print("Press Ctrl+C to stop streaming")
    try:
        if arm == "left":
            for joint_state in robot.left_arm.get_joint_states_stream():
                print(f"joint_state: {joint_state}")
                time.sleep(0.1)
        elif arm == "right":
            for joint_state in robot.right_arm.get_joint_states_stream():
                print(f"joint_state: {joint_state}")
                time.sleep(0.1) 
        else:
            print("Invalid arm. Valid options: left, right")
            return
    except KeyboardInterrupt:
        print("\nStopping streaming...")

def stream_arm_end_pose(robot: Robot, arm):
    print("Starting arm end pose streaming...")
    print("Press Ctrl+C to stop streaming")
    try:
        if arm == "left":
            for end_pose in robot.left_arm.get_end_pose_stream():
                print(f"end_pose: {end_pose}")
                time.sleep(0.1)
        elif arm == "right":
            for end_pose in robot.right_arm.get_end_pose_stream():
                print(f"end_pose: {end_pose}")
                time.sleep(0.1)
        else:
            print("Invalid arm. Valid options: left, right")
            return
    except KeyboardInterrupt:
        print("\nStopping streaming...")

def main(
    server: Annotated[str, typer.Option(help="server address")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="action: move, stream")] = "move",
    mode: Annotated[str, typer.Option(help="mode: joint_pos, end_pose")] = "joint_pos",
    arm: Annotated[str, typer.Option(help="left or right arm")] = "left"
):
    # connect to the robot
    robot = connect(f"x2://{server}")

    if action == "move":
        if mode == "joint_pos":
            move_by_joint_positions(robot, arm)
        elif mode == "end_pose":
            move_by_end_pose(robot, arm)
        else:
            print("Invalid mode")
            return
    elif action == "stream":
        if mode == "joint_pos":
            stream_arm_joint_states(robot, arm)
        elif mode == "end_pose":
            stream_arm_end_pose(robot, arm)
        else:
            print("Invalid mode")
            return


if __name__ == "__main__":
    typer.run(main)
