import time
import threading
import signal
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


def move_joints_toppra(body_part, target_positions: list, v_max=1.0, a_max=3):
    # 1. Define joint limits
    lower_limits = np.array([-3.14, -3.14, -3.14, -3.14])
    upper_limits = np.array([3.14, 3.14, 3.14, 3.14])

    # 2. Get current state
    current_state = body_part.get_joint_states()
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
        body_part.set_joint_positions(joint_cmd)
        time.sleep(dt)

    # Final forced alignment with target for precision
    final_cmd = JointPositions()
    final_cmd.positions = q_end.tolist()
    body_part.set_joint_positions(final_cmd)
    print("Movement Finished.")


def stream_waist_data(robot: Robot, mode: str):
    """Demonstrate waist controller's streaming interface - real-time monitoring of joint states"""
    print("Starting waist data streaming...")
    print("Press Ctrl+C to stop streaming")

    try:
        if mode == "joint_pos":
            for joint_state in robot.waist.get_joint_states_stream():
                print(f"joint_state: {joint_state}")
                time.sleep(0.1)
        elif mode == "end_pose":
            # 1. Initialization and configuration
            robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
            # Set to end pose control mode
            robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE))
            time.sleep(3)
            for end_pose in robot.waist.get_end_pose_stream():
                print(f"end_pose: {end_pose}")
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping streaming...")


def main(
    server: Annotated[str, typer.Option(help="server address")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="action: move, stream")] = "move",
    mode: Annotated[str, typer.Option(help="mode: joint_pos, end_pose")] = "joint_pos",
):
    # connect to the robot
    robot = connect(f"x2://{server}")

    if action == "move":
        if mode == "end_pose":
            raise ValueError("end_pose mode is not supported for waist control")
            return

        # 1. Initialization and configuration
        robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
        # Set to joint position control mode
        robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS))

        # 2. Get current state
        current_state = robot.waist.get_joint_states()
        q_start = np.array(current_state.position)
        print(f"current_state: {q_start}")
        # --- Task A: Joint Reset (All axes to zero) ---
        print("Resetting waist joints to zero...")
        #
        zero_positions = [0.0, 0.0, 0.0, 0.0]
        move_joints_toppra(robot.waist, zero_positions)

        time.sleep(1)

        # --- Task B: Move to specific joint angles ---
        # Convert angles to radians: deg * (pi/180)
        # Example: Rotate the 4th joint by 30 degrees, the 6th joint by 45 degrees
        target_q = [0.0, 0.0, 0.0, 0.3]
        print(f"Moving waist to specific pose: {target_q}")
        move_joints_toppra(robot.waist, target_q)

        time.sleep(2)
        print("Demo finished.")
    elif action == "stream":
        stream_waist_data(robot, mode)
    else:
        print("Invalid action. Valid options: move, stream")
        return


if __name__ == "__main__":
    typer.run(main)
