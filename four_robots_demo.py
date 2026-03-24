#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Four Robots Demonstration

This script demonstrates four different robots operating simultaneously
in the same simulation environment. Each robot performs different movements
to showcase their capabilities.

Author: BestMan Development Team
Date: 2026-03-24
"""

import os
import sys
import math
import time
import threading

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from Config import load_config
from Env import Client
from Robotics_API import Pose
from Visualization import Visualizer

# Import four different robot types
from Robotics_API import (
    Bestman_sim_ur5e_vacuum_long,
    Bestman_sim_panda_with_gripper,
    Bestman_sim_xarm_with_gripper,
    Bestman_sim_realman_ag95
)


def create_scene_with_four_robots(scene_path="Asset/Scene/Scene/Kitchen.json"):
    """
    Create simulation scene with four robots at different positions.
    
    Returns:
        tuple: (client, visualizer, robots_dict)
    """
    # Load configuration (assumes running from Examples directory)
    config_path = "Config/grasp_bowl_in_kitchen.yaml"
    cfg = load_config(config_path)
    
    # Initialize client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Load scene
    client.create_scene(scene_path)
    
    # Define initial positions for four robots
    robot_configs = [
        {
            'name': 'robot_ur5e',
            'class': Bestman_sim_ur5e_vacuum_long,
            'base_position': [0.5, 1.0, 0.0],
            'base_orientation': [0.0, 0.0, 0.0],
            'color': [1.0, 0.0, 0.0]  # Red
        },
        {
            'name': 'robot_panda',
            'class': Bestman_sim_panda_with_gripper,
            'base_position': [3.0, 1.0, 0.0],
            'base_orientation': [0.0, 0.0, math.pi/2],
            'color': [0.0, 1.0, 0.0]  # Green
        },
        {
            'name': 'robot_xarm',
            'class': Bestman_sim_xarm_with_gripper,
            'base_position': [0.5, 3.5, 0.0],
            'base_orientation': [0.0, 0.0, 0.0],
            'color': [0.0, 0.0, 1.0]  # Blue
        },
        {
            'name': 'robot_realman',
            'class': Bestman_sim_realman_ag95,
            'base_position': [3.0, 3.5, 0.0],
            'base_orientation': [0.0, 0.0, -math.pi/2],
            'color': [1.0, 1.0, 0.0]  # Yellow
        }
    ]
    
    robots = {}
    
    print("\n[Initializing Four Robots]")
    print("-" * 60)
    
    for i, robot_cfg in enumerate(robot_configs, 1):
        print(f"\n[{i}/4] Initializing {robot_cfg['name']}...")
        
        # Modify config for each robot's initial position
        robot_custom_cfg = load_config(config_path)
        robot_custom_cfg.Robot.base_init_pose = (
            robot_cfg['base_position'] + list(robot_cfg['base_orientation'])
        )
        
        try:
            # Create robot instance
            robot = robot_cfg['class'](client, visualizer, robot_custom_cfg)
            robots[robot_cfg['name']] = {
                'instance': robot,
                'config': robot_cfg,
                'initial_pose': Pose(
                    robot_cfg['base_position'],
                    list(robot_cfg['base_orientation'])
                )
            }
            print(f"    ✓ {robot_cfg['name']} initialized successfully")
            print(f"      Position: {robot_cfg['base_position']}")
            
        except Exception as e:
            print(f"    ✗ Failed to initialize {robot_cfg['name']}: {e}")
    
    print("\n" + "-" * 60)
    print(f"Successfully initialized {len(robots)} robots")
    
    return client, visualizer, robots


def robot_movement_pattern_1(robot_info, client, visualizer):
    """
    Movement pattern 1: Square navigation pattern
    """
    robot = robot_info['instance']
    name = robot_info['config']['name']
    initial_pos = robot_info['initial_pose'].get_position()
    
    print(f"[{name}] Starting square navigation pattern...")
    
    # Define square waypoints
    side_length = 0.8
    waypoints = [
        [initial_pos[0] + side_length, initial_pos[1], 0.0],
        [initial_pos[0] + side_length, initial_pos[1] + side_length, math.pi/2],
        [initial_pos[0], initial_pos[1] + side_length, math.pi],
        [initial_pos[0], initial_pos[1], -math.pi/2]
    ]
    
    for i, waypoint in enumerate(waypoints):
        target_pose = Pose(waypoint[:2] + [0.0], [0.0, 0.0, waypoint[2]])
        
        # Simple direct movement (without path planning for demo)
        current_pose = robot.sim_get_current_base_pose()
        
        # Move towards target
        robot.sim_navigate_base(target_pose, [[waypoint[0], waypoint[1]]])
        client.run(20)
        
        print(f"[{name}] Reached waypoint {i+1}/4")
    
    print(f"[{name}] Square pattern completed")


def robot_movement_pattern_2(robot_info, client, visualizer):
    """
    Movement pattern 2: Arm manipulation demo
    """
    robot = robot_info['instance']
    name = robot_info['config']['name']
    
    print(f"[{name}] Starting arm manipulation pattern...")
    
    # Get current arm pose
    current_joints = robot.sim_get_current_joint_values()
    print(f"[{name}] Current joints: {current_joints}")
    
    # Define some joint configurations for demonstration
    joint_configs = [
        [0.0, -math.pi/4, math.pi/2, -math.pi/4, 0.0, 0.0],
        [math.pi/4, -math.pi/6, math.pi/3, -math.pi/3, math.pi/4, 0.0],
        [-math.pi/4, -math.pi/6, math.pi/3, -math.pi/3, -math.pi/4, 0.0],
        [0.0, -math.pi/2, math.pi/2, -math.pi/2, 0.0, 0.0]  # Home position
    ]
    
    for i, config in enumerate(joint_configs):
        try:
            # Move to joint configuration
            robot.sim_set_arm_to_joint_values(config)
            client.run(30)
            print(f"[{name}] Arm configuration {i+1}/{len(joint_configs)} reached")
        except Exception as e:
            print(f"[{name}] Could not reach configuration {i+1}: {e}")
    
    # Reset arm using the correct method
    try:
        reset_joint_values = robot.arm_reset_jointValues
        robot.sim_reset_arm_to_joint_values(reset_joint_values)
        client.run(30)
    except Exception as e:
        print(f"[{name}] Could not reset arm: {e}")
    
    print(f"[{name}] Arm manipulation pattern completed")


def robot_movement_pattern_3(robot_info, client, visualizer):
    """
    Movement pattern 3: Rotation in place
    """
    robot = robot_info['instance']
    name = robot_info['config']['name']
    initial_pose = robot_info['initial_pose']
    
    print(f"[{name}] Starting rotation pattern...")
    
    # Rotate 360 degrees in increments
    num_steps = 8
    for i in range(num_steps + 1):
        angle = (2 * math.pi * i) / num_steps
        
        target_pose = Pose(
            initial_pose.get_position(),
            [0.0, 0.0, angle]
        )
        
        robot.sim_navigate_base(target_pose, [initial_pose.get_position()[:2]])
        client.run(15)
        
        print(f"[{name}] Rotation {i}/{num_steps} complete ({math.degrees(angle):.0f}°)")
    
    print(f"[{name}] Rotation pattern completed")


def robot_movement_pattern_4(robot_info, client, visualizer):
    """
    Movement pattern 4: Forward and backward movement
    """
    robot = robot_info['instance']
    name = robot_info['config']['name']
    initial_pose = robot_info['initial_pose']
    initial_pos = initial_pose.get_position()
    
    print(f"[{name}] Starting forward-backward pattern...")
    
    # Move forward
    for i in range(3):
        forward_pos = [
            initial_pos[0] + (i + 1) * 0.3,
            initial_pos[1],
            0.0
        ]
        target_pose = Pose(forward_pos, initial_pose.get_orientation())
        robot.sim_navigate_base(target_pose, [[forward_pos[0], forward_pos[1]]])
        client.run(20)
        print(f"[{name}] Moved forward step {i+1}/3")
    
    # Move backward
    for i in range(3, 0, -1):
        backward_pos = [
            initial_pos[0] + (i - 1) * 0.3,
            initial_pos[1],
            0.0
        ]
        target_pose = Pose(backward_pos, initial_pose.get_orientation())
        robot.sim_navigate_base(target_pose, [[backward_pos[0], backward_pos[1]]])
        client.run(20)
        print(f"[{name}] Moved backward step {4-i}/3")
    
    print(f"[{name}] Forward-backward pattern completed")


def run_sequential_movements(client, visualizer, robots):
    """
    Run movements for all robots sequentially.
    """
    print("\n" + "=" * 60)
    print("Starting Sequential Robot Movements")
    print("=" * 60)
    
    robot_list = list(robots.items())
    
    # Assign different movement patterns to each robot
    patterns = [
        robot_movement_pattern_1,
        robot_movement_pattern_2,
        robot_movement_pattern_3,
        robot_movement_pattern_4
    ]
    
    for i, (robot_name, robot_info) in enumerate(robot_list):
        pattern = patterns[i % len(patterns)]
        print(f"\n{'='*60}")
        print(f"Controlling: {robot_name}")
        print(f"Pattern: {pattern.__name__}")
        print('='*60)
        
        pattern(robot_info, client, visualizer)
        
        # Wait between robots
        print(f"\nWaiting before next robot...")
        client.run(60)
    
    print("\n" + "=" * 60)
    print("All Sequential Movements Completed")
    print("=" * 60)


def run_parallel_movements(client, visualizer, robots):
    """
    Run parallel movements for all robots simultaneously.
    All robots perform coordinated movements at the same time.
    """
    print("\n" + "=" * 60)
    print("Starting Parallel Robot Movements")
    print("=" * 60)
    
    robot_list = list(robots.items())
    
    print("\n[Phase 1] All robots rotating 90 degrees...")
    # Prepare target rotations (90 degrees from initial orientation)
    target_poses = []
    for robot_name, robot_info in robot_list:
        initial_pose = robot_info['initial_pose']
        current_orientation = initial_pose.get_orientation()
        target_pose = Pose(
            initial_pose.get_position(),
            [current_orientation[0], current_orientation[1], current_orientation[2] + math.pi/2]
        )
        target_poses.append((robot_info, target_pose))
    
    # Execute rotations simultaneously - interleaved steps
    num_steps = 20
    for step in range(num_steps):
        for robot_info, target_pose in target_poses:
            robot = robot_info['instance']
            initial_pose = robot_info['initial_pose']
            
            # Calculate intermediate orientation
            current_euler = robot.sim_get_current_base_pose().get_orientation("euler")
            target_euler = target_pose.get_orientation("euler")
            
            # Interpolate
            alpha = (step + 1) / num_steps
            interp_yaw = initial_pose.get_orientation("euler")[2] + alpha * (math.pi/2)
            
            # Create intermediate pose
            interp_pose = Pose(
                initial_pose.get_position(),
                [0.0, 0.0, interp_yaw]
            )
            
            # Move robot
            robot.sim_navigate_base(interp_pose, [initial_pose.get_position()[:2]])
        
        client.run(5)
        if step % 5 == 0:
            print(f"    Rotation progress: {step}/{num_steps}")
    
    print("\n[Phase 2] All robots moving forward 0.5m...")
    # Move all robots forward
    forward_targets = []
    for robot_name, robot_info in robot_list:
        initial_pose = robot_info['initial_pose']
        initial_pos = initial_pose.get_position()
        current_yaw = robot.sim_get_current_base_pose().get_orientation("euler")[2]
        
        # Calculate forward position
        forward_pos = [
            initial_pos[0] + 0.5 * math.cos(current_yaw),
            initial_pos[1] + 0.5 * math.sin(current_yaw),
            0.0
        ]
        forward_pose = Pose(forward_pos, [0.0, 0.0, current_yaw])
        forward_targets.append((robot_info, forward_pose, initial_pos[:2]))
    
    # Execute forward movement
    num_steps = 20
    for step in range(num_steps):
        for robot_info, target_pose, start_pos in forward_targets:
            robot = robot_info['instance']
            
            # Interpolate position
            alpha = (step + 1) / num_steps
            current_pos = robot.sim_get_current_base_pose().get_position()
            target_pos = target_pose.get_position()
            
            interp_pos = [
                start_pos[0] + alpha * (target_pos[0] - start_pos[0]),
                start_pos[1] + alpha * (target_pos[1] - start_pos[1]),
                0.0
            ]
            
            interp_pose = Pose(interp_pos, target_pose.get_orientation())
            robot.sim_navigate_base(interp_pose, [interp_pos[:2]])
        
        client.run(5)
        if step % 5 == 0:
            print(f"    Forward movement progress: {step}/{num_steps}")
    
    print("\n[Phase 3] All robots performing arm movements...")
    # Arm movements for all robots
    arm_configs = [
        [0.0, -math.pi/4, math.pi/2, -math.pi/4, 0.0, 0.0],
        [math.pi/4, -math.pi/6, math.pi/3, -math.pi/3, math.pi/4, 0.0],
    ]
    
    for config_idx, joint_config in enumerate(arm_configs):
        print(f"    Arm configuration {config_idx + 1}/{len(arm_configs)}")
        for robot_name, robot_info in robot_list:
            robot = robot_info['instance']
            try:
                robot.sim_set_arm_to_joint_values(joint_config)
            except Exception as e:
                print(f"        [{robot_name}] Arm movement failed: {e}")
        client.run(30)
    
    # Reset all arms
    print("    Resetting all arms...")
    for robot_name, robot_info in robot_list:
        robot = robot_info['instance']
        try:
            reset_joint_values = robot.arm_reset_jointValues
            robot.sim_reset_arm_to_joint_values(reset_joint_values)
        except Exception as e:
            print(f"        [{robot_name}] Arm reset failed: {e}")
    client.run(30)
    
    print("\n[Phase 4] All robots returning to initial positions...")
    # Return to initial positions
    for step in range(num_steps):
        for robot_name, robot_info in robot_list:
            robot = robot_info['instance']
            initial_pose = robot_info['initial_pose']
            current_pos = robot.sim_get_current_base_pose().get_position()
            initial_pos = initial_pose.get_position()
            
            # Interpolate back to start
            alpha = (step + 1) / num_steps
            interp_pos = [
                current_pos[0] + alpha * (initial_pos[0] - current_pos[0]),
                current_pos[1] + alpha * (initial_pos[1] - current_pos[1]),
                0.0
            ]
            
            # Calculate orientation to face initial direction
            interp_pose = Pose(interp_pos, initial_pose.get_orientation())
            robot.sim_navigate_base(interp_pose, [interp_pos[:2]])
        
        client.run(5)
    
    print("\n" + "=" * 60)
    print("Parallel Movements Completed")
    print("=" * 60)


def demo_all_robots():
    """
    Main demonstration function.
    """
    print("\n" + "=" * 60)
    print("Four Robots Demonstration")
    print("=" * 60)
    
    # Create scene with four robots
    scene_path = "Asset/Scene/Scene/Kitchen.json"
    client, visualizer, robots = create_scene_with_four_robots(scene_path)
    
    if len(robots) < 4:
        print(f"\nWarning: Only {len(robots)} robots initialized successfully")
        print("Continuing with available robots...")
    
    # Wait for scene to settle
    print("\nWaiting for scene to settle...")
    client.run(100)
    
    # Run parallel movements (all robots move simultaneously)
    print("\n>>> Running PARALLEL mode: All robots move at the same time <<<")
    run_parallel_movements(client, visualizer, robots)
    
    # Optional: Run sequential movements (one robot at a time)
    # print("\n>>> Running SEQUENTIAL mode: One robot at a time <<<")
    # run_sequential_movements(client, visualizer, robots)
    
    # Final wait
    print("\nDemo completed. Waiting before shutdown...")
    client.run(100)
    
    # Cleanup
    client.disconnect()
    
    print("\n" + "=" * 60)
    print("Demonstration Finished")
    print("=" * 60)


if __name__ == "__main__":
    # Change to Examples directory (required for config loading)
    os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Examples'))
    
    demo_all_robots()
