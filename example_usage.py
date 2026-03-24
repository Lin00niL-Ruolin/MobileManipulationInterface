#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Mobile Manipulation Interface - Example Usage

This example demonstrates how to use the MobileManipulationInterface
to perform complex mobile manipulation tasks in a kitchen environment.

Author: BestMan Development Team
Date: 2026-03-24
"""

import os
import sys
import math

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from Config import load_config
from Env import Client
from Robotics_API import Bestman_sim_ur5e_vacuum_long, Pose
from Visualization import Visualizer

from MobileManipulationInterface import (
    MobileManipulationInterface,
    TaskConfig,
    TaskSequence,
    NavigationAlgorithm,
    GraspType,
    PlacementType,
    VisualizationMode,
    create_mobile_manipulation_interface
)


def example_basic_pick_and_place():
    """
    Example: Basic pick and place operation
    
    This example shows how to:
    1. Initialize the mobile manipulation interface
    2. Navigate to an object
    3. Pick up the object
    4. Navigate to a destination
    5. Place the object
    """
    print("=" * 60)
    print("Example: Basic Pick and Place")
    print("=" * 60)
    
    # Load configuration
    config_path = "Config/grasp_bowl_in_kitchen.yaml"
    cfg = load_config(config_path)
    
    # Initialize client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Load scene
    scene_path = "Asset/Scene/Scene/Kitchen.json"
    client.create_scene(scene_path)
    
    # Initialize robot
    robot = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Create mobile manipulation interface
    mm_interface = create_mobile_manipulation_interface(
        client=client,
        robot=robot,
        visualizer=visualizer,
        navigation_algorithm=NavigationAlgorithm.A_STAR,
        grasp_type=GraspType.VACUUM,
        placement_type=PlacementType.ON_SURFACE,
        visualization_mode=VisualizationMode.REALTIME,
        enable_trajectory_trail=True,
        enable_metrics=True
    )
    
    # Load objects
    bowl_id = client.load_object(
        "bowl",
        "Asset/Scene/Object/URDF_models/utensil_bowl_blue/model.urdf",
        [3.8, 2.4, 0.95],
        [0.0, 0.0, 0.0],
        1.0,
    )
    
    table_id = client.load_object(
        "table",
        "Asset/Scene/Object/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        True,
    )
    
    print("\n[1] Picking up bowl...")
    pick_result = mm_interface.pick_object(
        object_name="bowl",
        standing_offset=[-0.5, 0.0, 0.0]
    )
    
    print(f"    Success: {pick_result.success}")
    print(f"    Execution time: {pick_result.execution_time:.2f}s")
    if pick_result.navigation_result:
        print(f"    Navigation distance: {pick_result.navigation_result.distance_traveled:.2f}m")
    
    print("\n[2] Placing bowl on table...")
    place_result = mm_interface.place_object(
        object_name="bowl",
        to_location="table",
        relative_position=[0.0, 0.0, 0.0]
    )
    
    print(f"    Success: {place_result.success}")
    print(f"    Execution time: {place_result.execution_time:.2f}s")
    
    # Get task summary
    summary = mm_interface.get_task_summary()
    print("\n[Task Summary]")
    print(f"    Total tasks: {summary['total_tasks']}")
    print(f"    Successful: {summary['successful_tasks']}")
    print(f"    Failed: {summary['failed_tasks']}")
    print(f"    Total time: {summary['total_execution_time']:.2f}s")
    
    # Cleanup
    mm_interface.shutdown()
    client.wait(5)
    client.disconnect()
    
    print("\nExample completed!")


def example_move_from_drawer_to_table():
    """
    Example: Move object from drawer to table
    
    This example demonstrates retrieving an object from a container
    (drawer) and placing it on a table.
    """
    print("=" * 60)
    print("Example: Move Object from Drawer to Table")
    print("=" * 60)
    
    # Load configuration
    config_path = "Config/move_bowl_from_drawer_to_table.yaml"
    cfg = load_config(config_path)
    
    # Initialize client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    
    # Load scene
    scene_path = "Asset/Scene/Scene/Kitchen_1.json"
    client.create_scene(scene_path)
    
    # Initialize robot
    robot = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Create mobile manipulation interface
    mm_interface = create_mobile_manipulation_interface(
        client=client,
        robot=robot,
        visualizer=visualizer,
        navigation_algorithm=NavigationAlgorithm.A_STAR,
        grasp_type=GraspType.VACUUM,
        placement_type=PlacementType.ON_SURFACE
    )
    
    # Open the drawer first
    print("\n[0] Opening drawer...")
    client.change_object_joint_angle("elementA", 36, 0.4)
    client.run(40)
    
    # Load bowl inside drawer
    bowl_id = client.load_object(
        "bowl",
        "Asset/Scene/Object/URDF_models/utensil_bowl_blue/model.urdf",
        [3.6, 2.4, 0.6],
        [0.0, 0.0, 0.0],
        1.0,
        False,
    )
    
    # Load table
    table_id = client.load_object(
        "table",
        "Asset/Scene/Object/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        True,
    )
    
    print("\n[1] Moving bowl from drawer to table...")
    results = mm_interface.move_object(
        object_name="bowl",
        from_location="elementA",  # Drawer
        to_location="table"
    )
    
    for i, result in enumerate(results):
        print(f"    Step {i+1}: {result.task_name}")
        print(f"        Success: {result.success}")
        print(f"        Time: {result.execution_time:.2f}s")
    
    # Get task summary
    summary = mm_interface.get_task_summary()
    print("\n[Task Summary]")
    print(f"    Total tasks: {summary['total_tasks']}")
    print(f"    Success rate: {summary['success_rate']*100:.1f}%")
    
    # Cleanup
    mm_interface.shutdown()
    client.wait(5)
    client.disconnect()
    
    print("\nExample completed!")


def example_task_sequence():
    """
    Example: Execute a sequence of tasks
    
    This example shows how to define and execute a complex task
    sequence involving multiple pick and place operations.
    """
    print("=" * 60)
    print("Example: Task Sequence Execution")
    print("=" * 60)
    
    # Load configuration
    config_path = "Config/grasp_bowl_in_kitchen.yaml"
    cfg = load_config(config_path)
    
    # Initialize client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Load scene
    scene_path = "Asset/Scene/Scene/Kitchen.json"
    client.create_scene(scene_path)
    
    # Initialize robot
    robot = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Create mobile manipulation interface
    mm_interface = create_mobile_manipulation_interface(
        client=client,
        robot=robot,
        visualizer=visualizer,
        navigation_algorithm=NavigationAlgorithm.A_STAR,
        grasp_type=GraspType.VACUUM,
        placement_type=PlacementType.ON_SURFACE,
        visualization_mode=VisualizationMode.REALTIME
    )
    
    # Load multiple objects
    bowl1_id = client.load_object(
        "bowl1",
        "Asset/Scene/Object/URDF_models/utensil_bowl_blue/model.urdf",
        [3.8, 2.4, 0.95],
        [0.0, 0.0, 0.0],
        1.0,
    )
    
    bowl2_id = client.load_object(
        "bowl2",
        "Asset/Scene/Object/URDF_models/utensil_bowl_red/model.urdf",
        [3.8, 2.6, 0.95],
        [0.0, 0.0, 0.0],
        1.0,
    )
    
    table_id = client.load_object(
        "table",
        "Asset/Scene/Object/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        True,
    )
    
    # Create task sequence
    sequence = TaskSequence(name="Organize Kitchen Items")
    
    # Add tasks to sequence
    sequence.add_pick_task("bowl1")
    sequence.add_place_task("bowl1", "table")
    sequence.add_navigate_task("bowl2")
    sequence.add_pick_task("bowl2")
    sequence.add_place_task("bowl2", "table")
    
    print(f"\nExecuting task sequence: {sequence.name}")
    print(f"Number of steps: {len(sequence.steps)}")
    
    # Execute sequence
    results = mm_interface.execute_task_sequence(sequence)
    
    print("\n[Execution Results]")
    for i, result in enumerate(results):
        print(f"    Step {i+1}: {result.task_name}")
        print(f"        Success: {result.success}")
        print(f"        Time: {result.execution_time:.2f}s")
        if result.error_message:
            print(f"        Error: {result.error_message}")
    
    # Get task summary
    summary = mm_interface.get_task_summary()
    print("\n[Task Summary]")
    print(f"    Total tasks: {summary['total_tasks']}")
    print(f"    Successful: {summary['successful_tasks']}")
    print(f"    Success rate: {summary['success_rate']*100:.1f}%")
    print(f"    Total time: {summary['total_execution_time']:.2f}s")
    
    # Export trajectory data
    if mm_interface.sim_visualizer:
        mm_interface.sim_visualizer.export_trajectory_data(
            "task_sequence_trajectory.json"
        )
        print("\n    Trajectory data exported to: task_sequence_trajectory.json")
    
    # Cleanup
    mm_interface.shutdown()
    client.wait(5)
    client.disconnect()
    
    print("\nExample completed!")


def example_with_visualization():
    """
    Example: Demonstrate visualization features
    
    This example showcases the visualization capabilities including
    trajectory trails, phase indicators, and metrics display.
    """
    print("=" * 60)
    print("Example: Visualization Features")
    print("=" * 60)
    
    # Load configuration
    config_path = "Config/navigation_basic.yaml"
    cfg = load_config(config_path)
    
    # Initialize client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Load scene
    scene_path = "Asset/Scene/Scene/Kitchen.json"
    client.create_scene(scene_path)
    
    # Initialize robot
    robot = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Create mobile manipulation interface with full visualization
    mm_interface = create_mobile_manipulation_interface(
        client=client,
        robot=robot,
        visualizer=visualizer,
        navigation_algorithm=NavigationAlgorithm.A_STAR,
        grasp_type=GraspType.VACUUM,
        visualization_mode=VisualizationMode.REALTIME,
        enable_trajectory_trail=True,
        enable_metrics=True
    )
    
    # Load objects
    bowl_id = client.load_object(
        "bowl",
        "Asset/Scene/Object/URDF_models/utensil_bowl_blue/model.urdf",
        [3.8, 2.4, 0.95],
        [0.0, 0.0, 0.0],
        1.0,
    )
    
    table_id = client.load_object(
        "table",
        "Asset/Scene/Object/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        True,
    )
    
    print("\n[1] Visualizing navigation path...")
    
    # Navigate to bowl with visualization
    nav_result = mm_interface.nav_interface.navigate_to_object(
        "bowl",
        standing_offset=[-0.5, 0.0, 0.0]
    )
    
    if nav_result.success and nav_result.path:
        # Visualize the path
        mm_interface.sim_visualizer.visualize_navigation_path(
            nav_result.path,
            color=[0.0, 1.0, 0.0],
            line_width=3.0
        )
        
        # Highlight target object
        mm_interface.sim_visualizer.highlight_object("bowl")
        
        # Show metrics
        mm_interface.sim_visualizer.show_metrics_panel({
            'Navigation Distance': f"{nav_result.distance_traveled:.2f}m",
            'Execution Time': f"{nav_result.execution_time:.2f}s",
            'Path Waypoints': len(nav_result.path)
        })
        
        print(f"    Path visualized with {len(nav_result.path)} waypoints")
    
    print("\n[2] Capturing screenshots...")
    
    # Capture screenshot
    screenshot_path = mm_interface.sim_visualizer.capture_screenshot(
        "visualization_example.png"
    )
    print(f"    Screenshot saved to: {screenshot_path}")
    
    # Get visualization summary
    vis_summary = mm_interface.sim_visualizer.create_task_summary()
    print("\n[Visualization Summary]")
    print(f"    Total time: {vis_summary.get('total_time', 0):.2f}s")
    print(f"    Distance traveled: {vis_summary.get('total_distance', 0):.2f}m")
    print(f"    States recorded: {vis_summary.get('num_states_recorded', 0)}")
    
    # Cleanup
    mm_interface.shutdown()
    client.wait(5)
    client.disconnect()
    
    print("\nExample completed!")


def main():
    """Main function to run all examples"""
    print("\n" + "=" * 60)
    print("Mobile Manipulation Interface - Examples")
    print("=" * 60 + "\n")
    
    # Change to Examples directory
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # Run examples
    examples = [
        ("Basic Pick and Place", example_basic_pick_and_place),
        ("Move from Drawer to Table", example_move_from_drawer_to_table),
        ("Task Sequence", example_task_sequence),
        ("Visualization Features", example_with_visualization),
    ]
    
    print("Available examples:")
    for i, (name, _) in enumerate(examples, 1):
        print(f"  {i}. {name}")
    
    print("\nNote: These examples require a running PyBullet GUI.")
    print("Select an example to run (1-4) or 0 to run all:")
    
    # For automated testing, run all examples
    try:
        choice = input("\nEnter choice (0-4): ").strip()
    except EOFError:
        choice = "1"  # Default to first example in non-interactive mode
    
    if choice == "0":
        for name, func in examples:
            print(f"\n\nRunning: {name}")
            print("-" * 60)
            try:
                func()
            except Exception as e:
                print(f"Error in {name}: {e}")
    elif choice.isdigit() and 1 <= int(choice) <= len(examples):
        examples[int(choice) - 1][1]()
    else:
        print("Invalid choice. Running basic example...")
        example_basic_pick_and_place()
    
    print("\n" + "=" * 60)
    print("All examples completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
