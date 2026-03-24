#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Object Placement Interface Module

This module provides a unified interface for object placement operations,
including target location selection, motion planning, and placement execution.

Author: BestMan Development Team
Date: 2026-03-24
"""

import math
from typing import List, Optional, Tuple, Dict, Any, Union
from dataclasses import dataclass
from enum import Enum
import numpy as np

import pybullet as p
from Robotics_API import Pose
from Motion_Planning.Manipulation.OMPL_Planner import OMPL_Planner


class PlacementType(Enum):
    """Supported placement types"""
    ON_SURFACE = "on_surface"
    IN_CONTAINER = "in_container"
    STACK = "stack"
    INSERT = "insert"


class PlacementPrecision(Enum):
    """Placement precision levels"""
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


@dataclass
class PlacementConfig:
    """Configuration for placement operations"""
    placement_type: PlacementType = PlacementType.ON_SURFACE
    precision: PlacementPrecision = PlacementPrecision.MEDIUM
    place_height_offset: float = 0.02
    approach_height: float = 0.15
    release_delay: float = 0.5
    retreat_height: float = 0.1


@dataclass
class PlacementTarget:
    """Target location for object placement"""
    location_name: str
    position: List[float]
    orientation: List[float]
    surface_height: float = 0.0
    container_id: Optional[int] = None


@dataclass
class PlacementResult:
    """Result of object placement execution"""
    success: bool
    object_name: str
    target_location: str
    final_pose: Optional[Pose]
    execution_time: float
    error_message: Optional[str] = None


class ObjectPlacementInterface:
    """
    Unified interface for object placement operations.
    
    This class encapsulates all object placement functionality including:
    - Target location detection and selection
    - Placement pose calculation
    - Motion planning for placement
    - Gripper release control
    - Post-placement verification
    
    Attributes:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Visualization instance
        planner: OMPL motion planner
        config: Placement configuration
        placement_history: History of placement operations
    """
    
    def __init__(
        self,
        client,
        robot,
        visualizer,
        planner_config: Optional[Dict] = None,
        placement_config: Optional[PlacementConfig] = None
    ):
        """
        Initialize the object placement interface.
        
        Args:
            client: PyBullet client instance
            robot: BestMan robot instance
            visualizer: Visualization instance
            planner_config: Configuration for motion planner
            placement_config: Configuration for placement operations
        """
        self.client = client
        self.robot = robot
        self.visualizer = visualizer
        self.planner_config = planner_config or {}
        self.config = placement_config or PlacementConfig()
        
        self.planner = None
        self._placement_history = []
        self._placed_objects = {}
        
    def initialize_planner(self, obstacles_info: bool = True):
        """
        Initialize the motion planner.
        
        Args:
            obstacles_info: Whether to get obstacle information
        """
        self.planner = OMPL_Planner(self.robot, self.planner_config)
        if obstacles_info:
            self.planner.get_obstacles_info()
    
    def detect_placement_surface(
        self,
        surface_name: str,
        visualize: bool = True
    ) -> Optional[PlacementTarget]:
        """
        Detect a surface for object placement.
        
        Args:
            surface_name: Name of the surface object
            visualize: Whether to visualize the surface
            
        Returns:
            PlacementTarget or None if detection fails
        """
        try:
            surface_id = self.client.resolve_object_id(surface_name)
            surface_pose = self.client.get_object_pose(surface_id)
            
            # Get surface bounding box
            aabb = self.client.get_bounding_box(surface_id)
            surface_height = aabb[1][2]  # Max z of bounding box
            
            position = surface_pose.get_position()
            orientation = surface_pose.get_orientation()
            
            if visualize:
                self.visualizer.draw_aabb(surface_id)
            
            return PlacementTarget(
                location_name=surface_name,
                position=position,
                orientation=orientation,
                surface_height=surface_height
            )
        except Exception as e:
            print(f"[ObjectPlacement] Surface detection failed: {e}")
            return None
    
    def calculate_placement_pose(
        self,
        target: PlacementTarget,
        object_name: Optional[str] = None,
        relative_position: Optional[List[float]] = None
    ) -> Pose:
        """
        Calculate placement pose for an object.
        
        Args:
            target: Placement target
            object_name: Name of object being placed (for size calculation)
            relative_position: Relative position on surface [x, y, z]
            
        Returns:
            Placement pose
        """
        # Calculate target position
        if relative_position:
            place_position = [
                target.position[0] + relative_position[0],
                target.position[1] + relative_position[1],
                target.surface_height + self.config.place_height_offset + relative_position[2]
            ]
        else:
            # Center on surface
            place_position = [
                target.position[0],
                target.position[1],
                target.surface_height + self.config.place_height_offset
            ]
        
        # Default placement orientation (upright)
        place_orientation = [0.0, math.pi / 2.0, 0.0]
        
        return Pose(place_position, place_orientation)
    
    def plan_placement_motion(
        self,
        placement_pose: Pose,
        start_joints: Optional[List[float]] = None,
        approach_offset: Optional[float] = None
    ) -> Optional[Dict[str, List[List[float]]]]:
        """
        Plan motion for object placement with approach and retreat.
        
        Args:
            placement_pose: Target placement pose
            start_joints: Starting joint configuration
            approach_offset: Height offset for approach
            
        Returns:
            Dictionary with 'approach', 'place', and 'retreat' trajectories
        """
        if self.planner is None:
            raise RuntimeError("Must call initialize_planner() before planning")
        
        offset = approach_offset or self.config.approach_height
        start = start_joints or self.robot.sim_get_current_joint_values()
        
        # Calculate approach pose (above placement position)
        approach_position = placement_pose.get_position().copy()
        approach_position[2] += offset
        approach_pose = Pose(approach_position, placement_pose.get_orientation())
        
        # Plan approach trajectory
        approach_goal = self.planner.set_target_pose(approach_pose)
        if approach_goal is None:
            return None
        
        approach_path = self.planner.plan(start, approach_goal)
        if approach_path is None:
            return None
        
        # Plan placement trajectory (approach to placement)
        place_goal = self.planner.set_target_pose(placement_pose)
        if place_goal is None:
            return None
        
        place_path = self.planner.plan(approach_goal, place_goal)
        
        # Plan retreat trajectory (reverse of approach)
        retreat_path = approach_path[::-1] if approach_path else None
        
        return {
            'approach': approach_path,
            'place': place_path,
            'retreat': retreat_path
        }
    
    def execute_placement(
        self,
        object_name: str,
        target_location: str,
        placement_pose: Optional[Pose] = None,
        relative_position: Optional[List[float]] = None,
        enable_plot: bool = True
    ) -> PlacementResult:
        """
        Execute complete object placement sequence.
        
        Args:
            object_name: Name of object to place
            target_location: Name of target location/surface
            placement_pose: Pre-calculated placement pose
            relative_position: Relative position on surface
            enable_plot: Whether to visualize execution
            
        Returns:
            PlacementResult with execution details
        """
        import time
        start_time = time.time()
        
        try:
            # Detect target surface
            target = self.detect_placement_surface(target_location)
            if target is None:
                return PlacementResult(
                    success=False,
                    object_name=object_name,
                    target_location=target_location,
                    final_pose=None,
                    execution_time=time.time() - start_time,
                    error_message="Target surface detection failed"
                )
            
            # Calculate placement pose if not provided
            if placement_pose is None:
                placement_pose = self.calculate_placement_pose(
                    target, object_name, relative_position
                )
            
            # Plan placement motion
            trajectories = self.plan_placement_motion(placement_pose)
            if trajectories is None:
                return PlacementResult(
                    success=False,
                    object_name=object_name,
                    target_location=target_location,
                    final_pose=None,
                    execution_time=time.time() - start_time,
                    error_message="Motion planning failed"
                )
            
            # Execute approach
            if trajectories['approach']:
                self.robot.sim_execute_trajectory(
                    trajectories['approach'],
                    enable_plot=enable_plot
                )
            
            # Execute placement
            if trajectories['place']:
                self.robot.sim_execute_trajectory(
                    trajectories['place'],
                    enable_plot=enable_plot
                )
            else:
                # Direct movement to placement pose
                self.robot.sim_move_eef_to_goal_pose(placement_pose)
            
            # Release object
            self._release_object()
            
            # Execute retreat
            if trajectories['retreat']:
                self.robot.sim_execute_trajectory(
                    trajectories['retreat'],
                    enable_plot=enable_plot
                )
            
            # Record placement
            self._placed_objects[object_name] = target_location
            
            execution_time = time.time() - start_time
            
            result = PlacementResult(
                success=True,
                object_name=object_name,
                target_location=target_location,
                final_pose=placement_pose,
                execution_time=execution_time
            )
            
            self._placement_history.append(result)
            return result
            
        except Exception as e:
            return PlacementResult(
                success=False,
                object_name=object_name,
                target_location=target_location,
                final_pose=None,
                execution_time=time.time() - start_time,
                error_message=str(e)
            )
    
    def place_in_container(
        self,
        object_name: str,
        container_name: str,
        close_container: bool = True,
        close_joint_id: int = 0
    ) -> PlacementResult:
        """
        Place object inside a container (drawer, cabinet, etc.).
        
        Args:
            object_name: Name of object to place
            container_name: Name of container
            close_container: Whether to close container after placement
            close_joint_id: Joint ID for closing container
            
        Returns:
            PlacementResult with execution details
        """
        # Execute placement inside container
        result = self.execute_placement(object_name, container_name)
        
        if result.success and close_container:
            # Close container
            self.client.change_object_joint_angle(container_name, close_joint_id, 0.0)
            self.client.run(40)
        
        return result
    
    def stack_object(
        self,
        object_name: str,
        base_object_name: str,
        stack_offset: List[float] = [0.0, 0.0, 0.0]
    ) -> PlacementResult:
        """
        Stack object on top of another object.
        
        Args:
            object_name: Name of object to stack
            base_object_name: Name of base object
            stack_offset: Offset from base object center
            
        Returns:
            PlacementResult with execution details
        """
        # Get base object pose and height
        base_id = self.client.resolve_object_id(base_object_name)
        base_aabb = self.client.get_bounding_box(base_id)
        base_height = base_aabb[1][2]
        
        # Calculate stack position
        base_pose = self.client.get_object_pose(base_id)
        base_position = base_pose.get_position()
        
        stack_position = [
            base_position[0] + stack_offset[0],
            base_position[1] + stack_offset[1],
            base_height + stack_offset[2]
        ]
        
        # Create placement target
        target = PlacementTarget(
            location_name=base_object_name,
            position=base_position,
            orientation=base_pose.get_orientation(),
            surface_height=base_height
        )
        
        # Calculate placement pose
        placement_pose = self.calculate_placement_pose(
            target,
            object_name,
            [stack_offset[0], stack_offset[1], stack_offset[2]]
        )
        
        # Execute placement
        return self.execute_placement(
            object_name,
            base_object_name,
            placement_pose=placement_pose
        )
    
    def verify_placement(
        self,
        object_name: str,
        expected_location: str,
        tolerance: float = 0.05
    ) -> bool:
        """
        Verify that object is at expected location.
        
        Args:
            object_name: Name of object to verify
            expected_location: Expected location name
            tolerance: Position tolerance in meters
            
        Returns:
            True if placement is correct
        """
        try:
            object_id = self.client.resolve_object_id(object_name)
            object_pose = self.client.get_object_pose(object_id)
            object_position = object_pose.get_position()
            
            # Get expected location
            location_id = self.client.resolve_object_id(expected_location)
            location_pose = self.client.get_object_pose(location_id)
            location_aabb = self.client.get_bounding_box(location_id)
            location_position = location_pose.get_position()
            
            # Check if object is within tolerance
            dx = abs(object_position[0] - location_position[0])
            dy = abs(object_position[1] - location_position[1])
            dz = abs(object_position[2] - location_aabb[1][2])
            
            return dx <= tolerance and dy <= tolerance and dz <= tolerance
        except Exception as e:
            print(f"[ObjectPlacement] Verification failed: {e}")
            return False
    
    def get_placement_history(self) -> List[PlacementResult]:
        """Get history of all placement operations"""
        return self._placement_history.copy()
    
    def get_placed_objects(self) -> Dict[str, str]:
        """Get dictionary of placed objects and their locations"""
        return self._placed_objects.copy()
    
    def _release_object(self):
        """Release the currently held object"""
        import time
        
        # Small delay before release
        time.sleep(self.config.release_delay)
        
        # Release based on gripper type
        if hasattr(self.robot, 'sim_close_vacuum_gripper'):
            # Vacuum gripper - deactivate
            self.robot.sim_close_vacuum_gripper()
        elif hasattr(self.robot, 'sim_open_gripper'):
            # Standard gripper - open
            self.robot.sim_open_gripper()
        
        # Allow physics to settle
        self.client.run(20)
    
    def visualize_placement_pose(
        self,
        placement_pose: Pose,
        duration: float = 5.0
    ):
        """
        Visualize placement pose in simulation.
        
        Args:
            placement_pose: Placement pose to visualize
            duration: Visualization duration in seconds
        """
        position = placement_pose.get_position()
        orientation = placement_pose.get_orientation()
        
        # Draw coordinate frame at placement pose
        self.visualizer.draw_coordinate_frame(position, orientation)
        
        # Draw placement point
        self.visualizer.draw_point(position, color=[0, 0, 1], size=0.05)
        
        # Draw approach point
        approach_position = position.copy()
        approach_position[2] += self.config.approach_height
        self.visualizer.draw_point(approach_position, color=[0, 1, 1], size=0.03)


def create_placement_interface(
    client,
    robot,
    visualizer,
    placement_type: PlacementType = PlacementType.ON_SURFACE,
    **kwargs
) -> ObjectPlacementInterface:
    """
    Factory function to create an object placement interface.
    
    Args:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Visualization instance
        placement_type: Type of placement to use
        **kwargs: Additional configuration parameters
        
    Returns:
        Configured ObjectPlacementInterface instance
    """
    placement_config = PlacementConfig(placement_type=placement_type, **kwargs)
    placement_interface = ObjectPlacementInterface(
        client, robot, visualizer, placement_config=placement_config
    )
    placement_interface.initialize_planner()
    return placement_interface
