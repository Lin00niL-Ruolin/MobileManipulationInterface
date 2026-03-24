#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Object Retrieval Interface Module

This module provides a unified interface for object retrieval operations,
including grasp pose estimation, motion planning, and gripper control.

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


class GraspType(Enum):
    """Supported grasp types"""
    VACUUM = "vacuum"
    PARALLEL_JAW = "parallel_jaw"
    PINCH = "pinch"
    POWER = "power"


class GraspApproach(Enum):
    """Grasp approach directions"""
    TOP_DOWN = "top_down"
    SIDEWAYS = "sideways"
    FRONT = "front"
    CUSTOM = "custom"


@dataclass
class GraspConfig:
    """Configuration for grasping operations"""
    grasp_type: GraspType = GraspType.VACUUM
    approach: GraspApproach = GraspApproach.TOP_DOWN
    pre_grasp_offset: float = 0.15
    grasp_offset: float = 0.0
    lift_height: float = 0.2
    approach_speed: float = 0.1
    grasp_force: float = 10.0


@dataclass
class RetrievalResult:
    """Result of object retrieval execution"""
    success: bool
    object_name: str
    grasp_pose: Optional[Pose]
    execution_time: float
    trajectory_length: int
    error_message: Optional[str] = None


class ObjectRetrievalInterface:
    """
    Unified interface for object retrieval operations.
    
    This class encapsulates all object retrieval functionality including:
    - Object detection and pose estimation
    - Grasp pose calculation
    - Motion planning for reaching objects
    - Gripper control for grasping
    - Object lifting and retraction
    
    Attributes:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Visualization instance
        planner: OMPL motion planner
        config: Grasp configuration
        grasped_object: Currently grasped object name
    """
    
    def __init__(
        self,
        client,
        robot,
        visualizer,
        planner_config: Optional[Dict] = None,
        grasp_config: Optional[GraspConfig] = None
    ):
        """
        Initialize the object retrieval interface.
        
        Args:
            client: PyBullet client instance
            robot: BestMan robot instance
            visualizer: Visualization instance
            planner_config: Configuration for motion planner
            grasp_config: Configuration for grasping operations
        """
        self.client = client
        self.robot = robot
        self.visualizer = visualizer
        self.planner_config = planner_config or {}
        self.config = grasp_config or GraspConfig()
        
        self.planner = None
        self.grasped_object = None
        self._retrieval_history = []
        
    def initialize_planner(self, obstacles_info: bool = True):
        """
        Initialize the motion planner.
        
        Args:
            obstacles_info: Whether to get obstacle information
        """
        self.planner = OMPL_Planner(self.robot, self.planner_config)
        if obstacles_info:
            self.planner.get_obstacles_info()
    
    def detect_object(
        self,
        object_name: str,
        visualize: bool = True
    ) -> Optional[Pose]:
        """
        Detect object and get its pose.
        
        Args:
            object_name: Name of the object to detect
            visualize: Whether to visualize object pose
            
        Returns:
            Object pose or None if not found
        """
        try:
            object_id = self.client.resolve_object_id(object_name)
            object_pose = self.client.get_object_pose(object_id)
            
            if visualize:
                self.visualizer.draw_object_pose(object_name)
            
            return object_pose
        except Exception as e:
            print(f"[ObjectRetrieval] Object detection failed: {e}")
            return None
    
    def calculate_grasp_pose(
        self,
        object_name: str,
        approach: Optional[GraspApproach] = None,
        custom_orientation: Optional[List[float]] = None
    ) -> Optional[Pose]:
        """
        Calculate grasp pose for an object.
        
        Args:
            object_name: Name of target object
            approach: Grasp approach direction
            custom_orientation: Custom orientation [roll, pitch, yaw] for custom approach
            
        Returns:
            Grasp pose or None if calculation fails
        """
        object_pose = self.detect_object(object_name, visualize=False)
        if object_pose is None:
            return None
        
        object_position = object_pose.get_position()
        approach = approach or self.config.approach
        
        # Calculate grasp position and orientation based on approach
        if approach == GraspApproach.TOP_DOWN:
            grasp_position = [
                object_position[0],
                object_position[1],
                object_position[2] + self.config.grasp_offset
            ]
            grasp_orientation = [0.0, math.pi / 2.0, 0.0]
            
        elif approach == GraspApproach.SIDEWAYS:
            grasp_position = [
                object_position[0] + self.config.grasp_offset,
                object_position[1],
                object_position[2]
            ]
            grasp_orientation = [0.0, 0.0, math.pi / 2.0]
            
        elif approach == GraspApproach.FRONT:
            grasp_position = [
                object_position[0],
                object_position[1] + self.config.grasp_offset,
                object_position[2]
            ]
            grasp_orientation = [math.pi / 2.0, 0.0, 0.0]
            
        elif approach == GraspApproach.CUSTOM and custom_orientation:
            grasp_position = [
                object_position[0],
                object_position[1],
                object_position[2] + self.config.grasp_offset
            ]
            grasp_orientation = custom_orientation
        else:
            raise ValueError(f"Invalid approach: {approach}")
        
        return Pose(grasp_position, grasp_orientation)
    
    def plan_retrieval_motion(
        self,
        object_name: str,
        grasp_pose: Optional[Pose] = None,
        start_joints: Optional[List[float]] = None
    ) -> Optional[List[List[float]]]:
        """
        Plan motion to reach and grasp an object.
        
        Args:
            object_name: Name of target object
            grasp_pose: Target grasp pose (calculated if None)
            start_joints: Starting joint configuration
            
        Returns:
            Joint trajectory or None if planning fails
        """
        if self.planner is None:
            raise RuntimeError("Must call initialize_planner() before planning")
        
        # Calculate grasp pose if not provided
        if grasp_pose is None:
            grasp_pose = self.calculate_grasp_pose(object_name)
            if grasp_pose is None:
                return None
        
        # Set target in planner
        goal_joints = self.planner.set_target_pose(grasp_pose)
        if goal_joints is None:
            return None
        
        # Get start configuration
        start = start_joints or self.robot.sim_get_current_joint_values()
        
        # Plan path
        path = self.planner.plan(start, goal_joints)
        return path
    
    def execute_grasp(
        self,
        object_name: str,
        path: Optional[List[List[float]]] = None,
        enable_plot: bool = True
    ) -> RetrievalResult:
        """
        Execute complete object retrieval sequence.
        
        Args:
            object_name: Name of object to grasp
            path: Pre-planned trajectory (will plan if None)
            enable_plot: Whether to visualize execution
            
        Returns:
            RetrievalResult with execution details
        """
        import time
        start_time = time.time()
        
        try:
            # Plan motion if not provided
            if path is None:
                path = self.plan_retrieval_motion(object_name)
                if path is None:
                    return RetrievalResult(
                        success=False,
                        object_name=object_name,
                        grasp_pose=None,
                        execution_time=time.time() - start_time,
                        trajectory_length=0,
                        error_message="Motion planning failed"
                    )
            
            # Get grasp pose
            grasp_pose = self.calculate_grasp_pose(object_name)
            
            # Execute approach trajectory
            self.robot.sim_execute_trajectory(path, enable_plot=enable_plot)
            
            # Activate gripper based on type
            if self.config.grasp_type == GraspType.VACUUM:
                self.robot.sim_open_vacuum_gripper(object_name)
            else:
                self.robot.sim_close_gripper()
            
            # Lift object
            self._lift_object()
            
            # Retract arm
            self._retract_arm(path)
            
            self.grasped_object = object_name
            
            execution_time = time.time() - start_time
            
            result = RetrievalResult(
                success=True,
                object_name=object_name,
                grasp_pose=grasp_pose,
                execution_time=execution_time,
                trajectory_length=len(path)
            )
            
            self._retrieval_history.append(result)
            return result
            
        except Exception as e:
            return RetrievalResult(
                success=False,
                object_name=object_name,
                grasp_pose=None,
                execution_time=time.time() - start_time,
                trajectory_length=len(path) if path else 0,
                error_message=str(e)
            )
    
    def retrieve_from_container(
        self,
        container_name: str,
        object_name: str,
        open_joint_id: int = 0,
        open_angle: float = 0.5
    ) -> RetrievalResult:
        """
        Retrieve object from a container (drawer, cabinet, etc.).
        
        Args:
            container_name: Name of container
            object_name: Name of object inside container
            open_joint_id: Joint ID to open container
            open_angle: Angle to open container
            
        Returns:
            RetrievalResult with execution details
        """
        # Open container
        self.client.change_object_joint_angle(container_name, open_joint_id, open_angle)
        self.client.run(40)
        
        # Retrieve object
        result = self.execute_grasp(object_name)
        
        return result
    
    def get_grasped_object(self) -> Optional[str]:
        """Get name of currently grasped object"""
        return self.grasped_object
    
    def is_holding_object(self) -> bool:
        """Check if robot is currently holding an object"""
        return self.grasped_object is not None
    
    def get_retrieval_history(self) -> List[RetrievalResult]:
        """Get history of all retrieval operations"""
        return self._retrieval_history.copy()
    
    def _lift_object(self, height: Optional[float] = None):
        """
        Lift grasped object to safe height.
        
        Args:
            height: Lift height (uses config if None)
        """
        lift_height = height or self.config.lift_height
        
        current_pose = self.robot.sim_get_current_arm_pose()
        current_position = current_pose.get_position()
        
        lift_pose = Pose(
            [
                current_position[0],
                current_position[1],
                current_position[2] + lift_height
            ],
            current_pose.get_orientation()
        )
        
        self.robot.sim_move_eef_to_goal_pose(lift_pose)
    
    def _retract_arm(self, approach_path: List[List[float]]):
        """
        Retract arm after grasping.
        
        Args:
            approach_path: Original approach path (reversed for retraction)
        """
        if approach_path and len(approach_path) > 1:
            retract_path = approach_path[::-1]
            self.robot.sim_execute_trajectory(retract_path)
        else:
            # Reset to home position
            self.robot.sim_reset_arm()
    
    def visualize_grasp_pose(self, grasp_pose: Pose, duration: float = 5.0):
        """
        Visualize grasp pose in simulation.
        
        Args:
            grasp_pose: Grasp pose to visualize
            duration: Visualization duration in seconds
        """
        position = grasp_pose.get_position()
        orientation = grasp_pose.get_orientation()
        
        # Draw coordinate frame at grasp pose
        self.visualizer.draw_coordinate_frame(position, orientation)
        
        # Draw grasp point
        self.visualizer.draw_point(position, color=[1, 0, 0], size=0.05)


def create_retrieval_interface(
    client,
    robot,
    visualizer,
    grasp_type: GraspType = GraspType.VACUUM,
    **kwargs
) -> ObjectRetrievalInterface:
    """
    Factory function to create an object retrieval interface.
    
    Args:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Visualization instance
        grasp_type: Type of grasp to use
        **kwargs: Additional configuration parameters
        
    Returns:
        Configured ObjectRetrievalInterface instance
    """
    grasp_config = GraspConfig(grasp_type=grasp_type, **kwargs)
    retrieval_interface = ObjectRetrievalInterface(
        client, robot, visualizer, grasp_config=grasp_config
    )
    retrieval_interface.initialize_planner()
    return retrieval_interface
