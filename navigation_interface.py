#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Navigation Interface Module

This module provides a unified interface for robot navigation functionality,
integrating SLAM, path planning algorithms (A*, RRT, PRM), and robot base control.

Author: BestMan Development Team
Date: 2026-03-24
"""

import math
from typing import List, Optional, Tuple, Dict, Any
from dataclasses import dataclass
from enum import Enum

import numpy as np
from Robotics_API import Pose
from Motion_Planning.Navigation.A_star.A_star import AStarPlanner
from Motion_Planning.Navigation.RRT.rrt import RRTPlanner
from Motion_Planning.Navigation.PRM.probabilistic_road_map import PRMPlanner
from SLAM import simple_slam


class NavigationAlgorithm(Enum):
    """Supported navigation planning algorithms"""
    A_STAR = "a_star"
    RRT = "rrt"
    PRM = "prm"


@dataclass
class NavigationConfig:
    """Configuration for navigation planning"""
    algorithm: NavigationAlgorithm = NavigationAlgorithm.A_STAR
    resolution: float = 0.05
    x_max: float = 10.0
    y_max: float = 10.0
    enable_plot: bool = False
    robot_size: float = 0.5


@dataclass
class NavigationResult:
    """Result of navigation execution"""
    success: bool
    path: Optional[List[List[float]]]
    execution_time: float
    distance_traveled: float
    error_message: Optional[str] = None


class NavigationInterface:
    """
    Unified interface for robot navigation operations.
    
    This class encapsulates all navigation-related functionality including:
    - Environment mapping using SLAM
    - Path planning using various algorithms (A*, RRT, PRM)
    - Robot base navigation execution
    - Real-time path visualization
    
    Attributes:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Visualization instance
        config: Navigation configuration
        planner: Current path planner instance
        nav_obstacles_bounds: Navigation obstacle boundaries
    """
    
    def __init__(
        self,
        client,
        robot,
        visualizer,
        config: Optional[NavigationConfig] = None
    ):
        """
        Initialize the navigation interface.
        
        Args:
            client: PyBullet client instance
            robot: BestMan robot instance
            visualizer: Visualization instance
            config: Navigation configuration (uses default if None)
        """
        self.client = client
        self.robot = robot
        self.visualizer = visualizer
        self.config = config or NavigationConfig()
        
        self.planner = None
        self.nav_obstacles_bounds = None
        self._current_path = None
        self._path_history = []
        
    def initialize_mapping(
        self,
        enable_slam: bool = True,
        draw_obstacles: bool = True
    ) -> List[Tuple[float, float, float, float]]:
        """
        Initialize environment mapping using SLAM.
        
        Args:
            enable_slam: Whether to enable SLAM mapping
            draw_obstacles: Whether to visualize obstacles
            
        Returns:
            List of obstacle bounding boxes [(x_min, y_min, x_max, y_max), ...]
        """
        self.nav_obstacles_bounds = simple_slam(
            self.client,
            self.robot,
            draw_obstacles
        )
        return self.nav_obstacles_bounds
    
    def create_planner(
        self,
        algorithm: Optional[NavigationAlgorithm] = None,
        **kwargs
    ) -> Any:
        """
        Create a path planner instance.
        
        Args:
            algorithm: Navigation algorithm to use (uses config default if None)
            **kwargs: Additional planner-specific parameters
            
        Returns:
            Planner instance
        """
        algo = algorithm or self.config.algorithm
        robot_size = kwargs.get('robot_size', self.config.robot_size)
        
        if self.nav_obstacles_bounds is None:
            raise RuntimeError("Must call initialize_mapping() before creating planner")
        
        if algo == NavigationAlgorithm.A_STAR:
            self.planner = AStarPlanner(
                robot_size=robot_size,
                obstacles_bounds=self.nav_obstacles_bounds,
                resolution=kwargs.get('resolution', self.config.resolution),
                x_max=kwargs.get('x_max', self.config.x_max),
                y_max=kwargs.get('y_max', self.config.y_max),
                enable_plot=kwargs.get('enable_plot', self.config.enable_plot)
            )
        elif algo == NavigationAlgorithm.RRT:
            self.planner = RRTPlanner(
                robot_size=robot_size,
                obstacles_bounds=self.nav_obstacles_bounds,
                enable_plot=kwargs.get('enable_plot', self.config.enable_plot)
            )
        elif algo == NavigationAlgorithm.PRM:
            self.planner = PRMPlanner(
                robot_size=robot_size,
                obstacles_bounds=self.nav_obstacles_bounds,
                enable_plot=kwargs.get('enable_plot', self.config.enable_plot)
            )
        else:
            raise ValueError(f"Unknown navigation algorithm: {algo}")
            
        return self.planner
    
    def plan_path(
        self,
        goal_pose: Pose,
        start_pose: Optional[Pose] = None
    ) -> Optional[List[List[float]]]:
        """
        Plan a path from current position to goal.
        
        Args:
            goal_pose: Target pose for navigation
            start_pose: Starting pose (uses current robot pose if None)
            
        Returns:
            List of waypoints [[x, y], ...] or None if planning fails
        """
        if self.planner is None:
            raise RuntimeError("Must call create_planner() before planning")
        
        start = start_pose or self.robot.sim_get_current_base_pose()
        
        path = self.planner.plan(start, goal_pose)
        self._current_path = path
        
        if path:
            self._path_history.append({
                'start': start,
                'goal': goal_pose,
                'path': path
            })
        
        return path
    
    def navigate_to_pose(
        self,
        target_pose: Pose,
        path: Optional[List[List[float]]] = None,
        enable_plot: bool = True
    ) -> NavigationResult:
        """
        Navigate robot to target pose.
        
        Args:
            target_pose: Target pose for navigation
            path: Pre-computed path (will plan if None)
            enable_plot: Whether to visualize navigation
            
        Returns:
            NavigationResult with execution details
        """
        import time
        start_time = time.time()
        
        try:
            # Plan path if not provided
            if path is None:
                path = self.plan_path(target_pose)
                if path is None:
                    return NavigationResult(
                        success=False,
                        path=None,
                        execution_time=time.time() - start_time,
                        distance_traveled=0.0,
                        error_message="Path planning failed"
                    )
            
            # Execute navigation
            self.robot.sim_navigate_base(target_pose, path, enable_plot=enable_plot)
            
            # Calculate distance traveled
            distance = self._calculate_path_distance(path)
            
            execution_time = time.time() - start_time
            
            return NavigationResult(
                success=True,
                path=path,
                execution_time=execution_time,
                distance_traveled=distance
            )
            
        except Exception as e:
            return NavigationResult(
                success=False,
                path=path,
                execution_time=time.time() - start_time,
                distance_traveled=0.0,
                error_message=str(e)
            )
    
    def navigate_to_object(
        self,
        object_name: str,
        standing_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        **kwargs
    ) -> NavigationResult:
        """
        Navigate to a suitable standing position near an object.
        
        Args:
            object_name: Name of target object
            standing_offset: Offset from object position [x, y, z]
            **kwargs: Additional navigation parameters
            
        Returns:
            NavigationResult with execution details
        """
        # Get object position
        object_id = self.client.resolve_object_id(object_name)
        object_pose = self.client.get_object_pose(object_id)
        object_position = object_pose.get_position()
        
        # Calculate standing position
        target_position = [
            object_position[0] + standing_offset[0],
            object_position[1] + standing_offset[1],
            standing_offset[2]
        ]
        
        # Calculate orientation to face object
        dx = object_position[0] - target_position[0]
        dy = object_position[1] - target_position[1]
        target_yaw = math.atan2(dy, dx)
        
        target_pose = Pose(target_position, [0.0, 0.0, target_yaw])
        
        return self.navigate_to_pose(target_pose, **kwargs)
    
    def get_current_pose(self) -> Pose:
        """Get current robot base pose"""
        return self.robot.sim_get_current_base_pose()
    
    def get_path_history(self) -> List[Dict]:
        """Get history of all planned paths"""
        return self._path_history.copy()
    
    def clear_path_history(self):
        """Clear path history"""
        self._path_history = []
    
    def _calculate_path_distance(self, path: List[List[float]]) -> float:
        """Calculate total distance of a path"""
        distance = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            distance += math.sqrt(dx**2 + dy**2)
        return distance
    
    def visualize_path(self, path: Optional[List[List[float]]] = None):
        """
        Visualize a path in the simulation.
        
        Args:
            path: Path to visualize (uses current path if None)
        """
        path = path or self._current_path
        if path is None:
            return
        
        # Draw path waypoints
        for i, waypoint in enumerate(path):
            self.visualizer.draw_point(
                position=[waypoint[0], waypoint[1], 0.05],
                color=[0, 1, 0],
                size=0.05
            )


def create_navigation_interface(
    client,
    robot,
    visualizer,
    algorithm: NavigationAlgorithm = NavigationAlgorithm.A_STAR,
    **kwargs
) -> NavigationInterface:
    """
    Factory function to create a navigation interface with standard configuration.
    
    Args:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Visualization instance
        algorithm: Navigation algorithm to use
        **kwargs: Additional configuration parameters
        
    Returns:
        Configured NavigationInterface instance
    """
    config = NavigationConfig(algorithm=algorithm, **kwargs)
    nav_interface = NavigationInterface(client, robot, visualizer, config)
    nav_interface.initialize_mapping()
    nav_interface.create_planner()
    return nav_interface
