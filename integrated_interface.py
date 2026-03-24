#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integrated Mobile Manipulation Interface

This module provides a unified high-level interface that combines navigation,
object retrieval, object placement, and visualization capabilities into a
single cohesive interface for mobile manipulation tasks.

Author: BestMan Development Team
Date: 2026-03-24
"""

import time
from typing import List, Optional, Dict, Any, Callable
from dataclasses import dataclass, field
from enum import Enum

from Robotics_API import Pose
from Config import load_config
from Env import Client
from Visualization import Visualizer

from .navigation_interface import (
    NavigationInterface, NavigationConfig, NavigationResult, NavigationAlgorithm
)
from .object_retrieval_interface import (
    ObjectRetrievalInterface, GraspConfig, RetrievalResult, GraspType
)
from .object_placement_interface import (
    ObjectPlacementInterface, PlacementConfig, PlacementResult, PlacementType
)
from .simulation_visualizer import (
    SimulationVisualizer, VisualizationConfig, VisualizationMode, TaskPhase
)


class TaskStatus(Enum):
    """Task execution status"""
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class TaskConfig:
    """Configuration for mobile manipulation tasks"""
    # Navigation settings
    navigation_algorithm: NavigationAlgorithm = NavigationAlgorithm.A_STAR
    nav_resolution: float = 0.05
    
    # Grasp settings
    grasp_type: GraspType = GraspType.VACUUM
    pre_grasp_offset: float = 0.15
    
    # Placement settings
    placement_type: PlacementType = PlacementType.ON_SURFACE
    place_height_offset: float = 0.02
    
    # Visualization settings
    visualization_mode: VisualizationMode = VisualizationMode.REALTIME
    enable_trajectory_trail: bool = True
    enable_metrics: bool = True
    
    # Execution settings
    auto_initialize: bool = True
    enable_recovery: bool = True


@dataclass
class TaskResult:
    """Result of task execution"""
    success: bool
    task_name: str
    navigation_result: Optional[NavigationResult] = None
    retrieval_result: Optional[RetrievalResult] = None
    placement_result: Optional[PlacementResult] = None
    execution_time: float = 0.0
    error_message: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TaskSequence:
    """Sequence of manipulation tasks"""
    name: str
    steps: List[Dict[str, Any]] = field(default_factory=list)
    results: List[TaskResult] = field(default_factory=list)
    current_step: int = 0
    
    def add_pick_task(self, object_name: str, from_location: Optional[str] = None):
        """Add a pick/retrieval task"""
        self.steps.append({
            'type': 'pick',
            'object': object_name,
            'location': from_location
        })
    
    def add_place_task(self, object_name: str, to_location: str):
        """Add a place task"""
        self.steps.append({
            'type': 'place',
            'object': object_name,
            'location': to_location
        })
    
    def add_navigate_task(self, target: Union[str, Pose], target_type: str = 'object'):
        """Add a navigation task"""
        self.steps.append({
            'type': 'navigate',
            'target': target,
            'target_type': target_type
        })
    
    def is_complete(self) -> bool:
        """Check if all steps are complete"""
        return self.current_step >= len(self.steps)


class MobileManipulationInterface:
    """
    Unified high-level interface for mobile manipulation tasks.
    
    This class integrates all components (navigation, retrieval, placement,
    visualization) into a single cohesive interface for executing complex
    mobile manipulation tasks.
    
    Attributes:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Base visualizer instance
        config: Task configuration
        nav_interface: Navigation interface
        retrieval_interface: Object retrieval interface
        placement_interface: Object placement interface
        sim_visualizer: Simulation visualizer
    """
    
    def __init__(
        self,
        client: Client,
        robot,
        visualizer: Visualizer,
        config: Optional[TaskConfig] = None
    ):
        """
        Initialize the mobile manipulation interface.
        
        Args:
            client: PyBullet client instance
            robot: BestMan robot instance
            visualizer: Base visualizer instance
            config: Task configuration
        """
        self.client = client
        self.robot = robot
        self.visualizer = visualizer
        self.config = config or TaskConfig()
        
        # Initialize sub-interfaces
        self.nav_interface: Optional[NavigationInterface] = None
        self.retrieval_interface: Optional[ObjectRetrievalInterface] = None
        self.placement_interface: Optional[ObjectPlacementInterface] = None
        self.sim_visualizer: Optional[SimulationVisualizer] = None
        
        # State tracking
        self._current_task: Optional[str] = None
        self._held_object: Optional[str] = None
        self._task_history: List[TaskResult] = []
        
        if self.config.auto_initialize:
            self.initialize()
    
    def initialize(self):
        """Initialize all sub-interfaces"""
        # Initialize navigation
        nav_config = NavigationConfig(
            algorithm=self.config.navigation_algorithm,
            resolution=self.config.nav_resolution
        )
        self.nav_interface = NavigationInterface(
            self.client, self.robot, self.visualizer, nav_config
        )
        self.nav_interface.initialize_mapping()
        self.nav_interface.create_planner()
        
        # Initialize retrieval
        grasp_config = GraspConfig(
            grasp_type=self.config.grasp_type,
            pre_grasp_offset=self.config.pre_grasp_offset
        )
        self.retrieval_interface = ObjectRetrievalInterface(
            self.client, self.robot, self.visualizer, grasp_config=grasp_config
        )
        self.retrieval_interface.initialize_planner()
        
        # Initialize placement
        placement_config = PlacementConfig(
            placement_type=self.config.placement_type,
            place_height_offset=self.config.place_height_offset
        )
        self.placement_interface = ObjectPlacementInterface(
            self.client, self.robot, self.visualizer, placement_config=placement_config
        )
        self.placement_interface.initialize_planner()
        
        # Initialize visualizer
        vis_config = VisualizationConfig(
            mode=self.config.visualization_mode,
            enable_trajectory_trail=self.config.enable_trajectory_trail,
            enable_metrics_panel=self.config.enable_metrics
        )
        self.sim_visualizer = SimulationVisualizer(
            self.client, self.visualizer, vis_config
        )
        self.sim_visualizer.start_visualization()
    
    def pick_object(
        self,
        object_name: str,
        from_location: Optional[str] = None,
        standing_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ) -> TaskResult:
        """
        Execute complete pick operation including navigation and grasping.
        
        Args:
            object_name: Name of object to pick
            from_location: Location/container where object is located
            standing_offset: Offset for robot standing position
            
        Returns:
            TaskResult with execution details
        """
        import time
        start_time = time.time()
        self._current_task = f"pick_{object_name}"
        
        try:
            # Update visualization
            self.sim_visualizer.update_state(
                self.robot,
                TaskPhase.NAVIGATION,
                f"Navigating to {object_name}"
            )
            
            # Navigate to object
            nav_result = self.nav_interface.navigate_to_object(
                object_name,
                standing_offset=standing_offset
            )
            
            if not nav_result.success:
                return TaskResult(
                    success=False,
                    task_name=self._current_task,
                    navigation_result=nav_result,
                    execution_time=time.time() - start_time,
                    error_message=f"Navigation failed: {nav_result.error_message}"
                )
            
            # Visualize navigation path
            if nav_result.path:
                self.sim_visualizer.visualize_navigation_path(nav_result.path)
            
            # Update visualization
            self.sim_visualizer.update_state(
                self.robot,
                TaskPhase.MANIPULATION,
                f"Grasping {object_name}"
            )
            
            # Highlight target object
            self.sim_visualizer.highlight_object(object_name)
            
            # Execute grasp
            if from_location:
                retrieval_result = self.retrieval_interface.retrieve_from_container(
                    from_location, object_name
                )
            else:
                retrieval_result = self.retrieval_interface.execute_grasp(object_name)
            
            if not retrieval_result.success:
                return TaskResult(
                    success=False,
                    task_name=self._current_task,
                    navigation_result=nav_result,
                    retrieval_result=retrieval_result,
                    execution_time=time.time() - start_time,
                    error_message=f"Grasping failed: {retrieval_result.error_message}"
                )
            
            self._held_object = object_name
            
            execution_time = time.time() - start_time
            
            result = TaskResult(
                success=True,
                task_name=self._current_task,
                navigation_result=nav_result,
                retrieval_result=retrieval_result,
                execution_time=execution_time,
                metadata={
                    'object': object_name,
                    'from_location': from_location
                }
            )
            
            self._task_history.append(result)
            return result
            
        except Exception as e:
            return TaskResult(
                success=False,
                task_name=self._current_task,
                execution_time=time.time() - start_time,
                error_message=str(e)
            )
    
    def place_object(
        self,
        object_name: Optional[str] = None,
        to_location: str = "",
        relative_position: Optional[List[float]] = None
    ) -> TaskResult:
        """
        Execute complete place operation including navigation and placement.
        
        Args:
            object_name: Name of object to place (uses held object if None)
            to_location: Target location for placement
            relative_position: Relative position on target surface
            
        Returns:
            TaskResult with execution details
        """
        import time
        start_time = time.time()
        
        object_name = object_name or self._held_object
        if not object_name:
            return TaskResult(
                success=False,
                task_name="place",
                execution_time=0.0,
                error_message="No object specified and no object is being held"
            )
        
        self._current_task = f"place_{object_name}_to_{to_location}"
        
        try:
            # Update visualization
            self.sim_visualizer.update_state(
                self.robot,
                TaskPhase.NAVIGATION,
                f"Navigating to {to_location}"
            )
            
            # Navigate to placement location
            nav_result = self.nav_interface.navigate_to_object(
                to_location,
                standing_offset=[0.0, -0.5, 0.0]  # Stand back from surface
            )
            
            if not nav_result.success:
                return TaskResult(
                    success=False,
                    task_name=self._current_task,
                    navigation_result=nav_result,
                    execution_time=time.time() - start_time,
                    error_message=f"Navigation failed: {nav_result.error_message}"
                )
            
            # Update visualization
            self.sim_visualizer.update_state(
                self.robot,
                TaskPhase.MANIPULATION,
                f"Placing {object_name} on {to_location}"
            )
            
            # Highlight target location
            self.sim_visualizer.highlight_object(to_location, color=[0, 1, 1])
            
            # Execute placement
            placement_result = self.placement_interface.execute_placement(
                object_name,
                to_location,
                relative_position=relative_position
            )
            
            if not placement_result.success:
                return TaskResult(
                    success=False,
                    task_name=self._current_task,
                    navigation_result=nav_result,
                    placement_result=placement_result,
                    execution_time=time.time() - start_time,
                    error_message=f"Placement failed: {placement_result.error_message}"
                )
            
            self._held_object = None
            
            execution_time = time.time() - start_time
            
            result = TaskResult(
                success=True,
                task_name=self._current_task,
                navigation_result=nav_result,
                placement_result=placement_result,
                execution_time=execution_time,
                metadata={
                    'object': object_name,
                    'to_location': to_location
                }
            )
            
            self._task_history.append(result)
            return result
            
        except Exception as e:
            return TaskResult(
                success=False,
                task_name=self._current_task,
                execution_time=time.time() - start_time,
                error_message=str(e)
            )
    
    def move_object(
        self,
        object_name: str,
        from_location: Optional[str],
        to_location: str
    ) -> List[TaskResult]:
        """
        Move object from one location to another.
        
        Args:
            object_name: Name of object to move
            from_location: Source location (None if object is in open space)
            to_location: Destination location
            
        Returns:
            List of TaskResults for pick and place operations
        """
        results = []
        
        # Pick operation
        pick_result = self.pick_object(object_name, from_location)
        results.append(pick_result)
        
        if not pick_result.success:
            return results
        
        # Place operation
        place_result = self.place_object(object_name, to_location)
        results.append(place_result)
        
        return results
    
    def execute_task_sequence(self, sequence: TaskSequence) -> List[TaskResult]:
        """
        Execute a sequence of tasks.
        
        Args:
            sequence: TaskSequence to execute
            
        Returns:
            List of TaskResults for each step
        """
        results = []
        
        while not sequence.is_complete():
            step = sequence.steps[sequence.current_step]
            step_type = step.get('type')
            
            if step_type == 'pick':
                result = self.pick_object(
                    step['object'],
                    step.get('location')
                )
            elif step_type == 'place':
                result = self.place_object(
                    step.get('object'),
                    step['location']
                )
            elif step_type == 'navigate':
                if step['target_type'] == 'object':
                    result = self.nav_interface.navigate_to_object(step['target'])
                else:
                    result = self.nav_interface.navigate_to_pose(step['target'])
                
                result = TaskResult(
                    success=result.success,
                    task_name=f"navigate_to_{step['target']}",
                    navigation_result=result,
                    execution_time=result.execution_time
                )
            else:
                result = TaskResult(
                    success=False,
                    task_name=f"unknown_{step_type}",
                    error_message=f"Unknown task type: {step_type}"
                )
            
            results.append(result)
            sequence.results.append(result)
            sequence.current_step += 1
            
            # Stop on failure if recovery is disabled
            if not result.success and not self.config.enable_recovery:
                break
        
        return results
    
    def get_task_summary(self) -> Dict[str, Any]:
        """
        Get summary of all executed tasks.
        
        Returns:
            Dictionary with task statistics
        """
        total_tasks = len(self._task_history)
        successful_tasks = sum(1 for r in self._task_history if r.success)
        
        total_time = sum(r.execution_time for r in self._task_history)
        
        # Get visualizer summary
        vis_summary = self.sim_visualizer.create_task_summary() if self.sim_visualizer else {}
        
        return {
            'total_tasks': total_tasks,
            'successful_tasks': successful_tasks,
            'failed_tasks': total_tasks - successful_tasks,
            'success_rate': successful_tasks / total_tasks if total_tasks > 0 else 0,
            'total_execution_time': total_time,
            'visualization_summary': vis_summary,
            'currently_holding': self._held_object
        }
    
    def reset(self):
        """Reset the interface state"""
        self._held_object = None
        self._current_task = None
        self._task_history = []
        
        if self.sim_visualizer:
            self.sim_visualizer.clear_visualization()
    
    def shutdown(self):
        """Shutdown the interface and cleanup resources"""
        if self.sim_visualizer:
            self.sim_visualizer.stop_visualization()
        
        self.reset()


def create_mobile_manipulation_interface(
    client: Client,
    robot,
    visualizer: Visualizer,
    **kwargs
) -> MobileManipulationInterface:
    """
    Factory function to create a mobile manipulation interface.
    
    Args:
        client: PyBullet client instance
        robot: BestMan robot instance
        visualizer: Base visualizer instance
        **kwargs: Additional configuration parameters
        
    Returns:
        Configured MobileManipulationInterface instance
    """
    config = TaskConfig(**kwargs)
    return MobileManipulationInterface(client, robot, visualizer, config)
