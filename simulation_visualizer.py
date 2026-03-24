#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Simulation Visualizer Module

This module provides comprehensive visualization capabilities for the simulation
process, including real-time monitoring, trajectory visualization, state tracking,
and progress reporting.

Author: BestMan Development Team
Date: 2026-03-24
"""

import math
import time
from typing import List, Optional, Dict, Any, Tuple, Callable
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
import threading

import numpy as np
import pybullet as p

from Robotics_API import Pose


class VisualizationMode(Enum):
    """Visualization modes"""
    REALTIME = "realtime"
    RECORDING = "recording"
    ANALYSIS = "analysis"
    MINIMAL = "minimal"


class TaskPhase(Enum):
    """Task execution phases"""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    PLANNING = "planning"
    IDLE = "idle"


@dataclass
class VisualizationConfig:
    """Configuration for simulation visualization"""
    mode: VisualizationMode = VisualizationMode.REALTIME
    enable_trajectory_trail: bool = True
    enable_phase_indicator: bool = True
    enable_metrics_panel: bool = True
    enable_object_highlighting: bool = True
    trail_length: int = 100
    update_frequency: float = 30.0  # Hz
    screenshot_interval: float = 0.0  # 0 = disabled


@dataclass
class SimulationState:
    """Current state of the simulation"""
    timestamp: float
    robot_position: List[float]
    robot_orientation: List[float]
    joint_values: List[float]
    phase: TaskPhase
    current_action: str
    held_object: Optional[str] = None
    metrics: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TrajectorySegment:
    """A segment of robot trajectory"""
    positions: List[List[float]]
    timestamps: List[float]
    phase: TaskPhase
    color: List[float]


class SimulationVisualizer:
    """
    Comprehensive visualization component for simulation process.
    
    This class provides:
    - Real-time robot state visualization
    - Trajectory trail rendering
    - Task phase indicators
    - Performance metrics display
    - Object highlighting
    - Screenshot capture
    - State history tracking
    
    Attributes:
        client: PyBullet client instance
        visualizer: Base visualizer instance
        config: Visualization configuration
        state_history: History of simulation states
        trajectories: Recorded trajectory segments
    """
    
    def __init__(
        self,
        client,
        visualizer,
        config: Optional[VisualizationConfig] = None
    ):
        """
        Initialize the simulation visualizer.
        
        Args:
            client: PyBullet client instance
            visualizer: Base visualizer instance
            config: Visualization configuration
        """
        self.client = client
        self.visualizer = visualizer
        self.config = config or VisualizationConfig()
        
        self.client_id = client.get_client_id()
        self._state_history = deque(maxlen=1000)
        self._trajectories: List[TrajectorySegment] = []
        self._current_trajectory: Optional[TrajectorySegment] = None
        self._current_phase = TaskPhase.IDLE
        self._metrics: Dict[str, Any] = {}
        self._visualization_items: Dict[str, int] = {}
        
        # Phase colors
        self._phase_colors = {
            TaskPhase.NAVIGATION: [0.0, 1.0, 0.0],      # Green
            TaskPhase.MANIPULATION: [1.0, 0.0, 0.0],    # Red
            TaskPhase.PERCEPTION: [0.0, 0.0, 1.0],      # Blue
            TaskPhase.PLANNING: [1.0, 1.0, 0.0],        # Yellow
            TaskPhase.IDLE: [0.5, 0.5, 0.5]             # Gray
        }
        
        # Threading
        self._running = False
        self._update_thread: Optional[threading.Thread] = None
        
    def start_visualization(self):
        """Start the visualization update loop"""
        self._running = True
        if self.config.mode == VisualizationMode.REALTIME:
            self._update_thread = threading.Thread(target=self._update_loop)
            self._update_thread.daemon = True
            self._update_thread.start()
    
    def stop_visualization(self):
        """Stop the visualization update loop"""
        self._running = False
        if self._update_thread:
            self._update_thread.join(timeout=1.0)
    
    def update_state(
        self,
        robot,
        phase: TaskPhase,
        current_action: str = "",
        held_object: Optional[str] = None,
        additional_metrics: Optional[Dict] = None
    ):
        """
        Update and record current simulation state.
        
        Args:
            robot: Robot instance
            phase: Current task phase
            current_action: Description of current action
            held_object: Name of held object (if any)
            additional_metrics: Additional metrics to record
        """
        timestamp = time.time()
        base_pose = robot.sim_get_current_base_pose()
        
        state = SimulationState(
            timestamp=timestamp,
            robot_position=base_pose.get_position(),
            robot_orientation=base_pose.get_orientation(),
            joint_values=robot.sim_get_current_joint_values(),
            phase=phase,
            current_action=current_action,
            held_object=held_object,
            metrics=additional_metrics or {}
        )
        
        self._state_history.append(state)
        
        # Update trajectory
        if phase != self._current_phase:
            self._finalize_current_trajectory()
            self._start_new_trajectory(phase)
        
        if self._current_trajectory:
            self._current_trajectory.positions.append(state.robot_position)
            self._current_trajectory.timestamps.append(timestamp)
        
        self._current_phase = phase
        
        # Update visualizations
        if self.config.enable_trajectory_trail:
            self._update_trajectory_visualization()
        
        if self.config.enable_phase_indicator:
            self._update_phase_indicator(state)
    
    def visualize_navigation_path(
        self,
        path: List[List[float]],
        color: Optional[List[float]] = None,
        line_width: float = 2.0
    ):
        """
        Visualize a navigation path.
        
        Args:
            path: List of waypoints [[x, y], ...]
            color: Line color [r, g, b]
            line_width: Width of the path line
        """
        color = color or [0.0, 1.0, 0.0]
        
        # Draw path as connected line segments
        for i in range(len(path) - 1):
            start_point = [path[i][0], path[i][1], 0.05]
            end_point = [path[i+1][0], path[i+1][1], 0.05]
            
            line_id = p.addUserDebugLine(
                start_point,
                end_point,
                lineColorRGB=color,
                lineWidth=line_width,
                lifeTime=0,
                physicsClientId=self.client_id
            )
            
            item_key = f"nav_path_{i}"
            if item_key in self._visualization_items:
                p.removeUserDebugItem(
                    self._visualization_items[item_key],
                    physicsClientId=self.client_id
                )
            self._visualization_items[item_key] = line_id
    
    def visualize_manipulation_trajectory(
        self,
        trajectory: List[List[float]],
        robot,
        color: Optional[List[float]] = None
    ):
        """
        Visualize a manipulation trajectory in Cartesian space.
        
        Args:
            trajectory: Joint trajectory
            robot: Robot instance
            color: Visualization color
        """
        color = color or [1.0, 0.0, 0.0]
        
        cartesian_points = []
        for joint_values in trajectory:
            # Calculate end-effector position for each configuration
            # This is a simplified visualization
            point = self._calculate_eef_position(robot, joint_values)
            cartesian_points.append(point)
        
        # Draw trajectory
        for i in range(len(cartesian_points) - 1):
            line_id = p.addUserDebugLine(
                cartesian_points[i],
                cartesian_points[i+1],
                lineColorRGB=color,
                lineWidth=2.0,
                lifeTime=0,
                physicsClientId=self.client_id
            )
            
            item_key = f"manip_traj_{i}"
            if item_key in self._visualization_items:
                p.removeUserDebugItem(
                    self._visualization_items[item_key],
                    physicsClientId=self.client_id
                )
            self._visualization_items[item_key] = line_id
    
    def highlight_object(
        self,
        object_name: str,
        color: List[float] = [1.0, 1.0, 0.0],
        highlight_duration: float = 5.0
    ):
        """
        Highlight an object in the scene.
        
        Args:
            object_name: Name of object to highlight
            color: Highlight color
            highlight_duration: Duration in seconds
        """
        try:
            object_id = self.client.resolve_object_id(object_name)
            aabb = self.client.get_bounding_box(object_id)
            
            # Draw bounding box
            self._draw_aabb_outline(aabb, color, highlight_duration)
            
            # Draw label
            center = [
                (aabb[0][0] + aabb[1][0]) / 2,
                (aabb[0][1] + aabb[1][1]) / 2,
                aabb[1][2] + 0.1
            ]
            
            label_id = p.addUserDebugText(
                object_name,
                center,
                textColorRGB=color,
                textSize=1.5,
                lifeTime=highlight_duration,
                physicsClientId=self.client_id
            )
            
            item_key = f"highlight_{object_name}"
            if item_key in self._visualization_items:
                p.removeUserDebugItem(
                    self._visualization_items[item_key],
                    physicsClientId=self.client_id
                )
            self._visualization_items[item_key] = label_id
            
        except Exception as e:
            print(f"[SimulationVisualizer] Failed to highlight object: {e}")
    
    def show_metrics_panel(self, metrics: Dict[str, Any]):
        """
        Display metrics panel in the simulation view.
        
        Args:
            metrics: Dictionary of metrics to display
        """
        self._metrics.update(metrics)
        
        # Format metrics text
        text_lines = ["=== Simulation Metrics ==="]
        for key, value in self._metrics.items():
            if isinstance(value, float):
                text_lines.append(f"{key}: {value:.3f}")
            else:
                text_lines.append(f"{key}: {value}")
        
        display_text = "\n".join(text_lines)
        
        # Display in top-left corner
        text_id = p.addUserDebugText(
            display_text,
            [0.1, 0.1, 1.0],  # Position in world coordinates
            textColorRGB=[1.0, 1.0, 1.0],
            textSize=1.0,
            lifeTime=0,
            physicsClientId=self.client_id
        )
        
        if "metrics_panel" in self._visualization_items:
            p.removeUserDebugItem(
                self._visualization_items["metrics_panel"],
                physicsClientId=self.client_id
            )
        self._visualization_items["metrics_panel"] = text_id
    
    def capture_screenshot(self, filename: Optional[str] = None) -> str:
        """
        Capture a screenshot of the current simulation view.
        
        Args:
            filename: Output filename (auto-generated if None)
            
        Returns:
            Path to saved screenshot
        """
        from datetime import datetime
        import cv2
        
        # Get camera image
        width, height, viewMatrix, projectionMatrix, _, _, _, _, _, _, _, _ = \
            p.getDebugVisualizerCamera(physicsClientId=self.client_id)
        
        _, _, rgb, _, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=viewMatrix,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            physicsClientId=self.client_id
        )
        
        # Generate filename
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"simulation_screenshot_{timestamp}.png"
        
        # Save image
        rgb_img = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        cv2.imwrite(filename, rgb_img)
        
        return filename
    
    def create_task_summary(self) -> Dict[str, Any]:
        """
        Create a summary of the simulation task.
        
        Returns:
            Dictionary containing task summary statistics
        """
        if not self._state_history:
            return {}
        
        # Calculate statistics
        total_time = self._state_history[-1].timestamp - self._state_history[0].timestamp
        
        phase_times = {}
        current_phase_start = self._state_history[0].timestamp
        current_phase = self._state_history[0].phase
        
        for i in range(1, len(self._state_history)):
            if self._state_history[i].phase != current_phase:
                phase_duration = self._state_history[i].timestamp - current_phase_start
                phase_times[current_phase.value] = phase_times.get(current_phase.value, 0) + phase_duration
                current_phase = self._state_history[i].phase
                current_phase_start = self._state_history[i].timestamp
        
        # Add final phase
        phase_duration = self._state_history[-1].timestamp - current_phase_start
        phase_times[current_phase.value] = phase_times.get(current_phase.value, 0) + phase_duration
        
        # Calculate distance traveled
        total_distance = 0.0
        for i in range(1, len(self._state_history)):
            pos1 = self._state_history[i-1].robot_position
            pos2 = self._state_history[i].robot_position
            distance = math.sqrt(
                (pos2[0] - pos1[0])**2 +
                (pos2[1] - pos1[1])**2 +
                (pos2[2] - pos1[2])**2
            )
            total_distance += distance
        
        return {
            "total_time": total_time,
            "phase_times": phase_times,
            "total_distance": total_distance,
            "num_states_recorded": len(self._state_history),
            "num_trajectories": len(self._trajectories)
        }
    
    def clear_visualization(self):
        """Clear all visualization items"""
        for item_id in self._visualization_items.values():
            p.removeUserDebugItem(item_id, physicsClientId=self.client_id)
        self._visualization_items.clear()
        
        p.removeAllUserDebugItems(physicsClientId=self.client_id)
    
    def export_trajectory_data(self, filename: str):
        """
        Export trajectory data to file.
        
        Args:
            filename: Output file path
        """
        import json
        
        data = {
            "trajectories": [
                {
                    "phase": traj.phase.value,
                    "positions": traj.positions,
                    "timestamps": traj.timestamps,
                    "color": traj.color
                }
                for traj in self._trajectories
            ],
            "states": [
                {
                    "timestamp": state.timestamp,
                    "position": state.robot_position,
                    "orientation": state.robot_orientation,
                    "phase": state.phase.value,
                    "action": state.current_action
                }
                for state in self._state_history
            ]
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
    
    def _update_loop(self):
        """Background update loop for real-time visualization"""
        last_screenshot = time.time()
        
        while self._running:
            # Update visualizations
            if self.config.enable_trajectory_trail:
                self._update_trajectory_visualization()
            
            # Capture screenshots at interval
            if self.config.screenshot_interval > 0:
                current_time = time.time()
                if current_time - last_screenshot >= self.config.screenshot_interval:
                    self.capture_screenshot()
                    last_screenshot = current_time
            
            time.sleep(1.0 / self.config.update_frequency)
    
    def _update_trajectory_visualization(self):
        """Update the trajectory trail visualization"""
        if len(self._state_history) < 2:
            return
        
        # Get recent positions
        recent_states = list(self._state_history)[-self.config.trail_length:]
        
        # Draw trajectory segments colored by phase
        for i in range(len(recent_states) - 1):
            state1 = recent_states[i]
            state2 = recent_states[i + 1]
            
            color = self._phase_colors.get(state2.phase, [0.5, 0.5, 0.5])
            
            p.addUserDebugLine(
                state1.robot_position,
                state2.robot_position,
                lineColorRGB=color,
                lineWidth=2.0,
                lifeTime=1.0 / self.config.update_frequency,
                physicsClientId=self.client_id
            )
    
    def _update_phase_indicator(self, state: SimulationState):
        """Update the phase indicator visualization"""
        color = self._phase_colors.get(state.phase, [0.5, 0.5, 0.5])
        
        # Draw phase indicator near robot
        indicator_pos = [
            state.robot_position[0],
            state.robot_position[1],
            state.robot_position[2] + 0.5
        ]
        
        text = f"Phase: {state.phase.value}"
        if state.current_action:
            text += f" | Action: {state.current_action}"
        
        text_id = p.addUserDebugText(
            text,
            indicator_pos,
            textColorRGB=color,
            textSize=1.0,
            lifeTime=0.5,
            physicsClientId=self.client_id
        )
        
        if "phase_indicator" in self._visualization_items:
            p.removeUserDebugItem(
                self._visualization_items["phase_indicator"],
                physicsClientId=self.client_id
            )
        self._visualization_items["phase_indicator"] = text_id
    
    def _start_new_trajectory(self, phase: TaskPhase):
        """Start recording a new trajectory segment"""
        self._current_trajectory = TrajectorySegment(
            positions=[],
            timestamps=[],
            phase=phase,
            color=self._phase_colors.get(phase, [0.5, 0.5, 0.5])
        )
    
    def _finalize_current_trajectory(self):
        """Finalize the current trajectory segment"""
        if self._current_trajectory and len(self._current_trajectory.positions) > 1:
            self._trajectories.append(self._current_trajectory)
        self._current_trajectory = None
    
    def _calculate_eef_position(self, robot, joint_values: List[float]) -> List[float]:
        """Calculate end-effector position from joint values"""
        # Simplified calculation - in practice, use forward kinematics
        # This is a placeholder for visualization purposes
        base_pose = robot.sim_get_current_base_pose()
        base_pos = base_pose.get_position()
        
        # Approximate EEF position based on arm extension
        # This is a rough approximation for visualization
        extension = sum(joint_values) / len(joint_values) if joint_values else 0
        
        return [
            base_pos[0] + 0.3 * math.cos(extension),
            base_pos[1] + 0.3 * math.sin(extension),
            base_pos[2] + 0.5
        ]
    
    def _draw_aabb_outline(
        self,
        aabb: Tuple[List[float], List[float]],
        color: List[float],
        duration: float
    ):
        """Draw outline of axis-aligned bounding box"""
        min_pt, max_pt = aabb
        
        # Define corners
        corners = [
            [min_pt[0], min_pt[1], min_pt[2]],
            [max_pt[0], min_pt[1], min_pt[2]],
            [max_pt[0], max_pt[1], min_pt[2]],
            [min_pt[0], max_pt[1], min_pt[2]],
            [min_pt[0], min_pt[1], max_pt[2]],
            [max_pt[0], min_pt[1], max_pt[2]],
            [max_pt[0], max_pt[1], max_pt[2]],
            [min_pt[0], max_pt[1], max_pt[2]],
        ]
        
        # Define edges
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
            (0, 4), (1, 5), (2, 6), (3, 7),  # Vertical edges
        ]
        
        for i, (start_idx, end_idx) in enumerate(edges):
            p.addUserDebugLine(
                corners[start_idx],
                corners[end_idx],
                lineColorRGB=color,
                lineWidth=2.0,
                lifeTime=duration,
                physicsClientId=self.client_id
            )


def create_simulation_visualizer(
    client,
    visualizer,
    mode: VisualizationMode = VisualizationMode.REALTIME,
    **kwargs
) -> SimulationVisualizer:
    """
    Factory function to create a simulation visualizer.
    
    Args:
        client: PyBullet client instance
        visualizer: Base visualizer instance
        mode: Visualization mode
        **kwargs: Additional configuration parameters
        
    Returns:
        Configured SimulationVisualizer instance
    """
    config = VisualizationConfig(mode=mode, **kwargs)
    sim_visualizer = SimulationVisualizer(client, visualizer, config)
    return sim_visualizer
