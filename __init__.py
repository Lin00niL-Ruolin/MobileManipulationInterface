#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Mobile Manipulation Interface Package

This package provides unified interfaces for mobile manipulation tasks including:
- Navigation interface for robot base movement
- Object retrieval interface for grasping operations
- Object placement interface for placing objects
- Simulation visualizer for process visualization

Author: BestMan Development Team
Date: 2026-03-24
"""

from .navigation_interface import (
    NavigationInterface,
    NavigationConfig,
    NavigationResult,
    NavigationAlgorithm,
    create_navigation_interface
)

from .object_retrieval_interface import (
    ObjectRetrievalInterface,
    GraspConfig,
    RetrievalResult,
    GraspType,
    GraspApproach,
    create_retrieval_interface
)

from .object_placement_interface import (
    ObjectPlacementInterface,
    PlacementConfig,
    PlacementResult,
    PlacementTarget,
    PlacementType,
    PlacementPrecision,
    create_placement_interface
)

from .simulation_visualizer import (
    SimulationVisualizer,
    VisualizationConfig,
    SimulationState,
    TrajectorySegment,
    VisualizationMode,
    TaskPhase,
    create_simulation_visualizer
)

from .integrated_interface import (
    MobileManipulationInterface,
    TaskConfig,
    TaskResult,
    TaskSequence,
    create_mobile_manipulation_interface
)

__all__ = [
    # Navigation
    'NavigationInterface',
    'NavigationConfig',
    'NavigationResult',
    'NavigationAlgorithm',
    'create_navigation_interface',
    
    # Object Retrieval
    'ObjectRetrievalInterface',
    'GraspConfig',
    'RetrievalResult',
    'GraspType',
    'GraspApproach',
    'create_retrieval_interface',
    
    # Object Placement
    'ObjectPlacementInterface',
    'PlacementConfig',
    'PlacementResult',
    'PlacementTarget',
    'PlacementType',
    'PlacementPrecision',
    'create_placement_interface',
    
    # Visualization
    'SimulationVisualizer',
    'VisualizationConfig',
    'SimulationState',
    'TrajectorySegment',
    'VisualizationMode',
    'TaskPhase',
    'create_simulation_visualizer',
    
    # Integrated Interface
    'MobileManipulationInterface',
    'TaskConfig',
    'TaskResult',
    'TaskSequence',
    'create_mobile_manipulation_interface',
]

__version__ = '1.0.0'
__author__ = 'BestMan Development Team'
