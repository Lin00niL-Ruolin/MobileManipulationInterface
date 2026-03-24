# Mobile Manipulation Interface

A comprehensive interface library for mobile manipulation tasks in the BestMan robotics simulation platform.

## Overview

This package provides unified, high-level interfaces for:
- **Navigation**: Robot base movement with path planning (A*, RRT, PRM)
- **Object Retrieval**: Grasping and picking operations
- **Object Placement**: Placing and positioning operations
- **Simulation Visualization**: Real-time process visualization and monitoring

## Installation

The package is located in the `MobileManipulationInterface` directory within the BestMan project.

No additional installation is required beyond the standard BestMan setup.

## Quick Start

```python
from MobileManipulationInterface import (
    create_mobile_manipulation_interface,
    NavigationAlgorithm,
    GraspType,
    PlacementType
)

# Initialize interface
mm_interface = create_mobile_manipulation_interface(
    client=client,
    robot=robot,
    visualizer=visualizer,
    navigation_algorithm=NavigationAlgorithm.A_STAR,
    grasp_type=GraspType.VACUUM,
    placement_type=PlacementType.ON_SURFACE
)

# Pick up an object
result = mm_interface.pick_object(
    object_name="bowl",
    standing_offset=[-0.5, 0.0, 0.0]
)

# Place the object
result = mm_interface.place_object(
    object_name="bowl",
    to_location="table"
)
```

## Module Structure

### 1. Navigation Interface (`navigation_interface.py`)

Provides robot navigation capabilities:
- Environment mapping using SLAM
- Path planning with multiple algorithms (A*, RRT, PRM)
- Robot base navigation execution
- Path visualization

**Key Classes:**
- `NavigationInterface`: Main navigation interface
- `NavigationConfig`: Configuration for navigation
- `NavigationResult`: Result of navigation execution

**Example:**
```python
from MobileManipulationInterface import NavigationInterface, NavigationConfig

nav_config = NavigationConfig(
    algorithm=NavigationAlgorithm.A_STAR,
    resolution=0.05
)

nav_interface = NavigationInterface(client, robot, visualizer, nav_config)
nav_interface.initialize_mapping()
nav_interface.create_planner()

# Navigate to object
result = nav_interface.navigate_to_object(
    "target_object",
    standing_offset=[-0.5, 0.0, 0.0]
)
```

### 2. Object Retrieval Interface (`object_retrieval_interface.py`)

Handles object grasping and retrieval:
- Object detection and pose estimation
- Grasp pose calculation
- Motion planning for reaching
- Gripper control

**Key Classes:**
- `ObjectRetrievalInterface`: Main retrieval interface
- `GraspConfig`: Configuration for grasping
- `RetrievalResult`: Result of retrieval execution
- `GraspType`: Supported grasp types (VACUUM, PARALLEL_JAW, etc.)

**Example:**
```python
from MobileManipulationInterface import ObjectRetrievalInterface, GraspConfig, GraspType

grasp_config = GraspConfig(
    grasp_type=GraspType.VACUUM,
    pre_grasp_offset=0.15
)

retrieval_interface = ObjectRetrievalInterface(
    client, robot, visualizer, grasp_config=grasp_config
)
retrieval_interface.initialize_planner()

# Execute grasp
result = retrieval_interface.execute_grasp("target_object")
```

### 3. Object Placement Interface (`object_placement_interface.py`)

Manages object placement operations:
- Target location detection
- Placement pose calculation
- Motion planning for placement
- Post-placement verification

**Key Classes:**
- `ObjectPlacementInterface`: Main placement interface
- `PlacementConfig`: Configuration for placement
- `PlacementResult`: Result of placement execution
- `PlacementType`: Supported placement types

**Example:**
```python
from MobileManipulationInterface import ObjectPlacementInterface, PlacementConfig

placement_config = PlacementConfig(
    placement_type=PlacementType.ON_SURFACE,
    place_height_offset=0.02
)

placement_interface = ObjectPlacementInterface(
    client, robot, visualizer, placement_config=placement_config
)
placement_interface.initialize_planner()

# Execute placement
result = placement_interface.execute_placement(
    "object_name",
    "target_location"
)
```

### 4. Simulation Visualizer (`simulation_visualizer.py`)

Provides comprehensive visualization:
- Real-time robot state tracking
- Trajectory trail rendering
- Task phase indicators
- Performance metrics display
- Screenshot capture

**Key Classes:**
- `SimulationVisualizer`: Main visualizer
- `VisualizationConfig`: Configuration for visualization
- `VisualizationMode`: Visualization modes (REALTIME, RECORDING, etc.)
- `TaskPhase`: Task execution phases

**Example:**
```python
from MobileManipulationInterface import SimulationVisualizer, VisualizationConfig

vis_config = VisualizationConfig(
    mode=VisualizationMode.REALTIME,
    enable_trajectory_trail=True,
    enable_metrics_panel=True
)

sim_visualizer = SimulationVisualizer(client, visualizer, vis_config)
sim_visualizer.start_visualization()

# Update state
sim_visualizer.update_state(
    robot,
    TaskPhase.NAVIGATION,
    "Navigating to target"
)

# Visualize path
sim_visualizer.visualize_navigation_path(path)

# Highlight object
sim_visualizer.highlight_object("target_object")
```

### 5. Integrated Interface (`integrated_interface.py`)

High-level unified interface combining all components:
- Complete pick-and-place operations
- Task sequence execution
- Automatic state management
- Comprehensive error handling

**Key Classes:**
- `MobileManipulationInterface`: Main integrated interface
- `TaskConfig`: Configuration for tasks
- `TaskSequence`: Sequence of manipulation tasks
- `TaskResult`: Result of task execution

**Example:**
```python
from MobileManipulationInterface import (
    MobileManipulationInterface,
    TaskSequence
)

# Create interface
mm_interface = MobileManipulationInterface(
    client, robot, visualizer
)

# Execute pick and place
results = mm_interface.move_object(
    object_name="bowl",
    from_location="drawer",
    to_location="table"
)

# Or create a task sequence
sequence = TaskSequence(name="Organize Items")
sequence.add_pick_task("bowl1")
sequence.add_place_task("bowl1", "table")
sequence.add_pick_task("bowl2")
sequence.add_place_task("bowl2", "table")

results = mm_interface.execute_task_sequence(sequence)
```

## Configuration Options

### Navigation Configuration

```python
NavigationConfig(
    algorithm=NavigationAlgorithm.A_STAR,  # A_STAR, RRT, PRM
    resolution=0.05,                       # Grid resolution
    x_max=10.0,                           # Map bounds
    y_max=10.0,
    enable_plot=False                     # Show planning visualization
)
```

### Grasp Configuration

```python
GraspConfig(
    grasp_type=GraspType.VACUUM,          # VACUUM, PARALLEL_JAW, etc.
    approach=GraspApproach.TOP_DOWN,      # TOP_DOWN, SIDEWAYS, FRONT
    pre_grasp_offset=0.15,                # Approach distance
    grasp_offset=0.0,                     # Grasp position offset
    lift_height=0.2                       # Post-grasp lift height
)
```

### Placement Configuration

```python
PlacementConfig(
    placement_type=PlacementType.ON_SURFACE,  # ON_SURFACE, IN_CONTAINER, STACK
    precision=PlacementPrecision.MEDIUM,      # HIGH, MEDIUM, LOW
    place_height_offset=0.02,                 # Height above surface
    approach_height=0.15,                     # Approach height
    release_delay=0.5                         # Delay before release
)
```

### Visualization Configuration

```python
VisualizationConfig(
    mode=VisualizationMode.REALTIME,      # REALTIME, RECORDING, ANALYSIS
    enable_trajectory_trail=True,         # Show robot path
    enable_phase_indicator=True,          # Show current phase
    enable_metrics_panel=True,            # Show performance metrics
    trail_length=100,                     # Number of trail points
    update_frequency=30.0                 # Update rate (Hz)
)
```

## Examples

See `example_usage.py` for comprehensive examples:

1. **Basic Pick and Place**: Simple object manipulation
2. **Move from Drawer to Table**: Container retrieval operations
3. **Task Sequence**: Complex multi-step operations
4. **Visualization Features**: Demonstrates visualization capabilities

Run examples:
```bash
cd MobileManipulationInterface
python example_usage.py
```

## API Reference

### NavigationInterface

| Method | Description |
|--------|-------------|
| `initialize_mapping()` | Initialize SLAM mapping |
| `create_planner()` | Create path planner |
| `plan_path(goal_pose)` | Plan path to goal |
| `navigate_to_pose(pose)` | Navigate to pose |
| `navigate_to_object(name)` | Navigate to object |

### ObjectRetrievalInterface

| Method | Description |
|--------|-------------|
| `initialize_planner()` | Initialize motion planner |
| `detect_object(name)` | Detect object pose |
| `calculate_grasp_pose(name)` | Calculate grasp pose |
| `execute_grasp(name)` | Execute grasp operation |
| `retrieve_from_container()` | Retrieve from container |

### ObjectPlacementInterface

| Method | Description |
|--------|-------------|
| `initialize_planner()` | Initialize motion planner |
| `detect_placement_surface()` | Detect target surface |
| `execute_placement()` | Execute placement |
| `place_in_container()` | Place in container |
| `stack_object()` | Stack on another object |

### MobileManipulationInterface

| Method | Description |
|--------|-------------|
| `pick_object()` | Complete pick operation |
| `place_object()` | Complete place operation |
| `move_object()` | Move object between locations |
| `execute_task_sequence()` | Execute task sequence |
| `get_task_summary()` | Get execution summary |

## Error Handling

All interfaces return result objects with success status:

```python
result = mm_interface.pick_object("bowl")

if result.success:
    print(f"Success! Time: {result.execution_time}")
else:
    print(f"Failed: {result.error_message}")
```

## Visualization Features

### Real-time Monitoring
- Robot position and orientation tracking
- Joint value monitoring
- Task phase indicators

### Path Visualization
- Navigation path display
- Manipulation trajectory rendering
- Trajectory trails with color-coded phases

### Object Highlighting
- Target object highlighting
- Bounding box visualization
- Label display

### Metrics Display
- Execution time tracking
- Distance traveled
- Success/failure statistics

### Data Export
- Trajectory data export (JSON)
- Screenshot capture
- Task summary generation

## Integration with Dyna_hmrc_web

The interface can be integrated with the Dyna_hmrc_web multi-robot collaboration system:

```python
from Dyna_hmrc_web.dynahmrc_web.dynahmrc.core.agent import RobotAgent
from MobileManipulationInterface import MobileManipulationInterface

class BestManRobotAgent(RobotAgent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.mm_interface = None
    
    def initialize_manipulation(self, client, robot, visualizer):
        self.mm_interface = MobileManipulationInterface(
            client, robot, visualizer
        )
    
    def execute_action(self, action):
        if action['type'] == 'pick':
            return self.mm_interface.pick_object(action['object'])
        elif action['type'] == 'place':
            return self.mm_interface.place_object(
                action['object'],
                action['location']
            )
```

## Contributing

When contributing to this package:
1. Follow the existing code style and documentation standards
2. Add comprehensive docstrings to all public methods
3. Include type hints for function parameters and returns
4. Add examples for new features
5. Update this README with new functionality

## License

This package is part of the BestMan project and follows the same license terms.
