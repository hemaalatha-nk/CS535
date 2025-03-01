# Task 1 - Getting Started with Isaac Sim

## Introduction
This guide walks you through opening and verifying the **Hello World** example in Isaac Sim, as well as accessing its source code for editing in Visual Studio Code. Additionally, it explains how the logic and UI elements of the application are structured and provides steps for reloading the example....

## Steps to Open the Hello World Example

### 1. Open the Hello World Example
- Go to the top menu bar and click **Isaac Examples > Hello World**.

### 2. Verify the Example Window
- Ensure that the **Hello World** example extension window is visible in the workspace.

### 3. Open the Source Code
- Click the **Open Source Code** button to launch the source code for editing in **Visual Studio Code**.

---

## Application Structure

- The **hello_world.py** script contains the core logic of the application.
- The **hello_world_extension.py** script handles the UI elements and links them to the logic in `hello_world.py`.

---

## Steps to Load and Reload the World

### 1. Load the World
- Click the **LOAD** button to load the World.

### 2. Create a New Stage
- Go to **File > New From Stage Template > Empty** to create a new stage.
- Click **Don’t Save** when prompted to save the current stage.

### 3. Reload the World
- Click the **LOAD** button again to load the World.

### 4. Modify and Hot-Reload the Script
- Open **hello_world.py** and press **Ctrl+S** to use the hot-reload feature.
- The example menu will disappear from the workspace (because it was restarted).

### 5. Reload the Example Menu
- Open the example menu again and click the **LOAD** button.

---

## Adding a Cube to the Scene

We can use the **Python API** to add a cube as a rigid body to the scene in **hello_world.py**.

### Code (`hello_world.py`):
```python
from omni.isaac.examples.base_sample import BaseSample
import numpy as np
# Can be used to create a new cube or to point to an already existing cube in stage.
from omni.isaac.core.objects import DynamicCuboid

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        fancy_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",  # The prim path of the cube in the USD stage
                name="fancy_cube",  # The unique name used to retrieve the object from the scene later on
                position=np.array([0, 0, 1.0]),  # Using the current stage units (meters by default).
                scale=np.array([0.5015, 0.5015, 0.5015]),  # Most arguments accept numpy arrays.
                color=np.array([0, 0, 1.0]),  # RGB channels, range from 0-1 (Blue cube).
            ))
        return
```
## Running and Simulating the Scene

### 1. Save and Hot-Reload
- Press **`Ctrl+S`** to save the code and hot-reload Omniverse Isaac Sim.

### 2. Open the Example Menu
- Open the **Hello World** example menu again.

### 3. Reload the Scene
- Click **File > New From Stage Template > Empty**, then click the **LOAD** button.  
- You need to perform this action **if you change anything in `setup_scene`**.  
- Otherwise, you only need to press the **LOAD** button.

### 4. Start the Simulation
- Press the **PLAY** button to start simulating the dynamic cube and see it fall.

# Continuously Inspecting Object Properties During Simulation

## Overview
To monitor the **world pose** and **velocity** of the cube during simulation at every physics step, we can use a **physics callback**. This allows us to print relevant object properties asynchronously, ensuring that the cube's position, orientation, and velocity are continuously logged during execution.

## Understanding the Workflow
- Since **Isaac Sim** runs **asynchronously**, we cannot directly control when physics steps occur.
- Instead, we can add a **physics callback**, ensuring that specific actions (such as printing cube properties) occur before each physics step.

---

## Implementing the Physics Callback

Modify the `hello_world.py` file to include a physics callback:

### `hello_world.py`
```python
from omni.isaac.examples.base_sample import BaseSample
import numpy as np
from omni.isaac.core.objects import DynamicCuboid

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        fancy_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0, 0, 1.0]),
                scale=np.array([0.5015, 0.5015, 0.5015]),
                color=np.array([0, 0, 1.0]),  # Blue cube
            ))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._cube = self._world.scene.get_object("fancy_cube")
        self._world.add_physics_callback("sim_step", callback_fn=self.print_cube_info)  
        # Callback names must be unique
        return

    # Define the physics callback, executed before each physics step.
    def print_cube_info(self, step_size):
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        # Log cube properties to the terminal
        print("Cube position is: " + str(position))
        print("Cube's orientation is: " + str(orientation))
        print("Cube's linear velocity is: " + str(linear_velocity))
```
[To execute the changes Go to Running and Simulating the Scene](#running-and-simulating-the-scene)
