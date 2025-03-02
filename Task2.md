# Task 2: Adding a Manipulator Robot


This task introduces a robot to the simulation, a **Franka Panda** manipulator. It describes how to add the robot to the scene and use its `PickAndPlaceController` class. After this tutorial, you will have more experience using different robot and controller classes in **Omniverse Isaac Sim**.

---

## Getting Started

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

## Creating the Scene
Begin by adding a **Franka Panda** robot from the `omni.isaac.franka` extension, as well as a cube for the Franka to pick up.

```python
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.franka import Franka

# Create a Franka Panda robot
franka = Franka(prim_path="/World/Franka", name="franka_robot")

# Add a cube to the scene for the Franka to pick up
cube_prim = stage_utils.add_reference_to_stage(usd_path="omniverse://localhost/NVIDIA/Assets/Props/Blocks/Cube.usd", prim_path="/World/Cube")

```python
from omni.isaac.examples.base_sample import BaseSample
# This extension has franka related tasks and controllers as well
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid
import numpy as np


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        # Robot specific class that provides extra functionalities
        # such as having gripper and end_effector instances.
        franka = world.scene.add(
            Franka(prim_path="/World/Fancy_Franka", name="fancy_franka")
        )

        # Add a cube for Franka to pick up
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0.3, 0.3, 0.3]),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),  # Blue color
            )
        )
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

## Use `PickPlaceController` to perform a **pick-and-place task**.

```python
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.tasks import BaseTask
import numpy as np

class FrankaPlaying(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])  # Target placement position
        self._task_achieved = False

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        # Adding the cube for pick-and-place
        self._cube = scene.add(DynamicCuboid(
            prim_path="/World/random_cube",
            name="fancy_cube",
            position=np.array([0.3, 0.3, 0.3]),  # Initial position of the cube
            scale=np.array([0.0515, 0.0515, 0.0515]),
            color=np.array([0, 0, 1.0])  # Blue color
        ))

        # Adding the Franka Panda manipulator
        self._franka = scene.add(Franka(
            prim_path="/World/Fancy_Franka",
            name="fancy_franka"
        ))

    def get_observations(self):
        """ Retrieve task-relevant observations. """
        cube_position, _ = self._cube.get_world_pose()
        current_joint_positions = self._franka.get_joint_positions()
        return {
            self._franka.name: {"joint_positions": current_joint_positions},
            self._cube.name: {"position": cube_position, "goal_position": self._goal_position}
        }

    def pre_step(self, control_index, simulation_time):
        """ Change cube color once placed correctly. """
        cube_position, _ = self._cube.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(self._goal_position - cube_position)) < 0.02:
            self._cube.get_applied_visual_material().set_color(color=np.array([0, 1.0, 0]))  # Green
            self._task_achieved = True

    def post_reset(self):
        """ Reset cube color and gripper state on simulation restart. """
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        self._cube.get_applied_visual_material().set_color(color=np.array([0, 0, 1.0]))  # Blue
        self._task_achieved = False


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self):
        world = self.get_world()
        world.add_task(FrankaPlaying(name="my_first_task"))  # Add task to world

    async def setup_post_load(self):
        """ Initialize Pick-and-Place Controller after loading. """
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")

        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )

        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()

    async def setup_post_reset(self):
        """ Reset controller after simulation reset. """
        self._controller.reset()
        await self._world.play_async()

    def physics_step(self, step_size):
        """ Execute pick-and-place logic in each simulation step. """
        current_observations = self._world.get_observations()
        actions = self._controller.forward(
            picking_position=current_observations["fancy_cube"]["position"],
            placing_position=current_observations["fancy_cube"]["goal_position"],
            current_joint_positions=current_observations["fancy_franka"]["joint_positions"],
        )
        self._franka.apply_action(actions)

        if self._controller.is_done():
            self._world.pause()

```
[To execute the changes Go to Running and Simulating the Scene](#running-and-simulating-the-scene)
