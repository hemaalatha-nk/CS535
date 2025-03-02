# Task 2: Import the URDF File

## Step 1: Download the Elephant Robotics ROS Repository

[Ensure you have downloaded the necessary repository for Elephant Robotics .](https://github.com/elephantrobotics/mycobot_ros)

## Step 2: Add Objects to Scene

### Import Robot URDF
1. Open **Isaac Utils > URDF Importer**.
2. Under import settings:
   - Set **Input File** to: Path_to/mycobot_ros/mycobot_description/urdf/myarm_300_pi/myarm_300_pi.urdf

   - Set **Output Directory** to **(same as source)**.
   - If not checked, enable **"Create Physics Scene"** in the **Input Options**.
3. Click **Import** to import the URDF.

### Copy the USD File
1. Navigate to the folder:
   ```
   Path_to/mycobot_ros/mycobot_description/urdf/myarm_300_pi/myarm_300_pi
   ```
2. Locate the `myarm_300_pi.usd` file.
3. Copy and paste the `myarm_300_pi.usd` file to the **Nucleus Files** directory.
4. Open the copied **USD** file.

### Step 3: Add Ground Plane and Lighting
1. Add a **ground plane**:
   - Go to **Create > Physics > Ground Plane**.
2. Ensure proper lighting:
   - If there is no light on the stage, add one from **Create > Light > Dome Light**.

Your URDF file is now imported and configured properly!

## Step 4: Add Python Scripts

1. Click on **“firefighter”** in the **Stage** view.
2. In the **Properties** panel, click **Add > Python Scripting**.
3. Add the script **manipulate_robot.py**.

## Step 5: Add and Configure `manipulation.py`

1. Save the following Python script as `manipulation.py`.
2. Attach this script to a **prim or xform** that has an **articulation root**.
3. Ensure the script is properly linked within the environment.

```python
# This file covers controlling the robot via target, scene control panel, and text box
# 7/19/23 - Ensure that this script is attached to a prim or xform that has an articulation root. 
# 8/15/23 - Target now changes color and returns to original when updated.

import logging
import math
from pathlib import Path
import time
import json
import numpy as np
import omni
import omni.kit.pipapi
import omni.usd.commands
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.motion_generation import (ArticulationKinematicsSolver,
                                          ArticulationMotionPolicy,
                                          LulaKinematicsSolver, RmpFlow)
from omni.kit.scripting import BehaviorScript
from pxr import Gf, Sdf, Usd

MOTION_GEN_ALGO = 'IK'
PATH_BASE = Path("/home/hning4/Documents/Omniverse_Files")

logger = logging.getLogger(__name__)
json_angles=[]

class RobotControl(BehaviorScript):
    def __init__(self, prim_path: Sdf.Path):
        super().__init__(prim_path)
        self.robot = None
        self.had_first_update = False
        self.ee_name = 'joint7'
        self.motion_gen_algo = LulaKinematicsSolver(
            robot_description_path=str(PATH_BASE) + '/robot_data/rd_myarm.yaml',
            urdf_path=str(PATH_BASE) +  '/robot_data/myarm_300_pi.urdf',
        )
        logger.warn('IK loaded')

    def on_first_update(self, current_time: float, delta_time: float):
        self.had_first_update = True
        self.robot = Articulation(str(self.prim_path))
        self.robot.initialize()
        self.motion_gen_solver = ArticulationKinematicsSolver(self.robot, self.motion_gen_algo, self.ee_name)

    def on_update(self, current_time: float, delta_time: float):
        if not self.had_first_update:
            self.on_first_update(current_time, delta_time)
        follow_target = self.stage.GetPrimAtPath('/World/Target')
        pos = get_world_translation(follow_target)
        rot = get_world_rotation(follow_target)
        target = (np.array(pos), np.array(rot))
        self.motion_gen_algo.set_robot_base_pose(*self.robot.get_world_pose())
        action, success = self.motion_gen_solver.compute_inverse_kinematics(target_position=target[0], target_orientation=target[1])
        if success:
            self.robot.apply_action(action)
            json_angles.append(action)

# Utility functions to get world transformation properties
def get_world_translation(prim: Usd.Prim) -> Gf.Vec3d:
    world_transform = omni.usd.get_world_transform_matrix(prim)
    return world_transform.ExtractTranslation()

def get_world_rotation(prim: Usd.Prim) -> Gf.Rotation:
    world_transform = omni.usd.get_world_transform_matrix(prim)
    return world_transform.ExtractRotation()
```

4. Attach the `manipulation.py` script to the robot's articulation root.
5. Ensure it is running within Omniverse for real-time robot control.

