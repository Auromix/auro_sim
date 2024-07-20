# control_robot.py
import numpy as np
from isaacsim import SimulationApp

# See DEFAULT_LAUNCHER_CONFIG for available configuration
# https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.kit/docs/index.html
launch_config = {"headless": False}
# Launch the Toolkit
simulation_app = SimulationApp(launch_config)

# Locate any other import statement after this point
from omni.isaac.core.world import World
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

# Instantiate World class
world = World()
# Add a ground plane to stage and register it in the scene
world.scene.add_default_ground_plane()

# Get isaac sim assets folder root path
# It should be "omniverse://localhost/NVIDIA/Assets/Isaac/4.0"
assets_root_path = get_assets_root_path()
print(assets_root_path)
if assets_root_path is None:
    print("Could not find nucleus server with '/Isaac' folder")

# Get franka in isaac sim official assets folder
robot_asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
print(robot_asset_path)

# Add robot asset reference to stage
# This will create a new XFormPrim and point it to the usd file as a reference
add_reference_to_stage(usd_path=robot_asset_path, prim_path="/World/MyRobot")

# Wrap the root of robot prim under a Robot(Articulation) class
# to use high level api to set/ get attributes as well as initializing physics handles needed
robot = Robot(prim_path="/World/MyRobot", name="my_robot")
# Add robot to the scene
world.scene.add(robot)
print(robot)

# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()

# This is an implicit PD controller of the robot articulation
# Stiffness and damping (gains) can be set through the articulation view
articulation_controller = robot.get_articulation_controller()

# Get number of active joints in the robot
num_dof = robot.num_dof
# 7 arm joints + 2 gripper joints
print(f"Num of dof(active joints)={num_dof}")
# Get joint limits
joint_limits_lower = robot.dof_properties["lower"]
joint_limits_upper = robot.dof_properties["upper"]
print(f"Joint limits lower={joint_limits_lower} upper={joint_limits_upper}")

# Start simulation
all_steps = 10000
for i in range(all_steps):
    # Update joint positions every 500 steps
    if i % (all_steps / 20) == 0:
        # Generate radom joint positions within joint limits
        target_joint_positions = np.random.uniform(
            joint_limits_lower, joint_limits_upper
        )
        # Construct an action to apply to the articulation
        # joint_indices = None will apply the action to all joints
        target_action = ArticulationAction(
            joint_positions=target_joint_positions,
            joint_efforts=None,
            joint_velocities=None,
            joint_indices=None,
        )
        # Apply action
        articulation_controller.apply_action(target_action)
        print(f"Step {i}: target_joint_positions={target_joint_positions}")
    # Update world
    world.step(render=True)


# Close the running Toolkit
simulation_app.close()
