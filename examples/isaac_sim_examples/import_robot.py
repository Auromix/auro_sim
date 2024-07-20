# import_robot.py
from omni.isaac.core.utils.nucleus import get_assets_root_path

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
from omni.isaac.core.utils.stage import add_reference_to_stage

add_reference_to_stage(usd_path=robot_asset_path, prim_path="/World/MyRobot")

# Wrap the root of robot prim under a Robot(Articulation) class
# to use high level api to set/ get attributes as well as initializing physics handles needed
from omni.isaac.core.robots import Robot
from omni.isaac.core.world import World

world = World()
robot = Robot(prim_path="/World/MyRobot", name="my_robot")
# Add robot to the scene
world.scene.add(robot)
print(robot)

# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()
