# get_object_info.py
from omni.isaac.core import World

# Instantiate World class
world = World()
# Get the DynamicCuboid instance from the scene
cube = world.scene.get_object("target_cuboid")
print(cube)
# Get the world pose of the object
position, orientation = cube.get_world_pose()
print(f"Position(x,y,z)={position}")
print(f"Orientation(qw,qx,qy,qz)={orientation}")
# Get the linear and angular velocities of the object
# DynamicCuboid is subclass of RigidPrim, which has the dynamic related methods
linear_velocity = cube.get_linear_velocity()
angular_velocity = cube.get_angular_velocity()
mass = cube.get_mass()

print(f"Linear velocity={linear_velocity}")
print(f"Angular velocity={angular_velocity}")
print(f"Mass={mass}")
