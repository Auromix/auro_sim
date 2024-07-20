# pick_and_place.py
import numpy as np
from isaacsim import SimulationApp

# See DEFAULT_LAUNCHER_CONFIG for available configuration
# https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.kit/docs/index.html
launch_config = {
    "headless": False,
    "width": "1920",
    "height": "1080",
}
# Launch the Toolkit
simulation_app = SimulationApp(launch_config)

# Locate any other import statement after this point
from omni.isaac.core.world import World
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
from isaac_sim_motion_generator.curobo_motion_generator import CuroboMotionGenerator

# Instantiate World class
world = World()
# Add a ground plane to stage and register it in the scene
ground_plane = world.scene.add_default_ground_plane()

# Add target cuboid to the stage
target_cuboid = DynamicCuboid(
    prim_path="/World/Target",  # Path in the stage
    name="target_cuboid",  # Nickname in the scene
    position=np.array(
        [0.36303193, 0.43660322, 0.1]
    ),  # Initial position as an array [x, y, z]
    orientation=np.array(
        [1, 0, 0, 0]
    ),  # Initial orientation as an array [qw, qx, qy, qz]
    color=np.array([1, 0, 0]),  # Normalized RGB color (red)
    size=0.05,  # Size of the cuboid, size*scale = [length, width, height]
    scale=np.array([1, 1, 1]),  # Scale factors for the cuboid
    mass=0.01,  # Mass of the cuboid in kilograms
)

# Register the cuboid in the scene
world.scene.add(target_cuboid)
# Reset the world to reinitialize objects in the scene
world.reset()

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

# Load robot world and user configuration
robot_config_file_name = "my_franka_robot_config.yml"
world_config_file_name = "collision_base.yml"
user_config_file_name = "isaac_sim_user_config.toml"
curobo_motion_generator = CuroboMotionGenerator(
    robot_config_file_name=robot_config_file_name,
    world_config_file_name=world_config_file_name,
    user_config_file_name=user_config_file_name,
)

# Motion generation (will take a few seconds to initialize)
curobo_motion_generator.init_motion_gen()


# Get curobo activated joint index in the robot active joint
curobo_activated_joint_index = [
    robot.get_dof_index(x) for x in curobo_motion_generator.joint_names
]
print(f"curobo joint names={curobo_motion_generator.joint_names}")
print(f"curobo_activated_joint_index ={curobo_activated_joint_index}")


# Init
target_cuboid_position, target_cuboid_orientation = (
    target_cuboid.get_world_pose()
)  # wxyz
target_cuboid_orientation = curobo_motion_generator.wxzy_to_xyzw(
    target_cuboid_orientation
)  # xyzw
target_cuboid_pose = np.concatenate([target_cuboid_position, target_cuboid_orientation])

static_counter = 0
need_to_pick_and_place = True
print("Please click PLAY to start.")
while simulation_app.is_running():
    # simulation_app.update()
    # Step
    world.step(render=True)

    # Wait for play
    if not world.is_playing():
        continue
    # Get current_time_step_index
    current_time_step_index = world.current_time_step_index
    # Init physics handle when simulation starts
    if current_time_step_index < 2:
        # Reset world simulation physics context
        world.reset()
        # Init robot articulation controller physics handle
        robot.initialize()
        # Set robot position to curobo default joint positions
        robot.set_joint_positions(
            curobo_motion_generator.default_joint_positions,
            curobo_activated_joint_index,
        )
    # Wait for a while
    if current_time_step_index < 60:
        continue

    # Update obstacle at init
    if current_time_step_index == 60:
        print(
            f"Initializing isaac sim obstacle world with respect to {robot.prim_path}..."
        )
        # Initialize obstacle world
        curobo_motion_generator.update_obstacle_from_isaac_sim(
            world=world,
            robot_prim_path=robot.prim_path,
            ignore_prim_paths=[
                robot.prim_path,
                ground_plane.prim_path,
                target_cuboid.prim_path,
            ],
        )
    # Update obstacle every 500 steps
    if current_time_step_index % 1000 == 0:
        curobo_motion_generator.update_obstacle_from_isaac_sim(
            world=world,
            robot_prim_path=robot.prim_path,
            ignore_prim_paths=[
                robot.prim_path,
                ground_plane.prim_path,
                target_cuboid.prim_path,
            ],
        )

    # Check if robot is static
    robot_static = False
    if robot.get_joint_velocities() is not None:
        if np.max(np.abs(robot.get_joint_velocities())) < 0.1:
            robot_static = True
    # Check if target object is static
    target_object_static = False
    if target_cuboid.get_linear_velocity() is not None:
        if np.max(np.abs(target_cuboid.get_linear_velocity())) < 0.01:
            target_object_static = True

    if target_object_static and robot_static:
        static_counter += 1
    else:
        static_counter = 0

    # Get current cube pose
    current_cuboid_position, current_cuboid_orientation = (
        target_cuboid.get_world_pose()
    )  # wxyz
    current_cuboid_orientation = curobo_motion_generator.wxzy_to_xyzw(
        current_cuboid_orientation
    )  # xyzw
    current_cuboid_pose = np.concatenate(
        [current_cuboid_position, current_cuboid_orientation]
    )

    if np.linalg.norm(target_cuboid_pose - current_cuboid_pose) > 3e-2:
        # Update target cuboid pose
        target_cuboid_pose = current_cuboid_pose
        # Reset static counter
        static_counter = 0
        need_to_pick_and_place = True

    if static_counter > 100 and need_to_pick_and_place:
        # Reset static counter
        static_counter = 0
        need_to_pick_and_place = False
        print(f"Target cuboid pose={target_cuboid_pose}")

        def pick_and_place(target_cuboid_pose):

            # Get current robot joint positions
            current_robot_joint_positions = robot.get_joint_positions().tolist()
            current_robot_joint_positions_without_gripper = (
                current_robot_joint_positions[:-2]
            )

            # Get pre_grasp pose
            z_offset = 0.15  # m
            pre_grasp_pose = target_cuboid_pose.copy()
            pre_grasp_pose[2] += z_offset
            # Manual set place pose
            place_pose = [0.06303193, 0.43660322, 0.02499907, 0.0, 0.0, 0.0, 1.0]
            pre_place_pose = place_pose.copy()
            pre_place_pose[2] += z_offset
            # Manual set wait pose
            wait_pose = [0.06303193, 0.43660322, 0.35, 0.0, 0.0, 0.0, 1.0]

            # Generate trajectory
            solution_to_pre_grasp_pose = curobo_motion_generator.motion_generate(
                start_joint_angles=current_robot_joint_positions_without_gripper,
                goal_end_effector_pose=pre_grasp_pose,
            )

            if solution_to_pre_grasp_pose is None:
                return None

            last_joint_angles = solution_to_pre_grasp_pose["positions"][-1]
            solution_to_grasp_pose = curobo_motion_generator.motion_generate(
                start_joint_angles=last_joint_angles,
                goal_end_effector_pose=target_cuboid_pose,
            )
            if solution_to_grasp_pose is None:
                return None

            last_joint_angles = solution_to_grasp_pose["positions"][-1]

            solution_to_pre_place_pose = curobo_motion_generator.motion_generate(
                start_joint_angles=last_joint_angles,
                goal_end_effector_pose=pre_place_pose,
            )
            if solution_to_pre_place_pose is None:
                return None

            last_joint_angles = solution_to_pre_place_pose["positions"][-1]

            solution_to_place_pose = curobo_motion_generator.motion_generate(
                start_joint_angles=last_joint_angles,
                goal_end_effector_pose=place_pose,
            )
            if solution_to_place_pose is None:
                return None

            last_joint_angles = solution_to_place_pose["positions"][-1]
            solution_to_wait_pose = curobo_motion_generator.motion_generate(
                start_joint_angles=last_joint_angles, goal_end_effector_pose=wait_pose
            )

            if solution_to_wait_pose is None:
                return None

            # Execute the trajectories
            # Pre grasp
            actions = curobo_motion_generator.convert_solution_to_isaac_sim_action(
                solution=solution_to_pre_grasp_pose, robot_prim=robot
            )
            for action_index, action in enumerate(actions):
                action.joint_indices = curobo_activated_joint_index[:-2]
                action.joint_positions = action.joint_positions[:-2]
                action.joint_velocities = action.joint_velocities[:-2]
                action.joint_efforts = None
                articulation_controller.apply_action(action)
                # Step
                world.step(render=True)
                print(f"Action pre grasp {action_index} / {len(actions)} executed")
            # Grasp
            actions = curobo_motion_generator.convert_solution_to_isaac_sim_action(
                solution=solution_to_grasp_pose, robot_prim=robot
            )
            for action_index, action in enumerate(actions):
                action.joint_indices = curobo_activated_joint_index[:-2]
                action.joint_positions = action.joint_positions[:-2]
                action.joint_velocities = action.joint_velocities[:-2]
                action.joint_efforts = None

                articulation_controller.apply_action(action)
                # Step
                world.step(render=True)
                print(f"Action grasp {action_index} / {len(actions)} executed")

            # Close gripper
            gripper_close_joint_positions = solution_to_grasp_pose["positions"][-1]
            gripper_close_joint_positions.append(
                joint_limits_lower[7] + target_cuboid.get_size() / 2 - 0.005
            )
            gripper_close_joint_positions.append(
                joint_limits_lower[8] + target_cuboid.get_size() / 2 - 0.005
            )
            gripper_close_action = ArticulationAction(
                joint_positions=gripper_close_joint_positions,
                joint_indices=curobo_activated_joint_index,
            )
            articulation_controller.apply_action(gripper_close_action)
            for _ in range(50):
                world.step(render=True)

            # Pre place
            actions = curobo_motion_generator.convert_solution_to_isaac_sim_action(
                solution=solution_to_pre_place_pose, robot_prim=robot
            )
            for action_index, action in enumerate(actions):
                action.joint_indices = curobo_activated_joint_index[:-2]
                action.joint_positions = action.joint_positions[:-2]
                action.joint_velocities = action.joint_velocities[:-2]
                action.joint_efforts = None
                articulation_controller.apply_action(action)
                # Step
                world.step(render=True)
                print(f"Action pre place {action_index} / {len(actions)} executed")
            # Place
            actions = curobo_motion_generator.convert_solution_to_isaac_sim_action(
                solution=solution_to_place_pose, robot_prim=robot
            )
            for action_index, action in enumerate(actions):
                action.joint_indices = curobo_activated_joint_index[:-2]
                action.joint_positions = action.joint_positions[:-2]
                action.joint_velocities = action.joint_velocities[:-2]
                action.joint_efforts = None
                articulation_controller.apply_action(action)
                # Step
                world.step(render=True)
                print(f"Action place {action_index} / {len(actions)} executed")

            # Open gripper
            gripper_open_joint_positions = solution_to_place_pose["positions"][-1]
            gripper_open_joint_positions.append(joint_limits_upper[7])
            gripper_open_joint_positions.append(joint_limits_upper[8])
            gripper_open_action = ArticulationAction(
                joint_positions=gripper_open_joint_positions,
                joint_indices=curobo_activated_joint_index,
            )
            articulation_controller.apply_action(gripper_open_action)
            for _ in range(50):
                world.step(render=True)

            # Default
            actions = curobo_motion_generator.convert_solution_to_isaac_sim_action(
                solution=solution_to_wait_pose, robot_prim=robot
            )
            for action_index, action in enumerate(actions):
                action.joint_indices = curobo_activated_joint_index[:-2]
                action.joint_positions = action.joint_positions[:-2]
                action.joint_velocities = action.joint_velocities[:-2]
                action.joint_efforts = None
                articulation_controller.apply_action(action)
                # Step
                world.step(render=True)
                print(f"Action default {action_index} / {len(actions)} executed")

        # Run pick and place
        pick_and_place(target_cuboid_pose)

        # Update object target pose
        current_cuboid_position, current_cuboid_orientation = (
            target_cuboid.get_world_pose()
        )  # wxyz
        current_cuboid_orientation = curobo_motion_generator.wxzy_to_xyzw(
            current_cuboid_orientation
        )  # xyzw
        current_cuboid_pose = np.concatenate(
            [current_cuboid_position, current_cuboid_orientation]
        )
        target_cuboid_pose = current_cuboid_pose


simulation_app.close()
