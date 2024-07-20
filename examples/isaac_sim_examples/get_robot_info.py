# get_robot_info.py
# You can also get the robot from the scene by its nickname
from omni.isaac.core.world import World

world = World()
world.reset()
robot = world.scene.get_object("my_robot")


num_dof = robot.num_dof
num_of_body_parts = robot.num_bodies
num_of_all_joint = robot._articulation_view.num_joints
active_joint_names = robot.dof_names
active_joint_indexes = [robot.get_dof_index(x) for x in active_joint_names]
print(f"Num of dof(active joints)={num_dof}")
print(f"Num of all joints={num_of_all_joint}")
print(f"Num of body parts={num_of_body_parts}")
print(f"Active joint names={active_joint_names}")
print(f"Active joint indexes={active_joint_indexes}")


def print_dof_properties(robot, joint_name=None, index=None):
    """
    Function to query and print properties of a DOF based on joint_name or index.

    Args:
        robot: The robot instance of Robot class in omni.isaac.core.robots
        joint_name (str, optional): The name of the active joint
        index (int, optional): The index of the active joint

    Raises:
        ValueError: If neither joint_name nor index is provided or if both are provided
    """
    # Get DOF properties
    dof_properties = robot.dof_properties

    # Check if joint_name or index is correctly provided
    if joint_name is None and index is None:
        raise ValueError("Either joint_name or index must be provided.")
    if joint_name is not None and index is not None:
        raise ValueError(
            "Only one of joint_name or index should be provided, not both."
        )

    # Query DOF properties based on joint_name or index
    if joint_name is not None:
        joint_names = robot.dof_names
        if joint_name not in joint_names:
            raise ValueError(f"joint_name {joint_name} does not exist.")

        index = robot.get_dof_index(joint_name)
    elif index is not None:
        if index < 0 or index >= robot.num_dof:
            raise ValueError(f"index {index} is out of range.")
        joint_name = robot.dof_names[index]

    # Print DOF properties
    joint_properties = dof_properties[index]
    print(
        f"DOF properties (joint_name: {joint_name if joint_name else 'N/A'}, index: {index}):"
    )
    print(f"Type: {joint_properties['type']}")
    print(f"Lower limit: {joint_properties['lower']}")
    print(f"Upper limit: {joint_properties['upper']}")
    print(f"Drive mode: {joint_properties['driveMode']}")
    print(f"Max effort: {joint_properties['maxEffort']}")
    print(f"Max velocity: {joint_properties['maxVelocity']}")
    print(f"Stiffness: {joint_properties['stiffness']}")
    print(f"Damping: {joint_properties['damping']}")
    print("")
    return joint_properties


# for joint_name in active_joint_names:
#     print_dof_properties(robot, joint_name=joint_name)

for index in active_joint_indexes:
    print_dof_properties(robot, index=index)
