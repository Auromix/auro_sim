from isaacsim import SimulationApp

# See DEFAULT_LAUNCHER_CONFIG for available configuration
# https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.kit/docs/index.html
launch_config = {"headless": False}
# Launch the Toolkit
simulation_app = SimulationApp(launch_config)

# Locate any other import statement after this point
import omni

for i in range(10000):
    simulation_app.update()

# Close the running Toolkit
simulation_app.close()
