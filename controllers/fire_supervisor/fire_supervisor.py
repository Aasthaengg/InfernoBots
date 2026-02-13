"""Fire Supervisor: Randomizes fire location at simulation start."""

from controller import Supervisor
import numpy as np

# Initialize supervisor
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Get reference to the FIRE node
fire_node = supervisor.getFromDef("FIRE")

if fire_node is not None:
    # Generate random fire location
    fire_x = np.random.uniform(-150, 150)
    fire_y = np.random.uniform(-150, 150)
    fire_z = np.random.uniform(5, 15)

    # Get the translation field and update it
    translation_field = fire_node.getField("translation")
    translation_field.setSFVec3f([fire_x, fire_y, fire_z])

    print("=" * 60)
    print("🔥 FIRE SUPERVISOR - Fire Randomized")
    print("=" * 60)
    print(f"Fire location: [{fire_x:.2f}, {fire_y:.2f}, {fire_z:.2f}]")
    print("=" * 60)
else:
    print("ERROR: FIRE node not found!")

# Keep supervisor running
while supervisor.step(timestep) != -1:
    pass
