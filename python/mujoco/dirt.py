import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.linalg import solve_continuous_are

model = mujoco.MjModel.from_xml_path("../../Fusion/cartPole_description/mujoco/cartpole.xml")
data = mujoco.MjData(model)

