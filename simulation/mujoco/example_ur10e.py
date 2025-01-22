import mujoco
import mujoco_viewer
from robot_descriptions.loaders.mujoco import load_robot_description

model = load_robot_description("panda_mj_description")
data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)

# simulate and render
for _ in range(10000):
    if viewer.is_alive:
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()