import mujoco
import mujoco.viewer
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.abspath(os.path.join(script_dir, "../../../models/urdf/colony.urdf"))

# The URDF mesh paths are "meshes/*.STL". Some MuJoCo versions drop the
# directory part, so try models/meshes first and then models/.
models_dir = os.path.abspath(os.path.join(script_dir, "../../../models"))
for mesh_dir in (os.path.join(models_dir, "meshes"), models_dir):
  spec = mujoco.MjSpec.from_file(urdf_path)
  spec.meshdir = mesh_dir
  try:
    model = spec.compile()
    break
  except ValueError:
    continue
else:
  raise RuntimeError("Could not find the URDF meshes under " + models_dir)
data = mujoco.MjData(model)
model.opt.gravity[:] = 0.0


with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()