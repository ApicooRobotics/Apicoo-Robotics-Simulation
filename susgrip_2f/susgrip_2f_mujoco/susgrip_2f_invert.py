import time
import numpy as np
import math
import mujoco
import mujoco.viewer
# import keyboard

model = mujoco.MjModel.from_xml_path('./scene.xml')
data = mujoco.MjData(model)

t = 0
def handle():
  global t
  t += 2
  t = t % 1300
  dis = t/10

  radian = math.acos((63.45-dis)/108)
  yF = math.sqrt(2916-(31.725-dis/2)**2)+3.5
  slider = (57.5-yF)/1000
  buff = -math.pi/2+radian
  
  data.qpos[0] = -(-dis+65)/2000    # finger_outer_l_joint
  data.qpos[1] = -buff              # inner_slider_l_joint
  data.qpos[2] = 2*buff             # base_pad_l_joint
  data.qpos[3] = 2*buff             # base_pad_link_l_joint             
  data.qpos[4] = -buff           # outer_slider_l_joint
  data.qpos[5] = slider            # slider_l_joint
  
  data.qpos[6] = (-dis+65)/2000
  data.qpos[7] = buff
  data.qpos[8] = -2*buff
  data.qpos[9] = -2*buff
  data.qpos[10] = buff
  data.qpos[11] = slider
  
  print(data.qpos)
  
with mujoco.viewer.launch_passive(model, data) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  
  start = time.time()
  while viewer.is_running() and time.time() - start < 50:
    step_start = time.time()
    
    handle()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(model, data)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
