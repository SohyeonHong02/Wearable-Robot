import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

xml_path='copy_InvertedPendulum.xml'
simend=10
print_camera_config=0 # set to 1 to print camera config
 # this is useful for initializing view of the model

# For callback functions
button_left=False
button_middle=False
button_right=False
lastx=0
lasty=0

def init_controller(model, data):
    # Initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation
    pass