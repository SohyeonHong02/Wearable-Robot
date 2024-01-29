import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import csv


# CSV 파일 / xml 파일
csv_file_path  =  '/home/kist/wearable/mujoco/mujoco200/only_leg/23-03-28_16_16_27_n.csv'   # 데이터 값 불러오기
xml_path       =  '/home/kist/wearable/mujoco/mujoco200/only_leg/model/Leg_170_60.xml'           # 제어하려는 xml model 불러오기
# 
simend = 50                        # simulation time (약 700초에 대한 데이터 / 전체 76030개의 데이터)
print_camera_config = 0            # set to 1 to print camera config (this is useful for initializing the view of the model) (# no )

# For callback functions   (# no )
button_left = False ; button_middle = False ; button_right = False
lastx = 0 ; lasty = 0

DEG2RAD = 3.141592/180.0
RAD2DEG = 180.0/np.pi


def init_controller(model, data): # don't
    # initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    # put the controller here. This function is called inside the simulation.
    # pass
    pass

def set_torque_servo(actuator_no, flag):
    if (flag == 0):
        model.actuator_gainprm[actuator_no, 0] = 0
    else:
        model.actuator_gainprm[actuator_no, 0] = 1

def set_position_servo(actuator_no, kp):
    model.actuator_gainprm[actuator_no, 0] = kp
    model.actuator_biasprm[actuator_no, 1] = -kp

def set_velocity_servo(actuator_no, kv):
    model.actuator_gainprm[actuator_no, 0] = kv
    model.actuator_biasprm[actuator_no, 2] = -kv

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left 
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


# get the full path  (# no )
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures  (# no )
model = mj.MjModel.from_xml_path(xml_path)    #  MuJoCo model
data = mj.MjData(model)                       #  MuJoCo data
cam = mj.MjvCamera()                          #  Abstract camera
opt = mj.MjvOption()                          #  visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync  (# no)
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures (# no )
mj.mjv_defaultCamera(cam); mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks ( # no )
glfw.set_key_callback(window, keyboard); glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button); glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration (# no )
cam.azimuth = -90.68741727466428; cam.elevation = -2.8073894766455036; cam.distance = 5.457557373462702
cam.lookat = np.array([0.0, 0.0, 3.0])

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)


## -----------------------------------------------------------------------------------------------------------
# 아래에서 plot하기 위해 list 형태로 data 저장
time_list = []
desired_positions_1  = [];  desired_positions_2 = []
actual_positions_1   = [];  actual_positions_2 = []
desired_velocities_1 = [];  desired_velocities_2 = []
actual_velocities_1  = [];  actual_velocities_2 = []
PDcontrol_L = [];           PDcontrol_R = [];  
DOFCcontrol_L = [];         DOFCcontrol_R = []

y_values = []
tau_values = []
data_to_write = []


## csv에서 데이터 불러오기 -----------------------------------------------------------------------------------
# 데이터를 저장할 리스트
time_data=[]
left_position_data = [] ; left_velocity_data = []
right_position_data= [] ; right_velocity_data= []

# CSV 파일 열기
with open(csv_file_path, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        time_data.append(float(row[0]))
        left_position_data.append(float(row[1]))     # LFEpos B
        left_velocity_data.append(float(row[2]))     # LFEvel C
        right_position_data.append(float(row[16]))   # RFEpos Q
        right_velocity_data.append(float(row[17]))   # RFEvel R

#---------------------------------------------------------------------------------------

# 170cm 60kg
# PD controller gains
kp_1 = 400   # Proportional gain for joint 1 (LEFT)
kv_1 = 100    # Derivative gain for joint 1 Dgain은 100이하로 줄이기
        
kp_2 = 400    # Proportional gain for joint 2 (RIGHT)
kv_2 = 100    # Derivative gain for joint 2

# 180cm 72kg
# kp_1 = 1000   # Proportional gain for joint 1 (LEFT)
# kv_1 = 100    # Derivative gain for joint 1 Dgain은 100이하로 줄이기

# kp_2 = 1000    # Proportional gain for joint 2 (RIGHT)
# kv_2 = 100    # Derivative gain for joint 2

#Constant (DOFC gain)
# k_t1 = 10     # LEFT wearable robot
# k_t2 = 10      # RIGHT wearable robot
kappa = 8
delta_t = 35

# 시뮬레이션 주기와 제어 주기를 동일하게 설정 (final_model.xml의 timestep=0.01)
control_period = 0.01

# MuJoCo 시뮬레이션 주기를 컨트롤 주기에 맞춰 변경
model.opt.timestep = control_period



# LPF
LPF_pos_1 = 0 ; LPF_pos_2 = 0
LPF_pre_pos_1 = 0 ; LPF_pre_pos_2 = 0
LPF_pre_vel_1 = 0 ; LPF_pre_vel_2 = 0
alpha = 0.05

mi = 1
prv_R_pos = []

while not glfw.window_should_close(window): 
    # if len(prv_R_pos) > 25:

    #     if (data.qpos[2] < prv_R_pos[-26]):
    #         mi = -1
    #     else:
    #         mi = 1

    # Calculate desired position and velocity using the current index ()
    index = int((data.time - time_data[0]) / control_period)    # Calculate the index
    
    desired_pos_1 = (left_position_data[index]-19) * DEG2RAD
    desired_vel_1 = (left_velocity_data[index]) * DEG2RAD
    desired_pos_2 = (right_position_data[index]-19) * DEG2RAD
    desired_vel_2 = (right_velocity_data[index]) * DEG2RAD

    # Low Pass Filter
    LPF_pos_1 = (1 - alpha) * LPF_pre_pos_1 + alpha * desired_pos_1
    LPF_vel_1 = (1 - alpha) * LPF_pre_vel_1 + alpha * desired_vel_1
    LPF_pos_2 = (1 - alpha) * LPF_pre_pos_2 + alpha * desired_pos_2
    LPF_vel_2 = (1 - alpha) * LPF_pre_vel_2 + alpha * desired_vel_2
    
    LPF_pre_pos_1 = LPF_pos_1
    LPF_pre_vel_1 = LPF_vel_1
    LPF_pre_pos_2 = LPF_pos_2
    LPF_pre_vel_2 = LPF_vel_2

    

    # Calculate control signals using PD controller
    left_leg  = kp_1 * (LPF_pos_1 - data.qpos[0]) + kv_1 * (LPF_vel_1 - data.qvel[0])
    right_leg = kp_2 * (LPF_pos_2 - data.qpos[2]) + kv_2 * (LPF_vel_2 - data.qvel[2])

    # Assign control values to the corresponding indices in data.ctrl
    data.ctrl[0] = left_leg
    data.ctrl[1] = right_leg

    gait_phs_R = 0
    # gait phase portrait method
    if (data.qpos[3] >= 0) and (data.qvel[3] <= 0):
        gait_phs_R = -np.arctan((data.qvel[3]*0.15)/ data.qpos[3])
    elif data.qpos[3] < 0:
        gait_phs_R = -np.arctan((data.qvel[3]*0.15)/ data.qpos[3]) + np.pi
    else:
        gait_phs_R = -np.arctan((data.qvel[3]*0.15)/ data.qpos[3]) + 2*np.pi

    gait_phs_R = (gait_phs_R/((2)*np.pi)) * 100

    gait_phs_L = 0
    # gait phase portrait method
    if (data.qpos[0] >= 0) and (data.qvel[0] <= 0):
        gait_phs_L = -np.arctan((data.qvel[0]*0.15)/ data.qpos[0])
    elif data.qpos[0] < 0:
        gait_phs_L = -np.arctan((data.qvel[0]*0.15)/ data.qpos[0]) + np.pi
    else:
        gait_phs_L = -np.arctan((data.qvel[0]*0.15)/ data.qpos[0]) + 2*np.pi

    gait_phs_L = (gait_phs_L/((2)*np.pi)) * 100


    # gait_phs_dot_L = (gait_phs_L - gait_phs_L_prev) / 0.0166
    # gait_phs_L_prev = gait_phs_L

    gait_torque_L = (-np.sin(gait_phs_L * ((3*np.pi)/132)))*kappa
    gait_torque_R = (-np.sin(gait_phs_R * ((3*np.pi)/132)))*kappa


    # Calculate the additional torque for assist_motor_1 and assist_motor_2
    y = np.sin(data.qpos[2]) - np.sin(data.qpos[0])

    # 계산된 y(t)를 저장
    y_values.append(y)
    
    # τ(t) 계산
    if len(y_values) > delta_t:
        print("\n", y, y_values[-1], y_values[-delta_t-1])
        tau_t = kappa * y_values[-delta_t-1] # time delay : 0.25s , simulation period : 0.01s
    else:
        tau_t = 0.0
    tau_values.append(tau_t)

    # data.ctrl[2] = gait_torque_L #left
    # data.ctrl[3] = gait_torque_R # right
    data.ctrl[2] = tau_t #left
    data.ctrl[3] = -tau_t # right

    mj.mj_step(model, data)

    print("desired_pos is ", LPF_pos_1, LPF_pos_2)
    print("qpos is        ", data.qpos[0] , data.qpos[2]) 
    print("desired_vel is ", LPF_vel_1, LPF_vel_2)
    print("qvel is        ", data.qvel[0] , data.qvel[2]) 
    print("ctrl is        ", data.ctrl)
    print("time is        ", index)

    # data_to_write.append(f"{index} {data.ctrl[0]} {data.ctrl[1]} {data.ctrl[2]} {data.ctrl[3]} {data.qpos[0]*RAD2DEG} {data.qpos[2]*RAD2DEG} {data.qvel[0]*RAD2DEG} {data.qvel[2]*RAD2DEG} {gait_phs_L} {gait_phs_R}")
    data_to_write.append(f"{index} {data.ctrl[0]} {data.ctrl[1]} {data.ctrl[2]} {data.ctrl[3]} {data.qpos[0]*RAD2DEG} {data.qpos[2]*RAD2DEG}")

    if (data.time >= simend): 
        break


    # Store data for plotting / 
    time_list.append(data.time)
    desired_positions_1.append(LPF_pos_1)              ;   desired_positions_2.append(LPF_pos_2)
    actual_positions_1.append(data.qpos[0])                ;   actual_positions_2.append(data.qpos[2])
    desired_velocities_1.append(LPF_vel_1)             ;   desired_velocities_2.append(LPF_vel_2)
    actual_velocities_1.append(data.qvel[0])               ;   actual_velocities_2.append(data.qvel[2])
    PDcontrol_L.append(data.ctrl[0])                       ;   PDcontrol_R.append(data.ctrl[1])
    DOFCcontrol_L.append(data.ctrl[2])                       ;   DOFCcontrol_R.append(data.ctrl[3])


    # get framebuffer viewport 
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # print camera configuration (help to initialize the view) ( # don't )
    if (print_camera_config == 1):
        print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =', cam.elevation, ';', 'cam.distance = ', cam.distance)
        print('cam.lookat =np.array([', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], '])')

    # Update scene and render 
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync) 
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks 
    glfw.poll_events()

glfw.terminate() 

# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/158cm_50kg/without_assist_0104_1.txt'
# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/158cm_50kg/0105_with_k5_d03_a05_assist.txt'
# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/158cm_50kg/with_k8_025_assist_0104_1.txt'

# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/without_assist_0104_1.txt'


# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0116_with_k8_61%_a05_assist_gaitphase.txt'
# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0116_with_k8_d025_a05_assist_gaitphase_arrangegaitphase.txt'
output_file_path = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0123_with_k8_d035_a05_assist.txt'

# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0111_without_a05_assist_posvelgait.txt'


# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/180cm_72kg/without_assist_0104_1.txt'
# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/180cm_72kg/0105_with_k8_d03_a05_assist.txt'
# output_file_path = 'mujoco/240104_parameter_result/only_leg_model/180cm_72kg/with_k15_025_assist_0104_1.txt'


with open(output_file_path, 'w') as output_file:
    for line in data_to_write:
        output_file.write(f"{line}\n")

####### graph plotting ############################################################################
# 

# Create subplots for figure 1 
fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
ax1.plot(time_list, desired_positions_1, label='Left Desired Position 1')
ax1.plot(time_list, actual_positions_1, label='Actual Position 1')

ax1.set_ylabel('Position')
ax1.legend()

# Plot velocity for joint 1
ax2.plot(time_list, desired_velocities_1, label='Desired Velocity 1')
ax2.plot(time_list, actual_velocities_1, label='Actual Velocity 1')

ax2.set_xlabel('Time') ; ax2.set_ylabel('Velocity')
ax2.legend()


# ---------------------------------------------------------------------
# Create subplots for figure 2
fig2, (ax3, ax4) = plt.subplots(2, 1, sharex=True)

# Figure 2 - Joint 2
# Plot position for joint 2
ax3.plot(time_list, desired_positions_2, label='Right Desired Position 2')
ax3.plot(time_list, actual_positions_2, label='Actual Position 2')

ax3.set_ylabel('Position')
ax3.legend()

# Plot velocity for joint 2 
ax4.plot(time_list, desired_velocities_2, label='Desired Velocity 2')
ax4.plot(time_list, actual_velocities_2, label='Actual Velocity 2')

ax4.set_xlabel('Time') ; ax4.set_ylabel('Velocity')
ax4.legend()

# ----------------------------------------------------------------------------
# Create subplots for figure 3
fig3, (ax5, ax6) = plt.subplots(2,1, sharex = True)

# ax5.plot(time_list, desired_positions_1, label = 'desired_positions_1')
# ax5.plot(time_list, actual_positions_1, label = 'actual_positions_L')
ax5.plot(time_list, DOFCcontrol_L, label = 'DOFCcontrol_L')
ax6.plot(time_list, DOFCcontrol_R, label = 'DOFCcontrol_R')

ax5.set_xlabel('Time') ; ax5.set_ylabel('Torque')
ax5.legend()

# ax6.plot(time_list, desired_positions_2, label='desired_positions_2')
# ax6.plot(time_list, actual_positions_2, label = 'actual_positions_R')
ax5.plot(time_list, PDcontrol_L, label = 'PDcontrol_L')
ax6.plot(time_list, PDcontrol_R, label = 'PDcontrol_R')

ax6.set_xlabel('Time') ; ax6.set_ylabel('Torque')
ax6.legend()

# Show the plot for figure 2
plt.show()

