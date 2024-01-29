import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import csv


# CSV 파일 / xml 파일
# csv_file_path = 'mujoco/mujoco200/model/inverted/wearable/sarcopenia_continous_data.csv'
csv_file_path = 'mujoco/mujoco200/leg_knee/normal_continous_data.csv'

xml_path       =  'model/LegKnee_170_60.xml'           # 제어하려는 xml model 불러오기
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

def calculate_desired_position(time, frequency, amplitude, offset):
    return amplitude * np.sin(2 * np.pi * frequency * time + offset)

def calculate_desired_velocity(time, frequency, amplitude, offset):
    return 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * time + offset)


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
desired_position_FE_L  = [];  desired_position_FE_R = []
actual_position_FE_L   = [];  actual_position_FE_R = []
desired_velocity_FE_L = [];  desired_velocity_FE_R = []
actual_velocity_FE_L  = [];  actual_velocity_FE_R = []

desired_position_AA_L  = [];  desired_position_AA_R = []
actual_position_AA_L   = [];  actual_position_AA_R = []
desired_velocity_AA_L = [];  desired_velocity_AA_R = []
actual_velocity_AA_L  = [];  actual_velocity_AA_R = []

desired_position_knee_L  = [];  desired_position_knee_R = []
actual_position_knee_L   = [];  actual_position_knee_R = []
desired_velocity_knee_L = [];  desired_velocity_knee_R = []
actual_velocity_knee_L  = [];  actual_velocity_knee_R = []

actual_gait_phase_L = []; actual_gait_phase_R = []
gait_phase_dot_L = []

FEcontrol_L = [];           FEcontrol_R = []
Kneecontrol_L = [];         Kneecontrol_R = []
DOFCcontrol_L = [];         DOFCcontrol_R = []
pos_dot_L_list = []; pos_dot_R_list = []

y_values = []
tau_values = []
data_to_write = []


## csv에서 데이터 불러오기 -----------------------------------------------------------------------------------
# 데이터를 저장할 리스트
time_data=[]
position_LFE_data = [] ; velocity_LFE_data = []
position_RFE_data= [] ; velocity_RFE_data= []
position_LKnee_data = [] ; velocity_LKnee_data = []
position_RKnee_data= [] ; velocity_RKnee_data= []

# CSV 파일 열기
with open(csv_file_path, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        time_data.append(float(row[0]))
        position_LFE_data.append(-float(row[4]))     # LFEpos B
        velocity_LFE_data.append(-float(row[8]))     # LFEvel C
        position_RFE_data.append(-float(row[1]))   # RFEpos Q
        velocity_RFE_data.append(-float(row[7]))   # RFEvel R

        position_LKnee_data.append(float(row[6]))     # LKnee pos AP
        velocity_LKnee_data.append(float(row[12]))     # LKnee vel AQ
        position_RKnee_data.append(float(row[3]))     # RKnee pos AU
        velocity_RKnee_data.append(float(row[11]))     # RKnee vel AV
        

#---------------------------------------------------------------------------------------

# PD controller gains

# 170_60
kp_1 = 600   # Proportional gain for joint 1 (LEFT)
kv_1 = 70    # Derivative gain for joint 1 Dgain은 100이하로 줄이기

kp_2 = 600    # Proportional gain for joint 2 (RIGHT)
kv_2 = 70    # Derivative gain for joint 2

# 158_50
# kp_1 = 400   # Proportional gain for joint 1 (LEFT)
# kv_1 = 50    # Derivative gain for joint 1 Dgain은 100이하로 줄이기

# kp_2 = 400    # Proportional gain for joint 2 (RIGHT)
# kv_2 = 50    # Derivative gain for joint 2

# 180_72
# kp_1 = 800   # Proportional gain for joint 1 (LEFT)
# kv_1 = 80    # Derivative gain for joint 1 Dgain은 100이하로 줄이기

# kp_2 = 800    # Proportional gain for joint 2 (RIGHT)
# kv_2 = 80    # Derivative gain for joint 2


kp_kn_1 = 6
kv_kn_1 = 3
kappa = 1.8

delta_t = 0 # delay 0sec
# delta_t = 6 # 0.1sec
# delta_t = 9 # 0.15sec
# delta_t = 12 # 0.2sec
# delta_t = 15 # 0.25sec
# delta_t = 18 # 0.3sec
# delta_t = 21 # 0.3sec


# 시뮬레이션 주기와 제어 주기를 동일하게 설정
control_period = 0.0166

# MuJoCo 시뮬레이션 주기를 컨트롤 주기에 맞춰 변경
model.opt.timestep = control_period


# left leg           |   right leg
frequency_L = 0.65   ;   frequency_R = 0.65
amplitude_L = 0.3    ;   amplitude_R = 0.3
offset_L = 0.0       ;   offset_R = np.pi

# LPF
LPF_pos_FE_L = 0     ; LPF_pos_FE_R = 0
LPF_pre_pos_FE_L = 0 ; LPF_pre_pos_FE_R = 0
LPF_pre_vel_FE_L = 0 ; LPF_pre_vel_FE_R = 0

LPF_pos_knee_L = 0     ; LPF_pos_knee_R = 0
LPF_pre_pos_knee_L = 0 ; LPF_pre_pos_knee_R = 0
LPF_pre_vel_knee_L = 0 ; LPF_pre_vel_knee_R = 0
alpha = 0.05

pos_dot_L = 0
pos_dot_R = 0

gait_phs_L_prev = 0

index = 0
cnt_L = 0
cnt_R = 0

while not glfw.window_should_close(window): 

    desired_pos_FE_L = (position_LFE_data[index]) * DEG2RAD
    desired_vel_FE_L = (velocity_LFE_data[index]) * DEG2RAD
    desired_pos_FE_R = (position_RFE_data[index]) * DEG2RAD
    desired_vel_FE_R = (velocity_RFE_data[index]) * DEG2RAD

    desired_pos_knee_L = (position_LKnee_data[index]) * DEG2RAD
    desired_vel_knee_L = (velocity_LKnee_data[index]) * DEG2RAD
    desired_pos_knee_R = (position_RKnee_data[index]) * DEG2RAD
    desired_vel_knee_R = (velocity_RKnee_data[index]) * DEG2RAD
    # Low Pass Filter - FE
    LPF_pos_FE_L = (1 - alpha) * LPF_pre_pos_FE_L + alpha * desired_pos_FE_L
    LPF_vel_FE_L = (1 - alpha) * LPF_pre_vel_FE_L + alpha * desired_vel_FE_L
    LPF_pos_FE_R = (1 - alpha) * LPF_pre_pos_FE_R + alpha * desired_pos_FE_R
    LPF_vel_FE_R = (1 - alpha) * LPF_pre_vel_FE_R + alpha * desired_vel_FE_R

    pos_dot_L = (LPF_pos_FE_L - LPF_pre_pos_FE_L) / control_period
    pos_dot_R = (LPF_pos_FE_R - LPF_pre_pos_FE_R) / control_period
    
    LPF_pre_pos_FE_L = LPF_pos_FE_L
    LPF_pre_vel_FE_L = LPF_vel_FE_L
    LPF_pre_pos_FE_R = LPF_pos_FE_R
    LPF_pre_vel_FE_R = LPF_vel_FE_R

    # Low Pass Filter - knee
    LPF_pos_knee_L = (1 - alpha) * LPF_pre_pos_knee_L + alpha * desired_pos_knee_L
    LPF_vel_knee_L = (1 - alpha) * LPF_pre_vel_knee_L + alpha * desired_vel_knee_L
    LPF_pos_knee_R = (1 - alpha) * LPF_pre_pos_knee_R + alpha * desired_pos_knee_R
    LPF_vel_knee_R = (1 - alpha) * LPF_pre_vel_knee_R + alpha * desired_vel_knee_R
    
    LPF_pre_pos_knee_L = LPF_pos_knee_L
    LPF_pre_vel_knee_L = LPF_vel_knee_L
    LPF_pre_pos_knee_R = LPF_pos_knee_R
    LPF_pre_vel_knee_R = LPF_vel_knee_R

    # Calculate control signals using PD controller
    left_leg  = kp_1 * (LPF_pos_FE_L - data.qpos[0]) + kv_1 * (LPF_vel_FE_L - data.qvel[0])
    right_leg = kp_2 * (LPF_pos_FE_R - data.qpos[3]) + kv_2 * (LPF_vel_FE_R - data.qvel[3])

    left_knee = kp_kn_1 * (LPF_pos_knee_L - data.qpos[1]) + kv_kn_1 * (LPF_vel_knee_L - data.qvel[1])
    right_knee = kp_kn_1 * (LPF_pos_knee_R - data.qpos[4]) + kv_kn_1 * (LPF_vel_knee_R - data.qvel[4])

    # Assign control values to the corresponding indices in data.ctrl
    data.ctrl[0] = left_leg
    data.ctrl[1] = right_leg
    data.ctrl[4] = left_knee
    data.ctrl[5] = right_knee


    gait_phs_R = 0
    # gait phase portrait method
    if (data.qpos[3] >= 0) and (pos_dot_R <= 0):
        gait_phs_R = -np.arctan((pos_dot_R*0.16)/ data.qpos[3])
    elif data.qpos[3] < 0:
        gait_phs_R = -np.arctan((pos_dot_R*0.16)/ data.qpos[3]) + np.pi
    else:
        gait_phs_R = -np.arctan((pos_dot_R*0.16)/ data.qpos[3]) + 2*np.pi

    gait_phs_R = (gait_phs_R/((2)*np.pi)) * 100

    gait_phs_L = 0
    # gait phase portrait method
    if (data.qpos[0] >= 0) and (pos_dot_L <= 0):
        gait_phs_L = -np.arctan((pos_dot_L*0.16)/ data.qpos[0])
    elif data.qpos[0] < 0:
        gait_phs_L = -np.arctan((pos_dot_L*0.16)/ data.qpos[0]) + np.pi
    else:
        gait_phs_L = -np.arctan((pos_dot_L*0.16)/ data.qpos[0]) + 2*np.pi

    gait_phs_L = (gait_phs_L/((2)*np.pi)) * 100


    gait_phs_dot_L = (gait_phs_L - gait_phs_L_prev) / 0.0166
    gait_phs_L_prev = gait_phs_L
    
    gait_torque_L = (-np.sin(gait_phs_L * ((3*np.pi)/132)))*kappa
    gait_torque_R = (-np.sin(gait_phs_R * ((3*np.pi)/132)))*kappa


    # Calculate the additional torque for assist_motor_1 and assist_motor_2
    y = np.sin(data.qpos[3]) - np.sin(data.qpos[0])

    # 계산된 y(t)를 저장
    y_values.append(y)
    
    # τ(t) 계산
    if len(y_values) > delta_t:
        print("\n", y, y_values[-1], y_values[-delta_t-1])
        tau_t = kappa * y_values[-delta_t-1] # time delay : 0.2s , simulation period : 0.01s
    else:
        tau_t = 0.0
    tau_values.append(tau_t)
    index += 1

    data.ctrl[2] = gait_torque_L #left
    data.ctrl[3] = gait_torque_R # right
    # data.ctrl[2] = tau_t #left
    # data.ctrl[3] = -tau_t # right

    mj.mj_step(model, data)

    print("desired_pos is ", LPF_pos_FE_L, LPF_pos_FE_R, LPF_pos_knee_L, LPF_pos_knee_R)
    print("qpos is        ", data.qpos[0] , data.qpos[3], data.qpos[1], data.qpos[4]) 
    print("desired_vel is ", LPF_vel_FE_L, LPF_vel_FE_R, LPF_vel_knee_L, LPF_vel_knee_R)
    print("qvel is        ", data.qvel[0] , data.qvel[3], data.qvel[1], data.qvel[4]) 
    print("ctrl is        ", data.ctrl)
    print("time is        ", data.time, index)
    # print("gait phase is  ", gait_phs)

    data_to_write.append(f"{index} {data.ctrl[0]} {data.ctrl[1]} {data.ctrl[2]} {data.ctrl[3]} {data.qpos[0]*RAD2DEG} {data.qpos[3]*RAD2DEG} {pos_dot_L*RAD2DEG} {pos_dot_R*RAD2DEG} {gait_phs_L} {gait_phs_R}")
    # index, left leg, right leg, left robot, right robot, left qpos, right qpos, left gait_phase, right gait_phase

    if (data.time >= simend): 
        break


    # Store data for plotting / 
    time_list.append(data.time)
    desired_position_FE_L.append(LPF_pos_FE_L)              ;   desired_position_FE_R.append(LPF_pos_FE_R)
    actual_position_FE_L.append(data.qpos[0])                ;   actual_position_FE_R.append(data.qpos[3])
    desired_velocity_FE_L.append(LPF_vel_FE_L)             ;   desired_velocity_FE_R.append(LPF_vel_FE_R)
    actual_velocity_FE_L.append(data.qvel[0])               ;   actual_velocity_FE_R.append(data.qvel[3])
    FEcontrol_L.append(data.ctrl[0])                       ;   FEcontrol_R.append(data.ctrl[1])
    Kneecontrol_L.append(data.ctrl[4])                       ;   Kneecontrol_R.append(data.ctrl[5])
    DOFCcontrol_L.append(data.ctrl[2])                       ;   DOFCcontrol_R.append(data.ctrl[3])

    desired_position_knee_L.append(LPF_pos_knee_L)              ;   desired_position_knee_R.append(LPF_pos_knee_R)
    actual_position_knee_L.append(data.qpos[1])                ;   actual_position_knee_R.append(data.qpos[4])
    desired_velocity_knee_L.append(LPF_vel_knee_L)             ;   desired_velocity_knee_R.append(LPF_vel_knee_R)
    actual_velocity_knee_L.append(data.qvel[1])               ;   actual_velocity_knee_R.append(data.qvel[4])
    actual_gait_phase_L.append(gait_phs_L)                    ;   actual_gait_phase_R.append(gait_phs_R)

    gait_phase_dot_L.append(gait_phs_dot_L)
    pos_dot_L_list.append(pos_dot_L)                        ; pos_dot_R_list.append(pos_dot_R)


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
output_file_path = '/home/kist/wearable/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0116_with_k8_d0_a05_assist_posvelgait_arrangegaitphase.txt'

# with open(output_file_path, 'w') as output_file:
#     for line in data_to_write:
#         output_file.write(f"{line}\n")

####### graph plotting ############################################################################
# 

# Create subplots for figure 1 
fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
ax1.plot(time_list, desired_position_FE_L, label='desired_position_FE_L')
ax1.plot(time_list, actual_position_FE_L, label='actual_position_FE_L')

ax1.set_xlabel('Time (sec)') ; ax1.set_ylabel('Position (rad)') 
ax1.legend()

# Plot velocity for joint 1
ax2.plot(time_list, desired_velocity_FE_L, label='desired_velocity_FE_L')
ax2.plot(time_list, actual_velocity_FE_L, label='actual_velocity_FE_L')

ax2.set_xlabel('Time (sec)') ; ax2.set_ylabel('Velocity (rad/s)')
ax2.legend()

#-------------------------------------------------------
# Create subplots for figure 1 
fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
ax1.plot(time_list, actual_position_FE_L, label='actual_position_FE_L')
ax1.plot(time_list, actual_velocity_FE_L, label='actual_velocity_FE_L')
ax1.plot(time_list, actual_gait_phase_L, label='actual_gait_phase_L')
ax1.plot(time_list, DOFCcontrol_L, label = 'DOFCcontrol_L')
# ax1.plot(time_list, DOFCcontrol_R, label = 'DOFCcontrol_R')
ax1.plot(time_list, FEcontrol_L, label = 'PDcontrol_L')
# ax1.plot(time_list, FEcontrol_R, label = 'PDcontrol_R')
ax1.plot(time_list, pos_dot_L_list, label = 'pos_dot_L_list')

# ax1.plot(time_list, gait_phase_dot_L, label='gait_phase_dot_L')

ax1.set_xlabel('Time (sec)') ; ax1.set_ylabel('Position (rad)') 
ax1.legend()

# Plot velocity for joint 1
ax2.plot(time_list, actual_position_FE_R, label='actual_position_FE_R')
ax2.plot(time_list, actual_velocity_FE_R, label='actual_velocity_FE_R')
ax2.plot(time_list, actual_gait_phase_R, label='actual_gait_phase_R')

ax2.set_xlabel('Time (sec)') ; ax2.set_ylabel('Position (rad)')
ax2.legend()


# ---------------------------------------------------------------------
# Create subplots for figure 2
fig2, (ax3, ax4) = plt.subplots(2, 1, sharex=True)

# Figure 2 - Joint 2
# Plot position for joint 2
ax3.plot(time_list, desired_position_knee_L, label='desired_position_knee_L')
ax3.plot(time_list, actual_position_knee_L, label='actual_position_knee_L')

ax3.set_xlabel('Time (sec)') ; ax3.set_ylabel('Position (rad)') 
ax3.legend()

# Plot velocity for joint 2 
ax4.plot(time_list, desired_velocity_knee_L, label='desired_velocity_knee_L')
ax4.plot(time_list, actual_velocity_knee_L, label='actual_velocity_knee_L')

ax4.set_xlabel('Time (sec)') ; ax4.set_ylabel('Velocity (rad/s)')
ax4.legend()

# ----------------------------------------------------------------------------
# Create subplots for figure 3
fig3, (ax5, ax6) = plt.subplots(2,1, sharex = True)

# ax5.plot(time_list, desired_positions_1, label = 'desired_positions_1')
# ax5.plot(time_list, actual_positions_1, label = 'actual_positions_L')
ax5.plot(time_list, DOFCcontrol_L, label = 'DOFCcontrol_L')
ax6.plot(time_list, DOFCcontrol_R, label = 'DOFCcontrol_R')



ax5.set_xlabel('Time (sec)') ; ax5.set_ylabel('Torque (N.m)')
ax5.legend()

# ax6.plot(time_list, desired_positions_2, label='desired_positions_2')
# ax6.plot(time_list, actual_positions_2, label = 'actual_positions_R')
ax5.plot(time_list, FEcontrol_L, label = 'PDcontrol_L')
ax6.plot(time_list, FEcontrol_R, label = 'PDcontrol_R')

ax6.set_xlabel('Time (sec)') ; ax6.set_ylabel('Torque (N.m)')
ax6.legend()

# Show the plot for figure 2
plt.show()