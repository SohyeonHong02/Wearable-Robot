import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import csv
import random




# CSV 파일 / xml 파일
csv_file_path  =  '/home/kist/wearable/mujoco/mujoco200/only_leg/23-03-28_16_16_27_n.csv'   # 데이터 값 불러오기
xml_path       =  '/home/kist/wearable/mujoco/mujoco200/only_leg/model/exoskeleton_model.xml'           # 제어하려는 xml model 불러오기

simend = 30                        # simulation time (약 700초에 대한 데이터 / 전체 76030개의 데이터)
print_camera_config = 0            # set to 1 to print camera config (this is useful for initializing the view of the model) (# no )

# For callback functions   (# no )
button_left = False ; button_middle = False ; button_right = False
lastx = 0 ; lasty = 0

DEG2RAD = 3.141592/180.0
RAD2DEG = 180.0/np.pi


def calculate_desired_position(time, frequency, amplitude, offset):
    return amplitude * np.sin(2 * np.pi * frequency * time + offset)

def calculate_desired_velocity(time, frequency, amplitude, offset):
    return 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * time + offset)

## -----------------------------------------------------------------------------------------------------------
# 아래에서 plot하기 위해 list 형태로 data 저장
time_list = []
desired_positions_1  = [];  desired_positions_2 = []
actual_positions_1   = [];  actual_positions_2 = []
desired_velocities_1 = [];  desired_velocities_2 = []
actual_velocities_1  = [];  actual_velocities_2 = []
PDcontrol_L = [];           PDcontrol_R = [];  
DOFCcontrol_L = [];         DOFCcontrol_R = []

q_left = []
q_right = []
y_values = []
tau_values = []
tau_pr = []

data_to_write = []


## csv에서 데이터 불러오기 -----------------------------------------------------------------------------------
# 데이터를 저장할 리스트
time_data=[]
left_position_data = [] ; left_velocity_data = []
right_position_data= [] ; right_velocity_data= []

# CSV 파일 열기
# with open(csv_file_path, 'r') as csv_file:
#     csv_reader = csv.reader(csv_file)
#     for row in csv_reader:
#         time_data.append(float(row[0]))
#         left_position_data.append(float(row[1]))     # LFEpos B
#         left_velocity_data.append(float(row[2]))     # LFEvel C
#         right_position_data.append(float(row[16]))   # RFEpos Q
#         right_velocity_data.append(float(row[17]))   # RFEvel R

#---------------------------------------------------------------------------------------

# PD controller gains
kp_1 = 3000   # Proportional gain for joint 1 (LEFT)
kv_1 = 80    # Derivative gain for joint 1 Dgain은 100이하로 줄이기

kp_2 = 3000    # Proportional gain for joint 2 (RIGHT)
kv_2 = 80    # Derivative gain for joint 2

#Constant (DOFC gain)
# k_t1 = 10     # LEFT wearable robot
# k_t2 = 10      # RIGHT wearable robot
kappa = 10

# 시뮬레이션 주기와 제어 주기를 동일하게 설정 (final_model.xml의 timestep=0.01)
control_period = 0.01

# MuJoCo 시뮬레이션 주기를 컨트롤 주기에 맞춰 변경
# model.opt.timestep = control_period

delta_t = 0.25

# left leg           |   right leg
frequency = 0.6    ;   frequency = 0.6
amplitude = 0.4    ;   amplitude = 0.4
offset_L = 0.0       ;   offset_R = np.pi

# LPF
LPF_pos_1 = 0 ; LPF_pos_2 = 0
LPF_pre_pos_1 = 0 ; LPF_pre_pos_2 = 0
LPF_pre_vel_1 = 0 ; LPF_pre_vel_2 = 0
alpha = 0.03

time_interval = 0.01

# while not glfw.window_should_close(window): 
t_ = 0
real_time = 0
i = 0
j=0
shift_ = 6.8551768380947475*DEG2RAD
pattern = 0
k = 0

# # 0~120
for i in np.arange(-30, 30.1, 0.5):
    desired_pos_1 = i
    for j in np.arange(-30, 30.1, 0.5):
        desired_pos_2 = j
        desired_positions_1.append(desired_pos_1)
        desired_positions_2.append(desired_pos_2)

# 0~120
# for i in np.arange(-15, 15.1, 1):
#     desired_pos_1 = i
#     for j in np.arange(-15, 15.1, 1):
#         desired_pos_2 = j
#         for m in np.arange(-16, 16.1, 1):
#             desired_vel_1 = m
#             for l in np.arange(-16, 16.1,1):
#                 desired_vel_2 = l
#                 desired_positions_1.append(desired_pos_1)
#                 desired_positions_2.append(desired_pos_2)
#                 desired_velocities_1.append(desired_vel_1)
#                 desired_velocities_2.append(desired_vel_2)


# print(len(desired_positions_1))
# print(len(desired_positions_2))

while 1:
    # sim_time = 72000
    # if k >= sim_time:
    #     break
    # print(k)
    if k >= len(desired_positions_1):
        break
    # print(desired_positions_1[k], desired_positions_2[k])

    # Calculate the additional torque for assist_motor_1 and assist_motor_2
    y = np.sin(desired_positions_2[k]*DEG2RAD) - np.sin(desired_positions_1[k]*DEG2RAD)
    # ydot = np.cos(desired_positions_2[k]*DEG2RAD) - np.cos(desired_positions_1[k]*DEG2RAD)

    # assist_torque_left  = k_t1 * y * (t_ - delta_t)
    # assist_torque_right = k_t2 * -y * (t_ - delta_t) 

    y_values.append(y)
    
    tau_t = kappa * y

    # data.ctrl[2] = tau_t #left
    # data.ctrl[3] = -tau_t # right

    # data_to_write.append(f"{desired_pos_1} {desired_pos_2} {desired_vel_1} {desired_vel_2} {tau_t}") # left right torque

    # print(f"{desired_positions_1[k]} {desired_positions_2[k]}", y)
    data_to_write.append(f"{desired_positions_1[k]} {desired_positions_2[k]} {y*10}") # left right torque

    # print("desired_pos is ", LPF_pos_1, LPF_pos_2)
    # print("qpos is        ", data.qpos[0] , data.qpos[2]) 
    # print("desired_vel is ", LPF_vel_1, LPF_vel_2)
    # print("qvel is        ", data.qvel[0] , data.qvel[2]) 
    # print("ctrl is        ", data.ctrl)
    # print("time is        ", real_time)

    k = k + 1
    # if (t_ >= simend): 
    #     break

    

    # Store data for plotting / 
    time_list.append(k)
    # desired_positions_1.append(desired_pos_1*RAD2DEG)                  ;   desired_positions_2.append(desired_pos_2*RAD2DEG)
    # actual_positions_1.append(data.qpos[0])                ;   actual_positions_2.append(data.qpos[2])
    # desired_velocities_1.append(LPF_vel_1)                 ;   desired_velocities_2.append(LPF_vel_2)
    # actual_velocities_1.append(data.qvel[0])               ;   actual_velocities_2.append(data.qvel[2])
    # PDcontrol_L.append(data.ctrl[0])                       ;   PDcontrol_R.append(data.ctrl[1])
    # DOFCcontrol_L.append(data.ctrl[2])                     ;   DOFCcontrol_R.append(data.ctrl[3])


    # get framebuffer viewport 
    # viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    # viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # # print camera configuration (help to initialize the view) ( # don't )
    # if (print_camera_config == 1):
    #     print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =', cam.elevation, ';', 'cam.distance = ', cam.distance)
    #     print('cam.lookat =np.array([', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], '])')

#     # Update scene and render 
#     mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
#     mj.mjr_render(viewport, scene, context)

#     # swap OpenGL buffers (blocking call due to v-sync) 
#     glfw.swap_buffers(window)

#     # process pending GUI events, call GLFW callbacks 
#     glfw.poll_events()

# glfw.terminate() 
print(k)
print("pattern", pattern)

output_file_path = '/home/kist/mujoco/mlp_train_30(0.1).txt'
with open(output_file_path, 'w') as output_file:
    for line in data_to_write:
        output_file.write(f"{line}\n")




####### graph plotting ############################################################################
# 

# Create subplots for figure 1 
fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
ax1.plot(time_list[10000:20000], desired_positions_1[10000:20000], label='Left Desired Position')
ax1.plot(time_list[10000:20000], desired_positions_2[10000:20000], label='Right Desired Position')

ax1.set_ylabel('Position')
ax1.legend()

# # Plot velocity for joint 1
# ax2.plot(time_list[10000:20000], tau_values[10000:20000], label='tau')
# # ax2.plot(time_list, actual_velocities_1, label='Actual Velocity 1')

# ax2.set_xlabel('Time') ; ax2.set_ylabel('Nm')
# ax2.legend()

plt.show()

# # ---------------------------------------------------------------------
# # Create subplots for figure 2
# fig2, (ax3, ax4) = plt.subplots(2, 1, sharex=True)

# # Figure 2 - Joint 2
# # Plot position for joint 2
# ax3.plot(time_list, desired_positions_2, label='Right Desired Position 2')
# ax3.plot(time_list, actual_positions_2, label='Actual Position 2')

# ax3.set_ylabel('Position')
# ax3.legend()

# # Plot velocity for joint 2 
# ax4.plot(time_list, desired_velocities_2, label='Desired Velocity 2')
# ax4.plot(time_list, actual_velocities_2, label='Actual Velocity 2')

# ax4.set_xlabel('Time') ; ax4.set_ylabel('Velocity')
# ax4.legend()

# # ----------------------------------------------------------------------------
# # Create subplots for figure 3
# fig3, (ax5, ax6) = plt.subplots(2,1, sharex = True)

# ax5.plot(time_list, desired_positions_1, label = 'desired_positions_1')
# ax5.plot(time_list, actual_positions_1, label = 'actual_positions_L')
# ax5.plot(time_list, DOFCcontrol_L, label = 'DOFCcontrol_L')


# ax5.set_xlabel('Time') ; ax5.set_ylabel('PD control')
# ax5.legend()

# ax6.plot(time_list, desired_positions_2, label='desired_positions_2')
# ax6.plot(time_list, actual_positions_2, label = 'actual_positions_R')
# ax6.plot(time_list, DOFCcontrol_R, label = 'DOFC control R')

# ax6.set_xlabel('Time') ; ax6.set_ylabel('DOFC control')
# ax6.legend()

# # Show the plot for figure 2
# plt.show()

