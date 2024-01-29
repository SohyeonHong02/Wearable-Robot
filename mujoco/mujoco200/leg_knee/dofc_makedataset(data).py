import mujoco as mj
# from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import csv


# CSV 파일 / xml 파일
# csv_file_path  =  '/home/kist/mujoco/mujoco200/model/inverted/wearable/23-03-28_16_16_27_n.csv'   # 데이터 값 불러오기
# csv_file_path = 'mujoco/mujoco200/model/inverted/wearable/sarcopenia_continous_data.csv'
csv_file_path = 'mujoco/mujoco200/leg_knee/normal_continous_data.csv'

# xml_path       =  'LegKnee_170_60.xml'           # 제어하려는 xml model 불러오기
# 
simend = 50                        # simulation time (약 700초에 대한 데이터 / 전체 76030개의 데이터)
print_camera_config = 0            # set to 1 to print camera config (this is useful for initializing the view of the model) (# no )

DEG2RAD = 3.141592/180.0
RAD2DEG = 180.0/np.pi



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

FEcontrol_L = [];           FEcontrol_R = []
Kneecontrol_L = [];         Kneecontrol_R = []
DOFCcontrol_L = [];         DOFCcontrol_R = []

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
# kp_kn_1 = 3
# kv_kn_1 = 1

# kp_kn_2 = 0.7
# kv_kn_2 = 0.5

#Constant (DOFC gain)
# k_t1 = 10     # LEFT wearable robot
# k_t2 = 10      # RIGHT wearable robot
kappa = 3

# delta_t = 0 # delay 0sec
# delta_t = 6 # 0.1sec
delta_t = 9 # 0.15sec
# delta_t = 12 # 0.2sec
# delta_t = 15 # 0.25sec
# delta_t = 18 # 0.3sec
# delta_t = 21 # 0.3sec


# 시뮬레이션 주기와 제어 주기를 동일하게 설정 (final_model.xml의 timestep=0.01)
control_period = 0.0166

# MuJoCo 시뮬레이션 주기를 컨트롤 주기에 맞춰 변경
# model.opt.timestep = control_period


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

index = 0

weight = 60 # kg

while 1:

    desired_pos_FE_L = (position_LFE_data[index]) * DEG2RAD
    desired_vel_FE_L = (velocity_LFE_data[index]) * DEG2RAD
    desired_pos_FE_R = (position_RFE_data[index]) * DEG2RAD
    desired_vel_FE_R = (velocity_RFE_data[index]) * DEG2RAD

    # desired_pos_knee_L = (position_LKnee_data[index]) * DEG2RAD
    # desired_vel_knee_L = (velocity_LKnee_data[index]) * DEG2RAD
    # desired_pos_knee_R = (position_RKnee_data[index]) * DEG2RAD
    # desired_vel_knee_R = (velocity_RKnee_data[index]) * DEG2RAD

    # Low Pass Filter - FE
    LPF_pos_FE_L = (1 - alpha) * LPF_pre_pos_FE_L + alpha * desired_pos_FE_L
    LPF_vel_FE_L = (1 - alpha) * LPF_pre_vel_FE_L + alpha * desired_vel_FE_L
    LPF_pos_FE_R = (1 - alpha) * LPF_pre_pos_FE_R + alpha * desired_pos_FE_R
    LPF_vel_FE_R = (1 - alpha) * LPF_pre_vel_FE_R + alpha * desired_vel_FE_R
    
    LPF_pre_pos_FE_L = LPF_pos_FE_L
    LPF_pre_vel_FE_L = LPF_vel_FE_L
    LPF_pre_pos_FE_R = LPF_pos_FE_R
    LPF_pre_vel_FE_R = LPF_vel_FE_R

    noise_stddev = 0.005 # 잡음의 강도, 표준 편차 (조절 가능)
    noise_stddev2 = 0.02
    noise1 = np.random.normal(0, noise_stddev)
    noise2 = np.random.normal(0, noise_stddev2)
    noise3 = np.random.normal(0, noise_stddev)
    noise4 = np.random.normal(0, noise_stddev2)

    # LPF_pos_FE_L = LPF_pos_FE_L + noise1
    # LPF_vel_FE_L = LPF_vel_FE_L + noise2
    # LPF_pos_FE_R = LPF_pos_FE_R + noise3
    # LPF_vel_FE_R = LPF_vel_FE_R + noise4

    LPF_pos_FE_L = LPF_pos_FE_L
    LPF_vel_FE_L = LPF_vel_FE_L
    LPF_pos_FE_R = LPF_pos_FE_R
    LPF_vel_FE_R = LPF_vel_FE_R

    # Low Pass Filter - knee
    # LPF_pos_knee_L = (1 - alpha) * LPF_pre_pos_knee_L + alpha * desired_pos_knee_L
    # LPF_vel_knee_L = (1 - alpha) * LPF_pre_vel_knee_L + alpha * desired_vel_knee_L
    # LPF_pos_knee_R = (1 - alpha) * LPF_pre_pos_knee_R + alpha * desired_pos_knee_R
    # LPF_vel_knee_R = (1 - alpha) * LPF_pre_vel_knee_R + alpha * desired_vel_knee_R
    
    # LPF_pre_pos_knee_L = LPF_pos_knee_L
    # LPF_pre_vel_knee_L = LPF_vel_knee_L
    # LPF_pre_pos_knee_R = LPF_pos_knee_R
    # LPF_pre_vel_knee_R = LPF_vel_knee_R


    # Calculate control signals using PD controller
    # left_leg  = kp_1 * (LPF_pos_FE_L - data.qpos[0]) + kv_1 * (LPF_vel_FE_L - data.qvel[0])
    # right_leg = kp_2 * (LPF_pos_FE_R - data.qpos[3]) + kv_2 * (LPF_vel_FE_R - data.qvel[3])

    # left_knee = kp_kn_1 * (LPF_pos_knee_L - data.qpos[1]) + kv_kn_1 * (LPF_vel_knee_L - data.qvel[1])
    # right_knee = kp_kn_1 * (LPF_pos_knee_R - data.qpos[4]) + kv_kn_1 * (LPF_vel_knee_R - data.qvel[4])

    # Assign control values to the corresponding indices in data.ctrl
    # data.ctrl[0] = left_leg
    # data.ctrl[1] = right_leg
    # data.ctrl[4] = left_knee
    # data.ctrl[5] = right_knee
    # data.ctrl[0] =  15
    # data.ctrl[1] =  15

    
    gait_phs_L = 0
    # gait phase portrait method
    if (LPF_pos_FE_L >= 0) and (LPF_vel_FE_L <= 0):
        gait_phs_L = -np.arctan(LPF_vel_FE_L*0.16/ LPF_pos_FE_L)
    elif LPF_pos_FE_L < 0:
        gait_phs_L = -np.arctan(LPF_vel_FE_L*0.16/ LPF_pos_FE_L) + np.pi
    else:
        gait_phs_L = -np.arctan(LPF_vel_FE_L*0.16/ LPF_pos_FE_L) + 2*np.pi

    gait_phs_L = (gait_phs_L/((2)*np.pi)) * 100

    gait_phs_R = 0
    # gait phase portrait method
    if (LPF_pos_FE_R >= 0) and (LPF_vel_FE_R <= 0):
        gait_phs_R = -np.arctan(LPF_vel_FE_R*0.16/ LPF_pos_FE_R)
    elif LPF_pos_FE_R < 0:
        gait_phs_R = -np.arctan(LPF_vel_FE_R*0.16/ LPF_pos_FE_R) + np.pi
    else:
        gait_phs_R = -np.arctan(LPF_vel_FE_R*0.16/ LPF_pos_FE_R) + 2*np.pi

    gait_phs_R = (gait_phs_R/((2)*np.pi)) * 100

    # gait_phs_dot_L = (gait_phs_L - gait_phs_L_prev) / 0.0166
    # gait_phs_L_prev = gait_phs_L

    gait_torque_L = (-np.sin(gait_phs_L * ((3*np.pi)/132)))*kappa
    gait_torque_R = (-np.sin(gait_phs_R * ((3*np.pi)/132)))*kappa



    # Calculate the additional torque for assist_motor_1 and assist_motor_2
    y = np.sin(LPF_pos_FE_R) - np.sin(LPF_pos_FE_L)

    # 계산된 y(t)를 저장
    y_values.append(y)
    
    # τ(t) 계산
    if len(y_values) > delta_t:
        # print("\n", y, y_values[-1], y_values[-delta_t-1])
        tau_t = kappa * y_values[-delta_t-1] # time delay : 0.2s , simulation period : 0.01s
    else:
        tau_t = 0.0
    tau_values.append(tau_t)
    index += 1

    # data.ctrl[2] = tau_t #left
    # data.ctrl[3] = -tau_t # right

    # mj.mj_step(model, data)

    # print("desired_pos is ", LPF_pos_FE_L, LPF_pos_FE_R, LPF_pos_knee_L, LPF_pos_knee_R)
    # print("qpos is        ", data.qpos[0] , data.qpos[3], data.qpos[1], data.qpos[4]) 
    # print("desired_vel is ", LPF_vel_FE_L, LPF_vel_FE_R, LPF_vel_knee_L, LPF_vel_knee_R)
    # print("qvel is        ", data.qvel[0] , data.qvel[3], data.qvel[1], data.qvel[4]) 
    # print("ctrl is        ", data.ctrl)
    # print("time is        ", data.time, index)
    # print("gait phase is  ", gait_phs)
    # data_to_write.append(f"{LPF_pos_FE_L*RAD2DEG} {LPF_pos_FE_R*RAD2DEG} {LPF_vel_FE_L*RAD2DEG} {LPF_vel_FE_R*RAD2DEG} {gait_phs_L} {gait_phs_R} {tau_t}")
    data_to_write.append(f"{weight} {LPF_pos_FE_L*RAD2DEG} {LPF_pos_FE_R*RAD2DEG} {LPF_vel_FE_L*RAD2DEG} {LPF_vel_FE_R*RAD2DEG} {gait_torque_L}")
    
    # data_to_write.append(f"{index} {data.ctrl[0]} {data.ctrl[1]} {data.ctrl[2]} {data.ctrl[3]} {data.qpos[0]*RAD2DEG} {data.qpos[3]*RAD2DEG} {data.qvel[0]*RAD2DEG} {data.qvel[3]*RAD2DEG} {gait_phs_L} {gait_phs_R}")
    # index, left leg, right leg, left robot, right robot, left qpos, right qpos, left gait_phase, right gait_phase

    if (index >= 28500): 
        break


#     # Store data for plotting / 
    time_list.append(index*0.0166)
    desired_position_FE_L.append(LPF_pos_FE_L*200)              ;   desired_position_FE_R.append(LPF_pos_FE_R*200)
#     actual_position_FE_L.append(data.qpos[0])                ;   actual_position_FE_R.append(data.qpos[3])
    desired_velocity_FE_L.append(LPF_vel_FE_L*100)             ;   desired_velocity_FE_R.append(LPF_vel_FE_R*100)
#     actual_velocity_FE_L.append(data.qvel[0])               ;   actual_velocity_FE_R.append(data.qvel[3])
#     FEcontrol_L.append(data.ctrl[0])                       ;   FEcontrol_R.append(data.ctrl[1])
#     Kneecontrol_L.append(data.ctrl[4])                       ;   Kneecontrol_R.append(data.ctrl[5])
    DOFCcontrol_L.append(gait_torque_L)                       ;   DOFCcontrol_R.append(gait_torque_R)

    # desired_position_knee_L.append(LPF_pos_knee_L)              ;   desired_position_knee_R.append(LPF_pos_knee_R)
#     actual_position_knee_L.append(data.qpos[1])                ;   actual_position_knee_R.append(data.qpos[4])
    # desired_velocity_knee_L.append(LPF_vel_knee_L)             ;   desired_velocity_knee_R.append(LPF_vel_knee_R)
#     actual_velocity_knee_L.append(data.qvel[1])               ;   actual_velocity_knee_R.append(data.qvel[4])
    actual_gait_phase_L.append(gait_phs_L)                    ;   actual_gait_phase_R.append(gait_phs_R)


#     # get framebuffer viewport 
#     viewport_width, viewport_height = glfw.get_framebuffer_size(window)
#     viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

#     # print camera configuration (help to initialize the view) ( # don't )
#     if (print_camera_config == 1):
#         print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =', cam.elevation, ';', 'cam.distance = ', cam.distance)
#         print('cam.lookat =np.array([', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], '])')

#     # Update scene and render 
#     mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
#     mj.mjr_render(viewport, scene, context)

#     # swap OpenGL buffers (blocking call due to v-sync) 
#     glfw.swap_buffers(window)

#     # process pending GUI events, call GLFW callbacks 
#     glfw.poll_events()

# glfw.terminate() 
print(data_to_write)
output_file_path = 'mujoco/240122_weight_data/leg_knee_model/240122_with_weight_66%_a05_assist_addgaussiannoise_1.txt'
# output_file_path = 'mujoco/240116_percent_data/leg_knee_model/170cm_60kg/0119_with_k3_66%_a05_assist_addgaussiannoise.txt'
# output_file_path = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0111_without_a05_assist_posvelgait.txt'
with open(output_file_path, 'a') as output_file:
    for line in data_to_write:
        output_file.write(f"{line}\n")

####### graph plotting ############################################################################
# 

# # Create subplots for figure 1 
# fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# # Figure 1 - Joint 1
# # Plot position for joint 1
# ax1.plot(time_list, desired_position_FE_L, label='desired_position_FE_L')
# ax1.plot(time_list, actual_position_FE_L, label='actual_position_FE_L')

# ax1.set_xlabel('Time (sec)') ; ax1.set_ylabel('Position (rad)') 
# ax1.legend()

# # Plot velocity for joint 1
# ax2.plot(time_list, desired_velocity_FE_L, label='desired_velocity_FE_L')
# ax2.plot(time_list, actual_velocity_FE_L, label='actual_velocity_FE_L')

# ax2.set_xlabel('Time (sec)') ; ax2.set_ylabel('Velocity (rad/s)')
# ax2.legend()

# #-------------------------------------------------------
# # Create subplots for figure 1 
fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
ax1.plot(time_list, desired_position_FE_L, label='actual_position_FE_L')
ax1.plot(time_list, desired_velocity_FE_L, label='actual_velocity_FE_L')
ax1.plot(time_list, actual_gait_phase_L, label='actual_gait_phase_L')
ax1.plot(time_list, DOFCcontrol_L, label='DOFCcontrol_L')

ax1.set_xlabel('Time (sec)') ; ax1.set_ylabel('Position (rad)') 
ax1.legend()

# Plot velocity for joint 1
ax2.plot(time_list, desired_position_FE_R, label='actual_position_FE_R')
ax2.plot(time_list, desired_velocity_FE_R, label='actual_velocity_FE_R')
ax2.plot(time_list, actual_gait_phase_R, label='actual_gait_phase_R')
ax2.plot(time_list, DOFCcontrol_R, label='DOFCcontrol_R')

ax2.set_xlabel('Time (sec)') ; ax2.set_ylabel('Position (rad)')
ax2.legend()


# # ---------------------------------------------------------------------
# # Create subplots for figure 2
# fig2, (ax3, ax4) = plt.subplots(2, 1, sharex=True)

# # Figure 2 - Joint 2
# # Plot position for joint 2
# ax3.plot(time_list, desired_position_knee_L, label='desired_position_knee_L')
# ax3.plot(time_list, actual_position_knee_L, label='actual_position_knee_L')

# ax3.set_xlabel('Time (sec)') ; ax3.set_ylabel('Position (rad)') 
# ax3.legend()

# # Plot velocity for joint 2 
# ax4.plot(time_list, desired_velocity_knee_L, label='desired_velocity_knee_L')
# ax4.plot(time_list, actual_velocity_knee_L, label='actual_velocity_knee_L')

# ax4.set_xlabel('Time (sec)') ; ax4.set_ylabel('Velocity (rad/s)')
# ax4.legend()

# # ----------------------------------------------------------------------------
# # Create subplots for figure 3
# fig3, (ax5, ax6) = plt.subplots(2,1, sharex = True)

# # ax5.plot(time_list, desired_positions_1, label = 'desired_positions_1')
# # ax5.plot(time_list, actual_positions_1, label = 'actual_positions_L')
# ax5.plot(time_list, DOFCcontrol_L, label = 'DOFCcontrol_L')
# ax6.plot(time_list, DOFCcontrol_R, label = 'DOFCcontrol_R')



# ax5.set_xlabel('Time (sec)') ; ax5.set_ylabel('Torque (N.m)')
# ax5.legend()

# # ax6.plot(time_list, desired_positions_2, label='desired_positions_2')
# # ax6.plot(time_list, actual_positions_2, label = 'actual_positions_R')
# ax5.plot(time_list, FEcontrol_L, label = 'PDcontrol_L')
# ax6.plot(time_list, FEcontrol_R, label = 'PDcontrol_R')

# ax6.set_xlabel('Time (sec)') ; ax6.set_ylabel('Torque (N.m)')
# ax6.legend()

# # Show the plot for figure 2
plt.show()