import csv
import numpy as np
import matplotlib.pyplot as plt
import math


csv_file_path  =  '/home/kist/mujoco/mujoco200/model/inverted/wearable/23-03-28_16_16_27_n.csv'   # 데이터 값 불러오기

DEG2RAD = 3.141592/180.0
RAD2DEG = 180.0/np.pi

## csv에서 데이터 불러오기 -----------------------------------------------------------------------------------
# 데이터를 저장할 리스트
time_data=[]
left_position_data = [] ; left_velocity_data = []
right_position_data= [] ; right_velocity_data= []

data_to_write = []

time_list = []
desired_positions_1  = [];  desired_positions_2 = []
tau_values = []


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
frequency_L = 0.6    ;   frequency_R = 0.6
amplitude_L = 0.4    ;   amplitude_R = 0.4
offset_L = 0.0       ;   offset_R = np.pi

# LPF
LPF_pos_L = 0 ; LPF_pos_R = 0
LPF_pre_pos_L = 0 ; LPF_pre_pos_R = 0
LPF_pre_vel_L = 0 ; LPF_pre_vel_R = 0
alpha = 0.03

# while not glfw.window_should_close(window): 
t_ = 0
i = 0
y_values = []
LR_p = 0
LR_points = []


while 1:
    if i >= 75000:
        break

    desired_pos_L = (left_position_data[i]-15) * DEG2RAD
    desired_vel_L = (left_velocity_data[i]) * DEG2RAD
    desired_pos_R = (right_position_data[i]-15) * DEG2RAD
    desired_vel_R = (right_velocity_data[i]) * DEG2RAD
    

    # Low Pass Filter
    LPF_pos_L = (1 - alpha) * LPF_pre_pos_L + alpha * desired_pos_L
    LPF_vel_L = (1 - alpha) * LPF_pre_vel_L + alpha * desired_vel_L
    LPF_pos_R = (1 - alpha) * LPF_pre_pos_R + alpha * desired_pos_R
    LPF_vel_R = (1 - alpha) * LPF_pre_vel_R + alpha * desired_vel_R
    
    LPF_pre_pos_L = LPF_pos_L
    LPF_pre_vel_L = LPF_vel_L
    LPF_pre_pos_R = LPF_pos_R
    LPF_pre_vel_R = LPF_vel_R

    # if np.abs(LPF_pos_L*RAD2DEG - LPF_pos_R*RAD2DEG) < 1.0:
    #     LR_p = LPF_pos_L*RAD2DEG
    #     LR_points.append(LR_p)

    

    # Calculate control signals using PD controller
    # left_leg  = kp_1 * (LPF_pos_1 - data.qpos[0]) + kv_1 * (LPF_vel_1 - data.qvel[0])
    # right_leg = kp_2 * (LPF_pos_2 - data.qpos[2]) + kv_2 * (LPF_vel_2 - data.qvel[2])

    # Assign control values to the corresponding indices in data.ctrl
    # data.ctrl[0] = left_leg
    # data.ctrl[1] = right_leg
    # data.ctrl[0] =  15
    # data.ctrl[1] =  15

    # Calculate the additional torque for assist_motor_1 and assist_motor_2
    y = np.sin(LPF_pos_R) - np.sin(LPF_pos_L)
    ydot = np.cos(LPF_pos_R) - np.cos(LPF_pos_L)

    # assist_torque_left  = k_t1 * y * (data.time - delta_t)
    # assist_torque_right = k_t2 * -y * (data.time - delta_t) 

    # 계산된 y(t)를 저장
    y_values.append(y)
    
    # τ(t) 계산
    if len(y_values) > 0:
        # print("\n", y, y_values[-1], y_values[-26])
        tau_t = kappa * y # time delay : 0.2s , simulation period : 0.01s
    else:
        tau_t = 0.0
    tau_values.append(tau_t)

    # data_to_write.append(f"{LPF_pos_L} {LPF_pos_R} {LPF_vel_L} {LPF_vel_R} {tau_t}")
    data_to_write.append(f"{LPF_pos_L*RAD2DEG} {LPF_pos_R*RAD2DEG} {ydot*10} {y*10}")

    t_ = t_ + 0.01
    i = i + 1

    # data.ctrl[2] = tau_t #left
    # data.ctrl[3] = -tau_t # right
    time_list.append(t_)
    desired_positions_1.append(LPF_pos_L*RAD2DEG)                  ;   desired_positions_2.append(LPF_pos_R*RAD2DEG)

output_file_path = '/home/kist/mujoco/mujopy_realdata_grav_nodelay_deg_ydot.txt'
with open(output_file_path, 'w') as output_file:
    for line in data_to_write:
        output_file.write(f"{line}\n")

####
# fs = 1000

# # Generate periodic data (combination of sine waves of 2 Hz and 10 Hz)
# # t = np.linspace(0, 1, fs, endpoint=False)
# # signal_periodic = np.sin(2 * np.pi * 2 * t) + np.sin(2 * np.pi * 10 * t)

# # Perform FFT
# frequencies = np.fft.fftfreq(len(time_list), 1/fs)
# fft_values =  np.fft.fft(desired_positions_1)

# # Visualize results
# plt.figure(figsize=(10, 6))
# plt.subplot(2, 1, 1)
# plt.plot(time_list, desired_positions_1, 'b-')
# plt.title('Periodic Signal (2 Hz and 10 Hz)')

# plt.subplot(2, 1, 2)
# plt.plot(frequencies, np.abs(fft_values), 'r-')
# plt.title('FFT of Periodic Signal')
# plt.xlabel('Frequency [Hz]')
# plt.ylabel('Magnitude')
# plt.grid(True)
# plt.tight_layout()
# plt.show()
####
print(t_)

####
# Fs = 2000
# # Calculate FFT ....................
# y_ = desired_positions_1
# T = 1/Fs
# t = np.arange(0,t_, T)

# n=len(y_)        # Length of signal
# NFFT=n      # ?? NFFT=2^nextpow2(length(y))  ??
# k=np.arange(NFFT)
# f0=k*Fs/NFFT    # double sides frequency range
# f0=f0[range(math.trunc(NFFT/2))]        # single sided frequency range

# Y=np.fft.fft(y_)/NFFT        # fft computing and normaliation
# Y=Y[range(math.trunc(NFFT/2))]          # single sided frequency range
# amplitude_Hz = 2*abs(Y)
# phase_ang = np.angle(Y)*180/np.pi

# figure 1 ..................................
# plt.figure(num=2,dpi=100,facecolor='white')
# plt.subplots_adjust(hspace = 0.6, wspace = 0.3)
# plt.subplot(3,1,1)

# plt.plot(t_,y_,'r')
# plt.title('Signal FFT analysis')
# plt.xlabel('time($sec$)')
# plt.ylabel('y')
#plt.xlim( 0, 0.1)

# Amplitude ....
#plt.figure(num=2,dpi=100,facecolor='white')
# plt.subplot(2,1,1)

# # Plot single-sided amplitude spectrum.

# plt.plot(f0,amplitude_Hz,'r')   #  2* ???
# plt.xticks(np.arange(0,500,20))
# plt.xlim( 0, 200)
# plt.ylim( 0, 1.2)
# #plt.title('Single-Sided Amplitude Spectrum of y(t)')
# plt.xlabel('frequency($Hz$)')
# plt.ylabel('amplitude')
# plt.grid()

# # Phase ....
# #plt.figure(num=2,dpi=100,facecolor='white')
# plt.subplot(2,1,2)
# plt.plot(f0,phase_ang,'r')   #  2* ???
# plt.xlim( 0, 200)
# plt.ylim( -180, 180)
# #plt.title('Single-Sided Phase Spectrum of y(t)')
# plt.xlabel('frequency($Hz$)')
# plt.ylabel('phase($deg.$)')
# plt.xticks([0, 60, 120, 200])
# plt.yticks([-180, -90, 0, 90, 180])
# plt.grid()

# # plt.savefig("./test_figure2.png",dpi=300)
# plt.show()
# ####


# mean = np.mean(LR_points)
# print(mean, len(LR_points))

# threshold = mean



######################## 극댓값 극솟값 평균 구하기 #####################
# # 데이터 리스트
# data_1 = desired_positions_1
# data_2 = desired_positions_2

# # 극댓값과 극솟값을 찾을 리스트
# local_maxima_1 = []
# local_minima_1 = []
# local_maxima_2 = []
# local_minima_2 = []

# # 최대값과 최소값을 저장할 변수
# max_value_1 = -np.inf
# min_value_1 = np.inf
# max_value_2 = -np.inf
# min_value_2 = np.inf

# # 임계값
# threshold = 6.85  # 예시로 6.85로 설정

# # 현재 상태 (극댓값 또는 극솟값)를 추적하는 변수
# current_state_1 = "unknown"
# current_state_2 = "unknown"

# # 데이터를 반복하며 극댓값과 극솟값을 찾음
# for value_1, value_2 in zip(data_1[1200:73000], data_2[1200:73000]):
#     if value_1 >= threshold:
#         if current_state_1 == "increasing":
#             local_maxima_1.append(current_value_1)
#             max_value_1 = max(max_value_1, current_value_1)
#         elif current_state_1 == "decreasing":
#             local_minima_1.append(current_value_1)
#             min_value_1 = min(min_value_1, current_value_1)
#         current_state_1 = "increasing"
#     elif value_1 < threshold:
#         if current_state_1 == "increasing":
#             current_state_1 = "decreasing"
#     current_value_1 = value_1

#     if value_2 >= threshold:
#         if current_state_2 == "increasing":
#             local_maxima_2.append(current_value_2)
#             max_value_2 = max(max_value_2, current_value_2)
#         elif current_state_2 == "decreasing":
#             local_minima_2.append(current_value_2)
#             min_value_2 = min(min_value_2, current_value_2)
#         current_state_2 = "increasing"
#     elif value_2 < threshold:
#         if current_state_2 == "increasing":
#             current_state_2 = "decreasing"
#     current_value_2 = value_2

# # 마지막 원소가 극댓값 또는 극솟값인 경우 처리
# if current_state_1 == "increasing":
#     local_maxima_1.append(current_value_1)
#     max_value_1 = max(max_value_1, current_value_1)
# elif current_state_1 == "decreasing":
#     local_minima_1.append(current_value_1)
#     min_value_1 = min(min_value_1, current_value_1)

# if current_state_2 == "increasing":
#     local_maxima_2.append(current_value_2)
#     max_value_2 = max(max_value_2, current_value_2)
# elif current_state_2 == "decreasing":
#     local_minima_2.append(current_value_2)
#     min_value_2 = min(min_value_2, current_value_2)

# # 극댓값과 극솟값의 평균 계산
# average_maxima_1 = np.mean(local_maxima_1)
# average_minima_1 = np.mean(local_minima_1)
# average_maxima_2 = np.mean(local_maxima_2)
# average_minima_2 = np.mean(local_minima_2)
# min__1 = np.min(local_maxima_1)
# min__2 = np.min(local_maxima_2)

# print("Average Maxima 1:", average_maxima_1)
# print("Average Minima 1:", average_minima_1)
# print("Average Maxima 2:", average_maxima_2)
# print("Average Minima 2:", average_minima_2)

# # 최대값 및 최소값 출력
# print("Max Value 1:", max_value_1)
# print("Min Value 1:", min__1)
# print("Max Value 2:", max_value_2)
# print("Min Value 2:", min__2)
###################################################

#################주기 구하기 ###############
# 데이터 리스트
# data_1 = desired_positions_1[1200:]
# data_2 = desired_positions_2[1200:]

# # 주기 시작, 중간 및 끝 지점을 저장할 리스트
# period_starts_1 = []
# period_mids_1 = []
# period_ends_1 = []

# period_starts_2 = []
# period_mids_2 = []
# period_ends_2 = []

# # 주기의 시작을 나타내는 변수
# period_start_index = 0

# # 주기의 상태를 나타내는 변수
# period_state_1 = "start"
# period_state_2 = "start"

# # threshold 설정
# threshold = 6.85

# for i in range(len(data_1)):
#     value_1 = data_1[i]
#     value_2 = data_2[i]

#     if period_state_1 == "start":
#         if value_1 > threshold:
#             period_starts_1.append(i)
#             period_state_1 = "up"
#     elif period_state_1 == "up":
#         if value_1 < threshold:
#             period_mids_1.append(i)
#             period_state_1 = "down"
#     elif period_state_1 == "down":
#         if value_1 > threshold:
#             period_ends_1.append(i)
#             period_state_1 = "start"

#     if period_state_2 == "start":
#         if value_2 > threshold:
#             period_starts_2.append(i)
#             period_state_2 = "up"
#     elif period_state_2 == "up":
#         if value_2 < threshold:
#             period_mids_2.append(i)
#             period_state_2 = "down"
#     elif period_state_2 == "down":
#         if value_2 > threshold:
#             period_ends_2.append(i)
#             period_state_2 = "start"

# # 주기의 시작, 중간, 끝 인덱스를 사용하여 주기 데이터 추출
# periods_1 = []
# for start, mid, end in zip(period_starts_1, period_mids_1, period_ends_1):
#     period_1 = data_1[start:end+1]
#     periods_1.append(period_1)

# periods_2 = []
# for start, mid, end in zip(period_starts_2, period_mids_2, period_ends_2):
#     period_2 = data_2[start:end+1]
#     periods_2.append(period_2)

# print(periods_1, periods_2)
##########################################


# Create subplots for figure 1 
fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
ax1.plot(time_list, desired_positions_1, label='Left Desired Position')
ax1.plot(time_list, desired_positions_2, label='Right Desired Position')

ax1.set_ylabel('deg')
ax1.legend()

# Plot velocity for joint 1
ax2.plot(time_list, tau_values, label='tau')
# ax2.plot(time_list, actual_velocities_1, label='Actual Velocity 1')

ax2.set_xlabel('Time') ; ax2.set_ylabel('Nm')
ax2.legend()

# plt.show()

