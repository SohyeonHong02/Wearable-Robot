import matplotlib.pyplot as plt
import csv

#------------ only leg model -------------------
# csv_file_path1  =  'mujoco/240104_parameter_result/only_leg_model/158cm_50kg/0104_1_with_k5_d025_a05_assist.txt'   # 데이터 값 불러오기
# csv_file_path2 =  'mujoco/240104_parameter_result/only_leg_model/158cm_50kg/0105_with_k5_d03_a05_assist.txt'   # 데이터 값 불러오기 
# csv_file_path1 =  'mujoco/240104_parameter_result/only_leg_model/158cm_50kg/with_k8_025_assist_0104_1.txt'   # 데이터 값 불러오기 

# csv_file_path1 =  'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0104_1_without_assist_a05.txt'   # 데이터 값 불러오기 
# csv_file_path2 =  'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0104_1_with_k8_d025_a05_assist.txt'   # 데이터 값 불러오기 
# csv_file_path1 =  'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0123_without_assist_a05.txt'   # 데이터 값 불러오기 
# csv_file_path2  =  'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0123_with_k8_d035_a05_assist.txt'   # 데이터 값 불러오기

# csv_file_path2  =  'mujoco/240104_parameter_result/only_leg_model/180cm_72kg/0105_with_k8_d03_a05_assist.txt'   # 데이터 값 불러오기
# csv_file_path1 =  'mujoco/240104_parameter_result/only_leg_model/180cm_72kg/0104_1_with_k8_d025_a05_assist.txt'   # 데이터 값 불러오기 
# csv_file_path2 =  'mujoco/240104_parameter_result/only_leg_model/180cm_72kg/0105_with_k8_d025_a1_assist.txt'   # 데이터 값 불러오기 


# csv_file_path1 =  'before_without_assist'   # 데이터 값 불러오기 
# csv_file_path2 =  'before_with_assist'   # 데이터 값 불러오기 


# csv_file_path1 = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0111_without_a05_assist_posvelgait.txt'
# csv_file_path2 = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0105_with_k8_d025_a05_assist.txt'
# csv_file_path2 = 'mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0105_with_k8_d035_a05_assist.txt'


#------------ only leg model -------------------
# csv_file_path2 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0123_with_k1.8_66%_a05_assist_posdotgaitphase.txt'
# csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0105_with_k8_d025_a05_assist.txt'
# csv_file_path2 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0105_with_k8_d035_a05_assist.txt'

# csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0111_without_a05_assist_posvelgait.txt'
csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0116_with_k8_d0_a05_assist_posvelgait_arrangegaitphase.txt'
csv_file_path2 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0116_with_k8_d015_a05_assist_posvelgait_arrangegaitphase.txt'
# csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0111_with_k8_d02_a05_assist_posvelgait.txt'


# csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/180cm_72kg/0111_without_a05_assist_posvelgait.txt'
# csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/180cm_72kg/0111_with_k8_d01_a05_assist_posvelgait.txt'
# csv_file_path2 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/180cm_72kg/0111_with_k12_d015_a05_assist_posvelgait.txt'


# csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/158cm_50kg/0111_without_a05_assist_posvelgait.txt'
# csv_file_path1 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/158cm_50kg/0111_with_k4_d015_a05_assist_posvelgait.txt'
# csv_file_path2 = 'mujoco/240110_-FE_parameter_result/leg_knee_model/158cm_50kg/0111_with_k8_d015_a05_assist_posvelgait.txt'



# 데이터를 저장할 리스트
without_left_leg = []
without_right_leg = []
without_left_robot = []
without_right_robot = []
with_left_leg = []
with_right_leg = []
with_left_robot = []
with_right_robot = []

with_left_pos = []
with_right_pos = []
with_left_vel = []
with_right_vel = []
with_gaitphase_L = []
with_gaitphase_R = []

without_left_pos = []
without_right_pos = []
without_left_vel = []
without_right_vel = []
without_gaitphase_L = []
without_gaitphase_R = []
time_data1=[]
time_data2=[]
time_data3=[]


with open(csv_file_path1, "r") as file1:
    for line1 in file1:
        # 각 줄을 띄어쓰기를 기준으로 분할
        parts1 = line1.split()
        
        # 데이터가 5개 이상인 경우에만 처리
        if len(parts1) >= 7:
            # 각 변수에 데이터 추가 (타입 변환을 원한다면 float(parts[i]) 사용)
            time_data1.append(int(parts1[0])*0.0166)
            without_left_leg.append(float(parts1[1]))
            without_right_leg.append(float(parts1[2]))
            without_left_robot.append(float(parts1[3]))
            without_right_robot.append(float(parts1[4]))
            without_left_pos.append(float(parts1[5]))
            without_right_pos.append(float(parts1[6]))
            without_left_vel.append(float(parts1[7]))
            without_right_vel.append(float(parts1[8]))
            without_gaitphase_L.append(float(parts1[9]))
            without_gaitphase_R.append(float(parts1[10])/10)

with open(csv_file_path2, "r") as file2:
    for line2 in file2:
        # 각 줄을 띄어쓰기를 기준으로 분할
        parts2 = line2.split()
        
        # 데이터가 5개 이상인 경우에만 처리
        if len(parts2) >= 7:
            # 각 변수에 데이터 추가 (타입 변환을 원한다면 float(parts[i]) 사용)
            time_data2.append(float(parts2[0])*0.0166)
            with_left_leg.append(float(parts2[1]))
            with_right_leg.append(float(parts2[2]))
            with_left_robot.append(float(parts2[3]))
            with_right_robot.append(float(parts2[4]))
            with_left_pos.append(float(parts2[5]))
            with_right_pos.append(float(parts2[6]))
            with_left_vel.append(float(parts2[7]))
            with_right_vel.append(float(parts2[8]))
            with_gaitphase_L.append(float(parts2[9]))
            with_gaitphase_R.append(float(parts2[10]))


# with open(csv_file_path3, "r") as file3:
#     for line3 in file3:
#         # 각 줄을 띄어쓰기를 기준으로 분할
#         parts3 = line3.split()
        
#         # 데이터가 5개 이상인 경우에만 처리
#         if len(parts3) >= 5:
#             # 각 변수에 데이터 추가 (타입 변환을 원한다면 float(parts[i]) 사용)
#             time_data3.append(int(parts3[0]))
#             with20_left_leg.append(float(parts3[1]))
#             with20_right_leg.append(float(parts3[2]))
#             with20_left_robot.append(float(parts3[3]))
#             with20_right_robot.append(float(parts3[4]))


            
# 결과 출력 (선택 사항)
# for i in range(len(time_data)):
#     print(f"Year: {time_data[i]}, Var1: {without_left_leg[i]}, Var2: {without_right_leg[i]}, Var3: {without_left_robot[i]}, Var4: {without_right_robot[i]}")

####### graph plotting ############################################################################

# Create subplots for figure 1 
fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

ax1.set_title('170cm 60kg')
# ax1.legend()
# Figure 1 - Joint 1
# Plot position for joint 1

ax1.plot(time_data1, without_left_leg, label='L_leg Torque (DOFC delay = 0.15)')
ax1.plot(time_data2, with_left_leg, label='L_leg Torque (gait phase estimation)')

# ax1.plot(time_data1, without_left_pos, label='L_leg pos (gait pattern A)')
# ax1.plot(time_data2, with_left_pos, label='L_leg pos')
# # ax1.plot(time_data2, with_left_pos, label='with_left_pos')
# ax1.plot(time_data2, with_left_vel, label='left vel (gait phase estimation)')
# ax1.plot(time_data2, without_left_robot, label='without_left_robot')
# ax1.plot(time_data2, with_left_robot, label='L_robot Torque')
# ax1.plot(time_data2, with_gaitphase_L, label='with_gaitphase_L')


ax1.set_xlabel('Time')
# ax1.set_ylabel('Torque (N.m)')
ax1.legend()

# Plot velocity for joint 1

# ax2.plot(time_data1, without_right_leg, label='R_leg Torque (DOFC delay = 0.15)')
# ax2.plot(time_data2, with_right_leg, label='R_leg Torque (gait phase estimation)')

# ax2.plot(time_data1, without_right_pos, label='R_leg pos')
ax2.plot(time_data2, with_right_pos, label='R_leg pos')

# # ax2.plot(time_data2, with_right_pos, label='with_right_pos')
# ax2.plot(time_data2, with_right_vel, label='with_right_vel')
ax2.plot(time_data2, without_right_robot, label='R_robot Torque (DOFC delay = 0)')
ax2.plot(time_data2, with_right_robot, label='R_robot Torque (DOFC delay = 0.15)')
ax2.plot(time_data2, without_gaitphase_R, label='with_gaitphase_R')


ax2.set_xlabel('Time')
# ax2.set_ylabel('Torque (N.m)')
ax2.legend()

plt.show()

# # ---------------------------------------------------------------------
# Create subplots for figure 2
# fig2, (ax3, ax4) = plt.subplots(2, 1, sharex=True)

# # Figure 1 - Joint 1
# # Plot position for joint 1
# ax3.plot(time_data2, without_left_robot, label='L_Robot Torque without assist')
# ax3.plot(time_data2, with_left_robot, label='L_Robot Torque with assist')

# ax3.set_xlabel('Time (sec)') ; ax3.set_ylabel('Torque (N.m)')
# ax3.legend()

# # Plot velocity for joint 1
# ax4.plot(time_data2, without_right_robot, label='R_Robot Torque without assist')
# ax4.plot(time_data2, with_right_robot, label='R_Robot Torque with assist')

# ax4.set_xlabel('Time (sec)') ; ax4.set_ylabel('Torque (N.m)')
# ax4.legend()

# # ----------------------------------------------------------------------------
# # Create subplots for figure 3
# fig3, (ax5, ax6) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
# ax5.plot(time_data1, without_left_leg, label='L_leg Torque without assist')
# ax5.plot(time_data2, with_left_leg, label='L_leg Torque with assist')
# ax5.plot(time_data2, without_left_robot, label='L_Robot Torque without assist')
# ax5.plot(time_data2, with_left_robot, label='L_Robot Torque with assist')
# ax5.plot(time_data2, without_left_pos, label='L_Robot Pos without assist')
# ax5.plot(time_data2, with_left_pos, label='L_Robot Pos with assist')


# ax5.set_xlabel('Time (sec)') ; ax5.set_ylabel('Torque (N.m)')
# ax5.legend()

# # Plot velocity for joint 1
# ax6.plot(time_data1, without_right_leg, label='R_leg Torque without assist')
# ax6.plot(time_data2, with_right_leg, label='R_leg Torque with assist')
# ax6.plot(time_data2, without_right_robot, label='R_Robot Torque without assist')
# ax6.plot(time_data2, with_right_robot, label='R_Robot Torque with assist')
# ax6.plot(time_data2, without_right_pos, label='R_Robot pos without assist')
# ax6.plot(time_data2, with_right_pos, label='R_Robot pos with assist')

# ax6.plot(time_data2, with_left_pos, label='L_Robot Pos with assist')



# ax6.set_xlabel('Time (sec)') ; ax6.set_ylabel('Torque (N.m)')
# ax6.legend()

# ----------------------------------------------------------------------------
# Create subplots for figure 3
fig4, (ax7, ax8) = plt.subplots(2, 1, sharex=True)

# Figure 1 - Joint 1
# Plot position for joint 1
# ax7.plot(time_data2, without_left_pos, label='without_left_pos')
# ax7.plot(time_data2, without_left_robot, label='L_Robot Torque without assist')
# ax7.plot(time_data2, with_left_robot, label='L_Robot Torque with assist')

# ax7.set_xlabel('Time (sec)') ; ax7.set_ylabel('Torque (N.m)')
# ax7.legend()

# # Plot velocity for joint 1
# ax8.plot(time_data2, without_right_pos, label='without_right_pos')
# ax8.plot(time_data2, without_right_robot, label='R_Robot Torque without assist')
# ax8.plot(time_data2, with_right_robot, label='R_Robot Torque with assist')

# ax7.plot(time_data1, without_left_pos, label='without_left_pos')
# # ax7.plot(time_data1, without_right_pos, label='without_right_pos')
# ax7.plot(time_data1, without_left_robot, label='without_left_robot')
# ax7.plot(time_data1, without_gaitphase_L, label='without_gaitphase_L')
# ax7.plot(time_data2, with_left_robot, label='L_Robot Torque with assist')

ax7.set_xlabel('Time (sec)') ; ax7.set_ylabel('Torque (N.m)')
ax7.legend()

# Plot velocity for joint 1
# ax8.plot(time_data2, with_left_pos, label='with_left_pos')
# ax8.plot(time_data2, with_left_robot, label='with_left_robot')
# ax8.plot(time_data2, without_left_robot, label='without_left_robot')
# ax8.plot(time_data2, with_gaitphase_L, label='with_gaitphase_L')
# ax8.plot(time_data2, with_right_pos, label='with_right_pos')
# ax8.plot(time_data2, with_right_robot, label='with_left_robot')
# ax8.plot(time_data2, with_gaitphase_R, label='with_gaitphase_R')

# ax8.plot(time_data2, without_left_pos, label='without_left_pos')
# ax8.plot(time_data2, without_left_robot, label='without_left_robot')
# ax8.plot(time_data2, without_gaitphase_L, label='without_gaitphase_L')
# ax8.plot(time_data2, with_left_pos, label='with_left_pos')
# ax8.plot(time_data2, with_left_robot, label='with_left_robot')
# ax8.plot(time_data2, with_gaitphase_L, label='with_gaitphase_L')



ax8.set_xlabel('Time (sec)') ; ax8.set_ylabel('Torque (N.m)')
ax8.legend()



# Show the plot for figure 2
