import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

# XML 파일 경로
xml_file_path = '/home/kist/mujoco/mujoco200/model/inverted/copy_InvertedPendulum.xml'

# XML 파일 파싱
tree = ET.parse(xml_file_path)
root = tree.getroot()

# 원하는 위치 값을 저장할 리스트
time_steps = 100  # 시간 스텝 수
time = np.linspace(0, 2*np.pi, time_steps)  # 0부터 2*pi까지 시간 범위 생성
desired_positions = np.sin(time)  # sin 함수를 사용하여 원하는 위치 생성

# 제어 에러 계산
errors = [desired_positions[i] - desired_positions[i-1] for i in range(1, len(desired_positions))]

# 시간에 따른 제어 에러 플롯 생성
time = range(1, len(errors)+1)  # 시간 설정
plt.plot(time, errors)
plt.xlabel('Time')
plt.ylabel('Control Error')
plt.title('Control Error over Time')
plt.show()