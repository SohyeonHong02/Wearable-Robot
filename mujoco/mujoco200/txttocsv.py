import csv

# 텍스트 파일 경로
# text_file = '/home/kist/mujoco/mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0111_with_k8_d015_a05_assist_posvelgait.txt'
text_file = '/home/kist/mujoco/mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0111_with_k8_d025_a05_assist_posvelgait.txt'

# CSV 파일 경로
# csv_file = '/home/kist/mujoco/mujoco/240110_-FE_parameter_result/leg_knee_model/170cm_60kg/0111_with_k8_d015_a05_assist_posvelgait.csv'
csv_file = '/home/kist/mujoco/mujoco/240104_parameter_result/only_leg_model/170cm_60kg/0111_with_k8_d025_a05_assist_posvelgait.csv'

# 텍스트 파일을 CSV 파일로 변환
with open(text_file, 'r') as infile, open(csv_file, 'w', newline='') as outfile:
    # 텍스트 파일 내용을 읽어옵니다.
    lines = infile.readlines()
    
    # CSV 파일 작성기를 생성합니다.
    csv_writer = csv.writer(outfile)
    
    # 각 줄을 쉼표로 구분된 항목으로 분할하여 CSV 파일에 작성합니다.
    for line in lines:
        items = line.strip().split()  # 이것은 공백을 기준으로 분할합니다. 필요에 따라 다른 구분자를 사용할 수 있습니다.
        csv_writer.writerow(items)

print(f'{text_file} 파일이 {csv_file} 파일로 성공적으로 변환되었습니다.')
