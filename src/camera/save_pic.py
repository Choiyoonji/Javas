import cv2
import os
import pyrealsense2 as rs
import numpy as np

# 폴더 생성
folder_name = 'a'
if not os.path.exists(folder_name):
    os.makedirs(folder_name)

# 리얼센스 카메라 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

profile = pipeline.start(config)

frame_count = 0

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        cv2.imshow('RealSense Camera', color_image)

        key = cv2.waitKey(1)
        if key == ord(' '):  # 스페이스바를 누르면 이미지 저장
            frame_count += 1
            filename = os.path.join(folder_name, f'frame_{frame_count:04d}.jpg')
            resized_frame = cv2.resize(color_image, (1280, 720))
            cv2.imwrite(filename, resized_frame)
            print(f'Saved {filename}')

        elif key == 27:  # ESC를 누르면 종료
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
