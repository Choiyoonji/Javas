#!/usr/bin/env python
# -- coding: utf-8 --

import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.checks import check_yaml
from ultralytics.yolo.utils import ROOT, yaml_load
import json
import rospy
from geometry_msgs.msg import Point
from valueup_project.msg import ObjPoint

from time import time
from std_msgs.msg import String

WIDTH = 1280
HEIGHT = 720

# model = YOLO('best.pt')
# model = YOLO('best_100.pt')
model = YOLO('best2.pt')
CLASSES = yaml_load(check_yaml('valueup_data.yaml'))['names']
# colors = np.random.uniform(0, 255, size=(len(CLASSES), 3)) # RGB
# colors = np.random.uniform(0, 255, size=(len(CLASSES), 3)) # RGB


class Depth_Camera():

    def __init__(self):
<<<<<<< HEAD
        self.pospub = rospy.Publisher('object_position', ObjPoint, queue_size=1)
=======
        self.tool_sub = rospy.Subscriber('tool_list', String, self.callback_tool, queue_size=1)
        self.pospub = rospy.Publisher('target_position', Point, queue_size=1)
>>>>>>> 933b0c9978e05e927f91c68da187636afb61b3d1
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.align_to = None
        self.detect = False
        self.position = Point()
<<<<<<< HEAD
        
        self.offset = {}
        
        self.offset['bottle'] = Point()
        self.offset['bottle'].x = 0
        self.offset['bottle'].y = 0
        self.offset['bottle'].z = -6
        self.offset['screwdriver'] = Point()
        self.offset['screwdriver'].x = 0
        self.offset['screwdriver'].y = 4
        self.offset['screwdriver'].z = -6
        self.offset['tapemeasure'] = Point()
        self.offset['tapemeasure'].x = 0
        self.offset['tapemeasure'].y = 5
        self.offset['tapemeasure'].z = -6
        self.offset['tape'] = Point()
        self.offset['tape'].x = 0
        self.offset['tape'].y = 0
        self.offset['tape'].z = -6
        self.offset['stapler'] = Point()
        self.offset['stapler'].x = 0
        self.offset['stapler'].y = 6
        self.offset['stapler'].z = -8
        self.offset['wheel'] = Point()
        self.offset['wheel'].x = 0
        self.offset['wheel'].y = 5
        self.offset['wheel'].z = -6
=======
        self.tool = None
>>>>>>> 933b0c9978e05e927f91c68da187636afb61b3d1

        self.translate_vector = np.array([0, 0.2, 0.3])
        self.rotate_degree = -30

        context = rs.context()
        connect_device = None
        if context.devices[0].get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device = context.devices[0].get_info(rs.camera_info.serial_number)

        print(" > Serial number : {}".format(connect_device))
        self.config.enable_device(connect_device)
        self.config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)

    def callback_tool(self, msg):
        self.tool = msg.data

    def __del__(self):
        print("Collecting process is done.\n")
    
    def rotate_x(self,vector, degree):
        # Degree to Radian
        rad = np.radians(degree)
        
        # Rotation Matrix
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(rad), -np.sin(rad)],
            [0, np.sin(rad), np.cos(rad)]
        ])
        
        return np.dot(rotation_matrix, vector)
    
    def translate(self,vector, translation):
        return vector + translation

    def execute(self):
        print('Collecting depth information...')
        
        try:
            self.pipeline.start(self.config)
        except:
            print("There is no signal sended from depth camera.")
            print("Check connection status of camera.")
            return
        
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        try:
            while True:

                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                depth_info = depth_frame.as_depth_frame()

                color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics # 내부 파라미터
                
                color_image = np.asanyarray(color_frame.get_data())
                color_image = cv2.resize(color_image, (WIDTH, HEIGHT)) # 원본으로 resize해야 world 좌표가 잘 나옴
                # 객체 인지도 1280 x 720이 더 잘 됨
                
                results = model(color_image, stream=True)

                class_ids = []
                confidences = []
                bboxes = []
                obj_centers = []

                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        confidence = box.conf
                        if confidence > 0.5 :
                            xyxy = box.xyxy.tolist()[0]
                            bboxes.append(xyxy)
                            confidences.append(float(confidence))
                            class_ids.append(box.cls.tolist())
                            cx = int((xyxy[2]+xyxy[0])//2)
                            cy = int((xyxy[3]+xyxy[1])//2)
                            obj_centers.append([cx,cy]) # 중심
                            

                result_boxes = cv2.dnn.NMSBoxes(bboxes, confidences, 0.25, 0.45, 0.5)
  
                font = cv2.FONT_HERSHEY_PLAIN
                
                for i in range(len(bboxes)):
                    label = str(CLASSES[int(class_ids[i][0])])

                    # if label == 'Screw Driver':
                    if i in result_boxes:
                        obj = ObjPoint()
                        bbox = list(map(int, bboxes[i])) 
                        x, y, x2, y2 = bbox
                        cx, cy = obj_centers[i]
                        
                        print("Depth : ", round((depth_info.get_distance(cx, cy) * 100), 2), "cm")

                        depth = round((depth_info.get_distance(cx, cy) * 100), 2)

                        if depth <= 10:
                            continue
                        
                        wx, wy, wz = rs.rs2_deproject_pixel_to_point(color_intrinsics, [cx, cy], depth)
                        # wx, wy, wz가 real world coordinator
                        # 단위는 cm
                        
                        print(wx, wy, wz)

                        vector = np.array([wx, wz, -wy])

                        # x축을 중심으로 d도 회전
                        rotated_vector = self.rotate_x(vector, self.rotate_degree)

                        translated_vector = self.translate(rotated_vector, self.translate_vector)

                        obj.object_name = label
                        obj.x = translated_vector[0]+self.offset[label].x
                        obj.y = translated_vector[1]+self.offset[label].y
                        obj.z = translated_vector[2]+self.offset[label].z

                        self.pospub.publish(obj)
                        
                        
                    try:
                        color = colors[i]
                        color = (int(color[0]), int(color[1]), int(color[2]), int(color[3]), int(color[4]))
                    except:
                        print("Error")
                    if label != "bottle":
                        color = (0,255,0)
                        cv2.rectangle(color_image, (x, y), (x2, y2), color, 4)
                        cv2.putText(color_image, "{} cm".format(depth), (x + 5, y + 60), 0, 1.0, color, 2)
                        cv2.putText(color_image, label, (x, y + 30), font, 3, color, 3)
                    
                cv2.imshow('image', color_image)
                #cv2.imwrite('./valueup_detect.png', color_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipeline.stop()
        print("┌──────────────────────────────────────┐")
        print('│ Collecting of depth info is stopped. │')
        print("└──────────────────────────────────────┘")

if __name__ == "__main__":
    rospy.init_node('camera')
    depth_camera = Depth_Camera()
    depth_camera.execute()
