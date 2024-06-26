#!/usr/bin/env python
# -- coding: utf-8 --

import os, sys
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk import *
from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk.port_handler import PortHandler
from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk.packet_handler import PacketHandler
from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk.robotis_def import *
from manipulator.msg import *
from manipulator_description import Manipulator
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, String
import numpy as np

import math
import sys, tty, termios

from valueup_project.msg import ObjPoint

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


#*************************AX-12A(PROTOCOL_VERSION 1.0)*****************************#

# Control table address
AX_ADDR_TORQUE_ENABLE          = 24 # 토크 활성화(1)/비활성화(0)
AX_ADDR_CW_COMPLIANCE_MARGIN   = 26 # 시계방향 : Goal Position을 도달했다고 판단하는 Margin값, 예를 들어 Goal Position이 30이고 Margin값이 2라면 28~32에 도달하면 goal position에 도달한것으로 판단함
AX_ADDR_CCW_COMPLIANCE_MARGIN  = 27 # 반시계 방향 : ```
AX_ADDR_CW_COMPLIANCE_SLOPE    = 28 # 시계방향 : 가속/김속하는 시간
AX_ADDR_CCW_COMPLIANCE_SLOPE   = 29 # 반시계방향 : ```
AX_ADDR_GOAL_POSITION          = 30 # 목표 각도
AX_ADDR_MOVING_SPEED           = 32 # 목표 속도
AX_ADDR_PRESENT_POSITION       = 36 # 현재 각도
AX_ADDR_PRESENT_SPEED          = 38 # 현재 속도
AX_ADDR_PRESENT_LOAD           = 40
AX_ADDR_MOVING                 = 46
AX_ADDR_PUNCH                  = 48 # 모터에 가하는 최소 전류 -> 다르게 생각하면 최소 속도라고 할 수 있을 듯

AX_PROTOCOL_VERSION = 1.0

AX_DXL_ID = [5,6,7]

BAUDRATE = 115200

AX_TORQUE_ENABLE = 1
AX_TORQUE_DISABLE = 0

AX_CW_COMPLIANCE_MARGIN = 1 #실제로 설정하려는 값
AX_CCW_COMPLIANCE_MARGIN = 1
AX_CW_COMPLIANCE_SLOPE = 128
AX_CCW_COMPLIANCE_SLOPE = 128

DEVICENAME = '/dev/ttyUSB0'


port_handler = PortHandler(DEVICENAME)
ax_packet_handler = PacketHandler(AX_PROTOCOL_VERSION)

#**********************************************************************************#


#**********************XM430-W350-R(PROTOCOL_VERSION 2.0)**************************#

# Control table address

XM_ADDR_TORQUE_ENABLE           = 64
XM_ADDR_VELOCITY_I_GAIN         = 76
XM_ADDR_VELOCITY_P_GAIN         = 78
XM_ADDR_POTISION_D_GAIN         = 80
XM_ADDR_POSITION_I_GAIN         = 82
XM_ADDR_POSITION_P_GAIN         = 84
XM_ADDR_FEEDFORWARD_2ND_GAIN    = 88
XM_ADDR_FEEDFORWARD_1ST_GAIN    = 90
XM_ADDR_PROFILE_ACCELERATION    = 108
XM_ADDR_PROFILE_VELOCITY        = 112
XM_ADDR_GOAL_POSITION           = 116
XM_ADDR_MOVING                  = 122
XM_ADDR_MOVING_STATUS           = 123
XM_ADDR_PRESENT_POSITION        = 132

XM_PROTOCOL_VERSION_1 = 1.0
XM_PROTOCOL_VERSION_2 = 2.0

XM_DXL_ID_P1 = [3,4]
XM_DXL_ID_P2 = [0,1,2]


XM_TORQUE_ENABLE = 1
XM_TORQUE_DISABLE = 0

xm_packet_handler_p1 = PacketHandler(XM_PROTOCOL_VERSION_1)
xm_packet_handler_p2 = PacketHandler(XM_PROTOCOL_VERSION_2)


MOTOR_VELOCITY = [40, 10, 10, 10, 10, 30, 50, 100]


#**********************************************************************************#

class MotorControlHub:

    def __init__(self):

        self.set_pos = SyncSetPosition()
        self.set_ax_speed = AXSyncSetMovingSpeed()

        self.manipulator = Manipulator()
        
        self.target_position = Point()
        self.previous_position = Point()
        self.save_position = Point()

        # 테스트용 (이후에 지워야 함)
        self.target_position.x = 0
        self.target_position.y = 30
        self.target_position.z = 20
        
        
        # 아래 방향 바라봄
        self.orientation_matrix_1 = [[1, 0, 0],
                                     [0, -1, 0],
                                     [0, 0, -1]]

        # 앞 방향 바라봄 ( 수정 필요할 듯 )
        self.orientation_matrix_2 = [[0, 1, 0],
                                     [0, 0, 1],
                                     [1, 0, 0]]
        
        # self.orientation_matrix_3 = [[math.sin(math.pi / 4), math.cos(math.pi / 4), 0],
        #                              [math.cos(math.pi / 4), math.sin(math.pi / 4), 0],
        #                              [0, 0, 1]]
        
        # self.orientation_matrix_3 = [[0, math.sin(45 * math.pi / 180), -math.cos(45 * math.pi / 180)],
        #                              [0, -math.cos(45 * math.pi / 180), math.sin(45 * math.pi / 180)],
        #                              [1, 0, 0]]
#         cos(45°)  -sin(45°)   0
# sin(45°)   cos(45°)   0
#    0          0       1
        
        self.D = Twist()
        self.U = Twist()
        self.D.linear.x = -0.5
        self.U.linear.x = 0.5

        self.gripper_position = 512
        self.gripper_present_load = 0
        self.gripper_load_check = False

        self.target_position_flag = False
        
        self.target_first_link_flag = False
        
        
        self.set_pos.ax_id = AX_DXL_ID
        self.set_pos.xm_id_p1 = XM_DXL_ID_P1
        self.set_pos.xm_id_p2 = XM_DXL_ID_P2

        self.set_pos.ax_position = [200, 512, 512]
        self.set_pos.xm_position_p1 = [2048,2048]
        self.set_pos.xm_position_p2 = [2048,2048+100-1024,2048-100+1024]#[2048, 2048, 2048, 2048, 2048]

        self.set_ax_speed.id = AX_DXL_ID
        self.set_ax_speed.speed = [80, 80, 80]
        
        self.object_coord = {}
        
        self.object_coord['bottle'] = Point()
        self.object_coord['screwdriver'] = Point()
        self.object_coord['tapemeasure'] = Point()
        self.object_coord['tape'] = Point()
        self.object_coord['stapler'] = Point()
        self.object_coord['wheel'] = Point()
        self.object_coord['door'] = Point()
        
        self.tool = []
        self.door_open = False
        self.hold_flag = False
        
        rospy.Subscriber('object_position', ObjPoint, self.position_callback, queue_size=1)
        rospy.Subscriber('tool_list', String, self.tool_callback, queue_size=1)

        rospy.Subscriber('set_position',SyncSetPosition, self.set_goal_pos_callback, queue_size=1)

        self.linear_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pos_pub = rospy.Publisher('present_position', SyncSetPosition, queue_size=1)
        self.ax_speed_pub = rospy.Publisher('present_ax_speed', AXSyncSetMovingSpeed, queue_size=1)

    def position_callback(self, data:ObjPoint):
        self.object_coord[data.object_name].x = data.x
        self.object_coord[data.object_name].y = data.y
        self.object_coord[data.object_name].z = data.z
        
    def tool_callback(self, data:String):
        self.tool = data.data.split(' ')
        self.tool_position_list = []
        for tool in self.tool:
            p = Point()
            p.x = self.object_coord[tool].x
            p.y = self.object_coord[tool].y
            p.z = self.object_coord[tool].z
            self.tool_position_list.append(p)
        
        for i, tool in enumerate(self.tool):
            if tool == 'door':
                self.door_open = True
            self.set_target_position_tool(i)
            self.door_open = False

    def point_distance(self, p1:Point, p2:Point):
        return ((p1.x-p2.x)**2+(p1.y-p2.y)**2+(p1.z-p2.z)**2)**(1/2)

    def set_target_position_tool(self, i):
        self.target_first_link_flag = True

        self.target_position.x = self.tool_position_list[i].x
        self.target_position.y = self.tool_position_list[i].y
        self.target_position.z = self.tool_position_list[i].z

        print(self.target_position)

        if self.door_open:
            self.target_position.y -= 20
            self.set_target_position()
            rospy.Rate(0.2).sleep()
            ############################# 문 손잡이로 접근 ####################################
            self.target_position.y += 10
            self.target_position.z += 3
            self.set_target_position()
            rospy.Rate(0.2).sleep()
            ############################## 좀만 더 접근 ######################################
            # for i in range()
            self.target_position.y += 4
            self.target_position.z += 1.0
            self.set_target_position()
            rospy.Rate(0.8).sleep()
            ############################# 마지막으로 접근 #####################################
            self.target_position.y += 4
            # self.target_position.z += 0.5
            self.set_target_position()
            rospy.Rate(0.8).sleep()
            ############################### 그리퍼 잡기 ######################################
            self.gripper_load_check = False
            self.gripper_position = 0
            self.set_target_position()
            rospy.Rate(0.3).sleep()
            ################################# 비틀기 ########################################
            self.hold_flag = True 
            self.set_target_position()
            rospy.Rate(1).sleep()
            # ############################# 비튼 채로 뒤로 ####################################
            self.target_position.y -= 5
            self.set_target_position()
            rospy.Rate(1).sleep()
            # ############################ 비튼거 다시 원위치 ##################################
            self.hold_flag = False
            self.set_target_position()
            rospy.Rate(0.5).sleep()
            # ####################### 아직까진 잡은 채로 좀 더 뒤로 ##############################
            self.target_position.y -= 5
            self.set_target_position()
            rospy.Rate(0.5).sleep()
            ################################ 잡은거 풀기 ######################################
            self.gripper_position = 512
            self.set_target_position()
            rospy.Rate(0.8).sleep()
            # ############################ 푼 채로 조금 뒤로 ###################################
            self.target_position.y -= 5
            self.set_target_position()
            rospy.Rate(1).sleep()
            ############################## 오른쪽으로 확 틀기 ##################################
            self.target_position.x += 19
            self.set_target_position()
            rospy.Rate(1.5).sleep()
            # ############################# 앞으로 많이 접근 ###################################
            self.target_position.y += 15
            self.set_target_position()
            rospy.Rate(0.2).sleep()
            # ###################### 왼쪽으로 조금 틀어서 문까지 접합 #############################
            self.target_position.x -= 8
            self.set_target_position()
            rospy.Rate(0.8).sleep()
            # ############################### 문 확 열기 ######################################
            self.target_position.x -= 11
            self.target_position.y -= 9
            self.set_target_position()
            rospy.Rate(0.3).sleep()
            # ####################### task를 끝내며 초기 위치로 이동 ##############################
            self.target_position.y -= 10
            self.set_target_position()
            rospy.Rate(0.3).sleep()

        else:
            self.target_position.z += 10

            self.set_target_position()
            rospy.Rate(0.2).sleep()

            ###############################################################################

            self.target_position.z -= 7


            self.set_target_position()
            rospy.Rate(0.2).sleep()

            ###############################################################################

            self.gripper_load_check = False
            self.gripper_position = 0
            self.set_target_position()
            rospy.Rate(0.3).sleep()

            ################################################################################

            self.target_position.z += 15


            self.set_target_position()
            rospy.Rate(0.2).sleep()

            ###############################################################################

            self.target_first_link_flag = False

            self.target_position.x = -20
            self.target_position.y = -5
            self.target_position.z = 0


            self.set_target_position()
            rospy.Rate(0.2).sleep()

            ##############################################################################

            self.linear_pub.publish(self.D)
            rospy.Rate(0.5).sleep()

            ##############################################################################

            self.gripper_position = 512
            self.set_target_position()
            rospy.Rate(0.8).sleep()

            ##############################################################################

            self.linear_pub.publish(self.U)
            rospy.Rate(0.8).sleep()


    def set_goal_pos_callback(self,data):
        self.set_pos = data


    def set_goal_pos(self,data:SyncSetPosition):
        for idx in range(len(data.ax_id)):
            # print("Set Goal AX_Position of ID %s = %s" % (data.ax_id[idx], data.ax_position[idx]))
            ax_packet_handler.write2ByteTxRx(port_handler,data.ax_id[idx], AX_ADDR_GOAL_POSITION, data.ax_position[idx])

        for idx in range(len(data.xm_id_p1)):
            # print("Set Goal XM_Position of ID %s = %s" % (data.xm_id[idx], data.xm_position[idx]))
            xm_packet_handler_p1.write4ByteTxRx(port_handler,data.xm_id_p1[idx], XM_ADDR_GOAL_POSITION, data.xm_position_p1[idx])

        for idx in range(len(data.xm_id_p2)):
            # print("Set Goal XM_Position of ID %s = %s" % (data.xm_id[idx], data.xm_position[idx]))
            if idx == 0 and self.target_first_link_flag is True:
                xm_packet_handler_p2.write4ByteTxRx(port_handler,data.xm_id_p2[idx], XM_ADDR_GOAL_POSITION, data.xm_position_p2[idx]-1024)
                continue
            xm_packet_handler_p2.write4ByteTxRx(port_handler,data.xm_id_p2[idx], XM_ADDR_GOAL_POSITION, data.xm_position_p2[idx])



    def present_position_callback(self):
        present_position = SyncSetPosition()
        present_position.ax_id = AX_DXL_ID
        present_position.xm_id_p1 = XM_DXL_ID_P1
        present_position.xm_position_p2= XM_DXL_ID_P2
        present_position.ax_position = []
        present_position.xm_position_p1 = []
        present_position.xm_position_p2 = []

        for id in AX_DXL_ID:
            dxl_present_position, dxl_comm_result, dxl_error = ax_packet_handler.read2ByteTxRx(port_handler, id, AX_ADDR_PRESENT_POSITION)
            present_position.ax_position.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return
            
        for id in XM_DXL_ID_P1:
            dxl_present_position, dxl_comm_result, dxl_error = xm_packet_handler_p1.read4ByteTxRx(port_handler, id, XM_ADDR_PRESENT_POSITION)
            present_position.xm_position_p1.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return

        for id in XM_DXL_ID_P2:
            dxl_present_position, dxl_comm_result, dxl_error = xm_packet_handler_p2.read4ByteTxRx(port_handler, id, XM_ADDR_PRESENT_POSITION)
            present_position.xm_position_p2.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return
            
        self.gripper_present_load, dxl_comm_result, dxl_error = ax_packet_handler.read2ByteTxRx(port_handler, AX_DXL_ID[2], AX_ADDR_PRESENT_LOAD)
        if(dxl_comm_result != COMM_SUCCESS) :
            return
        if(dxl_error != 0) :
            return
        self.gripper_present_load %= 1024
        self.pos_pub.publish(present_position)

    
    def set_target_position(self):
        target_pos = [self.target_position.x, self.target_position.y, self.target_position.z]

        target_y_check = False
        if self.target_position.y < 0:
            target_pos[0] *= -1
            target_pos[1] *= -1
            print("check: ", target_pos)
            target_y_check = True

        if self.door_open:
            motor_angles = self.manipulator.manipulator_link.inverse_kinematics(target_position=target_pos,
                                                                                target_orientation=self.orientation_matrix_2,
                                                                                orientation_mode="all")
        else:
            motor_angles = self.manipulator.manipulator_link.inverse_kinematics(target_position=target_pos,
                                                                                target_orientation=self.orientation_matrix_1,
                                                                                orientation_mode="all")

        if self.check_inverse_kinematics(target_pos,motor_angles) is False:
            print("도달할 수 없는 타겟")
            return
        
        if target_y_check is True:
            motor_angles[1] -= math.pi
            if motor_angles[1] < -math.pi:
                motor_angles[1] += 2*math.pi
                
        if self.hold_flag:
            motor_angles[6] -= math.pi / 2
            
        motor_angles = np.append(np.array(motor_angles[1:7]), [self.gripper_position])

        self.manipulator.set_position(motor_angles)
    
    def check_inverse_kinematics(self, target_position, motor_angles):
        return self.distance_target_point(np.transpose(np.array(self.manipulator.manipulator_link.forward_kinematics(motor_angles)[:3,3:]))[0], target_position)


    def distance_target_point(self, p1, p2):
        distance = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)**(1/2)
        # print("distance: ",distance)
        if distance > 1:
            return False
        else:
            return True



def main():
    rospy.init_node('motor_control_hub')
 

    try:
        port_handler.openPort()
        print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    try:
        port_handler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    for id in AX_DXL_ID :
        dxl_comm_result, dxl_error = ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % ax_packet_handler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % ax_packet_handler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print(f"DYNAMIXEL(ID : {id}) has been successfully connected")
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CW_COMPLIANCE_MARGIN, AX_CW_COMPLIANCE_MARGIN) #초기 margin 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CCW_COMPLIANCE_MARGIN, AX_CCW_COMPLIANCE_MARGIN) #초기 margin 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CW_COMPLIANCE_SLOPE, AX_CW_COMPLIANCE_SLOPE) #초기 slope 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CCW_COMPLIANCE_SLOPE, AX_CCW_COMPLIANCE_SLOPE) #초기 slope 설정
            ax_packet_handler.write2ByteTxRx(port_handler, id, AX_ADDR_MOVING_SPEED, MOTOR_VELOCITY[id]) #초기 속도 설정

    for id in XM_DXL_ID_P1 :
        dxl_comm_result, dxl_error = xm_packet_handler_p1.write1ByteTxRx(port_handler, id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % xm_packet_handler_p1.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % xm_packet_handler_p1.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            xm_packet_handler_p1.write4ByteTxRx(port_handler, id, XM_ADDR_PROFILE_VELOCITY, MOTOR_VELOCITY[id])
        print(f"DYNAMIXEL(ID : {id}) has been successfully connected")

    for id in XM_DXL_ID_P2 :
        dxl_comm_result, dxl_error = xm_packet_handler_p2.write1ByteTxRx(port_handler, id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % xm_packet_handler_p2.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % xm_packet_handler_p2.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            xm_packet_handler_p2.write4ByteTxRx(port_handler, id, XM_ADDR_PROFILE_VELOCITY, MOTOR_VELOCITY[id])
        print(f"DYNAMIXEL(ID : {id}) has been successfully connected")

    
    print("Ready to get & set Position.")

    ############################################################################################################
    #  여기까지는 dynamixel 기본 설정
    ############################################################################################################


    data_hub = MotorControlHub()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        data_hub.set_goal_pos(data_hub.set_pos)

        data_hub.present_position_callback()

        rate.sleep()

        if(data_hub.gripper_present_load > 300 and data_hub.gripper_load_check is False) :
            print("dasdas:", data_hub.gripper_present_load, data_hub.manipulator.ax_position[2])
            data_hub.gripper_position = data_hub.manipulator.ax_position[2]-30
            data_hub.set_target_position()
            data_hub.gripper_load_check = True



if __name__ == '__main__':
    main()
