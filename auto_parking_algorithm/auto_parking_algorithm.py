#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : ACDC
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
from math import pi, sin, cos, atan2, sqrt, acos, inf, degrees, radians
import rospy
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#=============================================
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
rx, ry, rd = [], [], [] # 경로를 저장할 리스트
steer_dict = {'L': 1, 'R': -1, 'S': 0 }  # dubins 방향을 결정하는 딕셔너리
target_index = 0

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
W = 64  # [pixel] 차량의 너비
L = 128  # [pixel] 차량의 길이
WB = 84  # [pixel] 차량의 축 거리 - 전륜과 후륜의 중간(휠 베이스)
MAX_STEER = 20 # [degree] 최대 조향각
MIN_RADIUS = 250  # [pixel] 최소 회전 반지름
STEP_SIZE = 1  # 경로 생성 시 이동 거리

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
# syaw is the angle of the car in degrees starting from south and increasing counter clockwise
# dubins angle is the angle of the car in degrees starting from east and increasing counter clockwise
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry, rd, target_index

    print("Start Planning")
    target_index = 0
    rx, ry, rd = [], [], []
    forward_start = np.array([[sx], [sy], [radians(syaw+90)]])
    entry = np.array([[P_ENTRY[0]], [P_ENTRY[1]], [radians(-45)]])
    end = np.array([[P_END[0]], [P_END[1]], [radians(-45)]])

    path1 = generate_dubins_path(forward_start, entry, MIN_RADIUS, STEP_SIZE)
    path2 = generate_dubins_path(entry, end, MIN_RADIUS, STEP_SIZE)
    path1_points = [point for point in path1]
    path2_points = [point for point in path2]
    rx, ry = [point[0] for point in path1_points + path2_points], [point[1] for point in path1_points + path2_points]
    rd = [1] * len(rx)

    if not (all(0 <= x <= 1200 for x in rx) and all(0 <= y <= 800 for y in ry)):
        print("Forward path is out of bounds")
        print("Planning backward path")
        rx, ry, rd = [], [], []
        backward_start = np.array([[sx], [sy], [radians(syaw+270)]])
        layover = np.array([[P_ENTRY[0]-300], [P_ENTRY[1]+300], [radians(135)]])
        path1 = generate_dubins_path(backward_start, layover, MIN_RADIUS, STEP_SIZE)
        rx, ry = [point[0] for point in path1], [point[1] for point in path1]
        rd = [-1] * len(rx)
        layover[2] = -45
        path2 = generate_dubins_path(layover, entry, MIN_RADIUS, STEP_SIZE)
        path3 = generate_dubins_path(entry, end, MIN_RADIUS, STEP_SIZE)
        for point in path2 + path3:
            rx.append(point[0])
            ry.append(point[1])
        rd.extend([1] * len(path2 + path3))

    else:
        print("Forward path planned successfully")

    print("End Planning")
    return rx, ry

def generate_dubins_path(start=np.zeros((3, 1)), end=np.ones((3, 1)), min_radius=1, step_size=1, **kwargs):
    diff = end[0:2] - start[0:2]
    dis = np.linalg.norm(diff)
    radian = atan2(diff[1, 0], diff[0, 0])
    d = dis / min_radius
    start_theta = start[2, 0]
    goal_theta = end[2, 0]
    alpha = (start_theta - radian) % (2*pi)
    beta = (goal_theta - radian) % (2*pi)
    admissible_path = [dubins_LSL, dubins_RSR, dubins_RSL, dubins_LSR, dubins_RLR, dubins_LRL] # the candidate curves

    min_length = inf

    for ap in admissible_path:
        # find the shortest path
        t, p, q, mode = ap(alpha, beta, d)

        if t == None:
            continue

        total_length = abs(t) + abs(p) + abs(q)
        if total_length < min_length:
            min_length = total_length
            sh_t = t
            sh_p = p
            sh_q = q
            sh_mode = mode

    path_point_list = path_generate(start, sh_t, sh_p, sh_q, sh_mode, min_radius, step_size)  # generate the path points

    return path_point_list

def path_generate(start, t, p, q, mode, min_radius, step_size):
    # generate a series points from the (t, p, q) set by the mode

    length = [t, p, q]

    trajectory = [start]
    init_point = start

    for i in range(3):
        if length[i] != 0:
            point_list, end_point = element_sample(length[i], init_point, mode[i], min_radius, step_size)
            trajectory = trajectory + point_list
            init_point = end_point

    return trajectory

def element_sample(length, start_point, steer_mode, min_radius, step_size):
    # generate a series points from the an element
    steer = steer_dict[steer_mode]
    real_length = length * min_radius

    # generate a series of sample points
    cur_length = 0
    path_list = []

    while cur_length <= real_length - step_size:
        cur_length = cur_length + step_size
        next_pose = trans_pose(start_point, cur_length, steer, min_radius)
        path_list.append(next_pose)

    end_pose = trans_pose(start_point, real_length, steer, min_radius)

    # add the end pose under current transform
    temp = len(path_list)

    if len(path_list) != 0:
        if np.linalg.norm(end_pose - path_list[-1]) <= 0.01:
            path_list[-1] = end_pose
        else:
            path_list.append(end_pose)
    else:
        path_list.append(end_pose)

    return path_list, end_pose

def trans_pose(start_pose, length, steer, min_radius):
    # pose: x, y, theta, 3*1
    # rotate the start pose with a rot_theta by a center point

    cur_theta = start_pose[2, 0]
    start_position = start_pose[0:2]
    rot_theta = steer * length / min_radius

    if rot_theta == 0:
        trans_matrix = length * np.array([ [cos(cur_theta)], [sin(cur_theta)], [0]])
        end_pose = start_pose + trans_matrix
    else:
        center_theta = cur_theta + steer*pi/2
        center_position = start_position + min_radius * np.array([ [cos(center_theta)], [sin(center_theta)]])

        rot_matrix = np.array([[cos(rot_theta), -sin(rot_theta)], [sin(rot_theta), cos(rot_theta)]])
        end_position = center_position + rot_matrix @ (start_position - center_position)
        end_theta = (cur_theta + rot_theta)
        while end_theta > pi:
            end_theta = end_theta - 2 * pi
        while end_theta < -pi:
            end_theta = end_theta + 2 * pi
        end_pose = np.vstack((end_position, [end_theta]))

    return end_pose

def dubins_LSL(alpha, beta, d):
    mode = ['L', 'S', 'L']
    temp0 = atan2(cos(beta)- cos(alpha), d+sin(alpha)-sin(beta))
    t = (-alpha + temp0) % (2*pi)
    temp1 = 2 + d**2 - 2*cos(alpha-beta) + 2 * d * (sin(alpha) - sin(beta))
    if temp1 < 0:
        return None, None, None, mode
    p = sqrt(temp1)
    q = (beta - temp0) % (2*pi)
    return t, p, q, mode

def dubins_RSR(alpha, beta, d):
    mode = ['R', 'S', 'R']
    temp0 = atan2(cos(alpha)- cos(beta), d-sin(alpha)+sin(beta))
    t = (alpha - temp0) % (2*pi)
    temp1 = 2 + d**2 - 2*cos(alpha-beta) + 2*d*(sin(beta)-sin(alpha))
    if temp1 < 0:
        return None, None, None, mode
    p = sqrt(temp1)
    q = (-beta % (2*pi) + temp0) % (2*pi)
    return t, p, q, mode

def dubins_LSR(alpha, beta, d):
    mode=['L', 'S', 'R']
    temp0 = -2 + d**2 + 2*cos(alpha-beta) + 2*d*(sin(alpha) + sin(beta) )
    if temp0 < 0:
        return None, None, None, mode
    p = sqrt(temp0)
    temp1 = atan2( (-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)) )
    t = (-alpha+temp1 - atan2(-2, p)) % (2*pi)
    q = (-beta % (2*pi)+temp1-atan2(-2, p)) % (2*pi)
    return t, p, q, mode

def dubins_RSL(alpha, beta, d):
    mode = ['R', 'S', 'L']
    temp0 = d**2-2+2*cos(alpha-beta)-2*d*(sin(alpha)+sin(beta))
    if temp0 < 0:
        return None, None, None, mode
    p = sqrt(temp0)
    temp1 = atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta) ) )
    t = (alpha - temp1 + atan2(2, p)) % (2*pi)
    q = (beta % (2*pi) - temp1 + atan2(2, p)) % (2*pi)
    return t, p, q, mode

def dubins_RLR(alpha, beta, d):
    mode = ['R', 'L', 'R']
    temp0 = (6-d**2+2*cos(alpha-beta)+2*d*(sin(alpha) - sin(beta))) / 8
    if abs(temp0) > 1:
        return None, None, None, mode
    p = acos(temp0)
    temp1 = atan2( (cos(alpha) - cos(beta)), (d-sin(alpha)+sin(beta)) )
    t = (alpha - temp1 + p/2) % (2*pi)
    q = (alpha - beta - t + p) % (2*pi)
    return t, p, q, mode

def dubins_LRL(alpha, beta, d):
    mode=['L', 'R', 'L']
    temp0 = (6-d**2+2*cos(alpha-beta)+2*d*(sin(beta) - sin(alpha))) / 8
    if abs(temp0) > 1:
        return None, None, None, mode
    p = acos(temp0)
    temp1 = atan2( (cos(beta)) - cos(alpha), (d+sin(alpha)-sin(beta)) )
    t = (- alpha + temp1 + p/2) % (2*pi)
    q = ( beta % (2*pi) - alpha - t  + p) % (2*pi)
    return t, p, q, mode

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
# -50 left max, 50 right max
# yaw is the current angle of the car in degrees starting from east and increasing counter clockwise
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry, rd, target_index

    lookahead_distance = 10

    for i in range(len(rx) - 1, target_index, -1):
        if get_distance(x, y, rx[i], ry[i]) < lookahead_distance:
            target_index = i
            break

    lx = rx[target_index]
    ly = ry[target_index]
    print("Current Point:", x, y)
    print("Lookahead Point:", lx, ly)

    alpha = degrees(atan2(y - ly, lx - x)) - yaw
    steer_angle = -degrees(atan2(2 * WB * sin(radians(alpha)), lookahead_distance))

    if rd[target_index] == -1:  # backward
        speed = -50

    else:
        if get_distance(x, y, P_END[0], P_END[1]) < 64: # parking complete
            speed = 0
            target_index = 0
            print("Parking Complete")

        else:   # forward
            speed = 50

    print("Vehicle yaw:", yaw, "Alpha:", alpha)
    print("Steer angle:", steer_angle, "Speed:", speed)
    print("\n")
    drive(steer_angle, speed)

def get_distance(x1, y1, x2, y2):
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

