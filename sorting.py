#!/usr/bin/python3
# coding=utf8
# Date:2024/02/05
# Author:Aiden
# 第13章 ArmPi FPV创意玩法课程\1.AI视觉创意玩法\第4课 物品分拣（13. ArmPi FPV creative AI application course\1. AI vision creative program\Lesson 4 object sorting)
import cv2
import math
import queue
import rospy
import signal
import threading
import numpy as np
from threading import Timer

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool, SetBoolResponse
from std_msgs.msg import Float32

from warehouse.msg import Grasp
from object_sorting.srv import SetTarget
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from ros_robot_controller.msg import RGBState, RGBsState, BuzzerState 

from armpi_fpv_common import pid, misc, common, apriltag
from hiwonder_servo_controllers import bus_servo_control
from armpi_fpv_kinematics.kinematics_control import set_pose_target

# 分捡(sorting)
# 如未声明，使用的长度，距离单位均为m(if not specified, lengths and distances are assumed to be in meters)
d_tag_map = 0

tag_z_min = 0.01
tag_z_max = 0.015

d_color_map = 30

color_z_min = 0.01
color_z_max = 0.015
d_color_y = 20
color_y_adjust = 400

center_x = 320

running = False

lock = threading.RLock()
mask1 = cv2.imread('/home/ubuntu/armpi_fpv/src/object_sorting/scripts/mask1.jpg', 0)
mask2 = cv2.imread('/home/ubuntu/armpi_fpv/src/object_sorting/scripts/mask2.jpg', 0)
rows, cols = mask1.shape

# 初始位置(initialization position)
def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(joints_pub, 1.5, ((1, 200), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    if delay:
        rospy.sleep(2) 
 
def set_rgb(r, g, b):
    rgb1 = RGBState()
    rgb1.id = 1
    rgb1.r = r
    rgb1.g = g
    rgb1.b = b
    rgb2 = RGBState()
    rgb2.id = 2
    rgb2.r = r
    rgb2.g = g
    rgb2.b = b
    msg = RGBsState()
    msg.data = [rgb1, rgb2]
    rgb_pub.publish(msg)
 
x_dis = 500
Y_DIS = 0
y_dis = Y_DIS
last_x_dis = x_dis
last_x_dis = y_dis
x_pid = pid.PID(P=0.01, I=0.001, D=0)#pid初始化(pid initialization)
y_pid = pid.PID(P=0.000008, I=0, D=0)

tag_x_dis = 500
tag_y_dis = 0
tag_x_pid = pid.PID(P=0.02, I=0.001, D=0)#pid初始化(pid initialization)
tag_y_pid = pid.PID(P=0.02, I=0, D=0)

stop_state = 0
move_state = 1
adjust = False
approach = False
rotation_angle = 0
start_move = False
adjust_error = False
last_X, last_Y = 0, 0
box_rotation_angle = 0
last_box_rotation_angle = 0
tag1 = ['tag1', -1, -1, -1, 0]
tag2 = ['tag2', -1, -1, -1, 0]
tag3 = ['tag3', -1, -1, -1, 0]
current_tag = ['tag1', 'tag2', 'tag3']
detect_color = ('red', 'green', 'blue')

count = 0
count2 = 0
count3 = 0
count_d = 0
count_timeout = 0
count_tag_timeout = 0
count_adjust_timeout = 0
# 变量重置(variable reset)
def reset():
    global X, Y
    global adjust    
    global approach
    global move_state
    global start_move
    global current_tag
    global detect_color
    global x_dis, y_dis
    global adjust_error
    global last_X, last_Y
    global tag1, tag2, tag3
    global box_rotation_angle
    global tag_x_dis, tag_y_dis
    global last_x_dis, last_y_dis
    global rotation_angle, last_box_rotation_angle    
    global count, count2, count3, count_timeout, count_adjust_timeout, count_d, count_tag_timeout
    
    with lock:
        X = 0
        Y = 0
        
        x_dis = 500
        y_dis = Y_DIS
        tag_x_dis = 500
        tag_y_dis = 0    
        x_pid.clear()
        y_pid.clear()
        tag_x_pid.clear()
        tag_y_pid.clear()
        last_x_dis = x_dis
        last_y_dis = y_dis
        
        adjust = False
        approach = False
        start_move = False
        adjust_error = False
        
        move_state = 1
        set_rgb(0, 0, 0)
        rotation_angle = 0
        box_rotation_angle = 0
        last_box_rotation_angle = 0

        count = 0
        count2 = 0
        count3 = 0
        count_d = 0
        count_timeout = 0
        count_tag_timeout = 0
        count_adjust_timeout = 0

        tag1 = ['tag1', -1, -1, -1, 0]
        tag2 = ['tag2', -1, -1, -1, 0]
        tag3 = ['tag3', -1, -1, -1, 0]
        current_tag = ['tag1', 'tag2', 'tag3']
        detect_color = ('red', 'green', 'blue')

color_range = None
def load_lab_config():
    global color_range
    
    # 获取lab参数(get lab parameter)
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})  # get lab range from ros param server

def load_config():
    global config
    global d_tag_map
    global tag_z_min
    global tag_z_max
    global d_color_map
    global color_z_min
    global color_z_max
    global d_color_y
    global color_y_adjust
    global center_x
    
    
    config = rospy.get_param('config', {})
    if config != {}:        
        d_tag_map = config['d_tag_map']

        tag_z_min = config['tag_z_min']
        tag_z_max = config['tag_z_max']

        d_color_map = config['d_color_map']

        color_z_min = config['color_z_min']
        color_z_max = config['color_z_max']
        d_color_y = config['d_color_y']
        color_y_adjust = config['color_y_adjust']

        center_x = config['center_x']

# app初始化调用(app initialization call)
def init():
    global stop_state
    global target_data

    rospy.loginfo("object sorting Init")
    load_lab_config()
    load_config()
    stop_state = 0
    target_data = ((), ())
    initMove()
    reset()

y_d = 0
roll_angle = 0
gripper_rotation = 0
# 木块对角长度一半(half the diagonal length of the wooden block)
square_diagonal = 0.03*math.sin(math.pi/4)
F = 1000/240.0
# 夹取(grasp)
def pick(grasps, have_adjust=False):
    return False

def place(places):
    return False

##################################################
# 放置坐标x, y, z(m)(position of each shelf layer x, y, z(m))
place_position = {'red':  [0.190,    0.061,  0.01],
                  'green':[0.190,    0,      0.01],
                  'blue': [0.190,   -0.059,  0.01],
                  'tag1': [-0.190,   0.053,  0.01],
                  'tag2': [-0.190,   0,      0.01],
                  'tag3': [-0.190,  -0.061,  0.01]}
###################################################


def move():
 return 0

# 检测apriltag(detect apriltag)
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):
    global tag1, tag2, tag3
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)
    
    tag1 = ['tag1', -1, -1, -1, 0]
    tag2 = ['tag2', -1, -1, -1, 0]
    tag3 = ['tag3', -1, -1, -1, 0]
    if len(detections) != 0:
        for i, detection in enumerate(detections):              
            corners = np.rint(detection.corners)  # 获取四个角点(get four angular point)
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            tag_id = int(detection.tag_id)  # 获取tag_id(get tag_id)
            
            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点(center point)
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角(calculate rotation angle)
            
            cv2.putText(img, str(tag_id), (object_center_x - 10, object_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 255], 2)
            
            if tag_id == 1:
                tag1 = ['tag1', object_center_x, object_center_y, object_angle]
            elif tag_id == 2:
                tag2 = ['tag2', object_center_x, object_center_y, object_angle]
            elif tag_id == 3:
                tag3 = ['tag3', object_center_x, object_center_y, object_angle]

# 获取roi，防止干扰(get roi, prevent interference)
def getROI(rotation_angle):
    rotate1 = cv2.getRotationMatrix2D((rows*0.5, cols*0.5), int(rotation_angle), 1)
    rotate_rotate1 = cv2.warpAffine(mask2, rotate1, (cols, rows))
    mask_and = cv2.bitwise_and(rotate_rotate1, mask1)
    rotate2 = cv2.getRotationMatrix2D((rows*0.5, cols*0.5), int(-rotation_angle), 1)
    rotate_rotate2 = cv2.warpAffine(mask_and, rotate2, (cols, rows))
    frame_resize = cv2.resize(rotate_rotate2, (710, 710), interpolation=cv2.INTER_NEAREST)
    roi = frame_resize[40:280, 184:504]
    
    return roi

size = (320, 240)

last_x = 0
last_y = 0
state = None
x_adjust = 0
pick_color = ''
# 颜色夹取策略(color grasping strategy)
def color_sort(img, target):
    global X, Y
    global count
    global state
    global adjust
    global approach
    global x_adjust    
    global pick_color
    global current_tag
    global adjust_error
    global x_dis, y_dis
    global detect_color
    global count_timeout
    global rotation_angle
    global box_rotation_angle
    global last_x_dis, last_y_dis
    global last_box_rotation_angle    
    global last_x, last_y, count_d, start_move

    img_h, img_w = img.shape[:2]
      
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)
    
    max_area = 0
    color_area_max = None
    areaMaxContour_max = 0
    roi = getROI(rotation_angle)
    for i in color_range:
        if i in target:
            if i in detect_color:
                target_color_range = color_range[i]                
                frame_mask1 = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(target_color_range['max']))  # 对原图像和掩模进行位运算(perform bitwise operations on the original image and the mask)
                #mask = cv2.bitwise_and(roi, frame_gray)
                frame_mask2 = cv2.bitwise_and(roi, frame_mask1)
                eroded = cv2.erode(frame_mask2, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
                #cv2.imshow('mask', dilated)
                #cv2.waitKey(1)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find out contour)
                areaMaxContour, area_max = common.get_area_max_contour(contours)  # 找出最大轮廓(find out the largest contour)
                if areaMaxContour is not None:
                    if area_max > max_area and area_max > 100:#找最大面积(find the largest area)
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
    if max_area > 100:  # 有找到最大面积(the largest area is found)
        rect = cv2.minAreaRect(areaMaxContour_max)
        box_rotation_angle = rect[2]
        if box_rotation_angle > 45:
            box_rotation_angle =  box_rotation_angle - 90        
        
        box = np.int0(cv2.boxPoints(rect))   
        for j in range(4): # 映射到原图大小(mapping to original image size)
            box[j, 0] = int(misc.map(box[j, 0], 0, size[0], 0, img_w))
            box[j, 1] = int(misc.map(box[j, 1], 0, size[1], 0, img_h))
        
        cv2.drawContours(img, [box], -1, common.range_rgb[color_area_max], 2)
        
        centerX = int(misc.map(((areaMaxContour_max[areaMaxContour_max[:,:,0].argmin()][0])[0] + (areaMaxContour_max[areaMaxContour_max[:,:,0].argmax()][0])[0])/2, 0, size[0], 0, img_w))
        centerY = int(misc.map((areaMaxContour_max[areaMaxContour_max[:,:,1].argmin()][0])[1], 0, size[1], 0, img_h))
        
        print(f"Detected color: {color_area_max}, Center: ({centerX}, {centerY}), Last: ({last_x}, {last_y})")
        #cv2.line(img, (0, 430), (640, 430), (0, 255, 255), 2)
        #cv2.circle(img, (int(centerX), int(centerY)), 5, common.range_rgb[color_area_max], -1)
        if abs(centerX - last_x) <= 5 and abs(centerY - last_y) <= 5 and not start_move:
            count_d += 1
            if count_d > 5:
                count_d = 0
                start_move = True
                print(color_area_max)
                set_rgb(common.range_rgb[color_area_max][2], common.range_rgb[color_area_max][1], common.range_rgb[color_area_max][0])
                rospy.sleep(0.1)
                
                # 位置映射
                if 298 + d_color_map < centerY <= 424 + d_color_map:
                    Y = misc.map(centerY, 298 + d_color_map, 424 + d_color_map, 0.12, 0.12 - 0.04)
                elif 198 + d_color_map < centerY <= 298 + d_color_map:
                    Y = misc.map(centerY, 198 + d_color_map, 298 + d_color_map, 0.12 + 0.04, 0.12)
                elif 114 + d_color_map < centerY <= 198 + d_color_map:
                    Y = misc.map(centerY, 114 + d_color_map, 198 + d_color_map, 0.12 + 0.08, 0.12 + 0.04)
                elif 50 + d_color_map < centerY <= 114 + d_color_map:
                    Y = misc.map(centerY, 50 + d_color_map, 114 + d_color_map, 0.12 + 0.12, 0.12 + 0.08)
                elif 0 + d_color_map < centerY <= 50 + d_color_map:
                    Y = misc.map(centerY, 0 + d_color_map, 50 + d_color_map, 0.12 + 0.16, 0.12 + 0.12)
                else:
                    Y = 1
        else:
            count_d = 0
        
        last_x = centerX
        last_y = centerY
        print(f"Updated last_y: {last_y}")
        if (not approach or adjust) and start_move: # pid调节(pid adjustment)
            detect_color = (color_area_max, )            
            x_pid.SetPoint = center_x #设定(setting)
            x_pid.update(centerX) #当前(current)
            dx = x_pid.output
            x_dis += dx #输出(output)
            # print(x_dis, dx, center_x centerX)
            x_dis = 0 if x_dis < 0 else x_dis          
            x_dis = 1000 if x_dis > 1000 else x_dis
             
            if adjust:
                y_pid.SetPoint = color_y_adjust
                start_move = True
                # print(1111, gripper_rotation, roll_angle)
                centerY += abs(misc.map(70*math.sin(math.pi/4)/2, 0, size[0], 0, img_w)*math.sin(math.radians(abs(gripper_rotation) + 45))) + 65*math.sin(math.radians(abs(roll_angle)))
                if Y < 0.12 + 0.04:
                    centerY += d_color_y 
                if 0 < centerY - color_y_adjust <= 5:
                    centerY = color_y_adjust
                y_pid.update(centerY)

                dy = y_pid.output
                y_dis += dy
                y_dis = 0.1 if y_dis > 0.1 else y_dis
                y_dis = -0.1 if y_dis < -0.1 else y_dis
            else:
                dy = 0
            if abs(dx) < 0.1 and abs(dy) < 0.0001 and (abs(last_box_rotation_angle - rect[2]) <= 10 or abs(last_box_rotation_angle - rect[2] >= 80)):
                count += 1
                rospy.sleep(0.01)
                if (adjust and count > 10) or (not adjust and count >= 10):
                    count = 0
                    if adjust:
                        adjust = False
                    else:
                        rotation_angle = 240 * (x_dis - 500)/1000.0
                        X = round(-Y * math.tan(math.radians(rotation_angle)), 4)
                        state = 'color'
                        pick_color = detect_color[0]
                        adjust_error = False
                        approach = True
            else:
                count = 0
                
            if adjust and (abs(last_x_dis - x_dis) >= 2 or abs(last_y_dis - y_dis) > 0.002):
                position = grasps.grasp_pos.position
                rotation = grasps.grasp_pos.rotation
                target = set_pose_target((position.y + y_dis, position.x, position.z), rotation.r, [-90, 90], 0)
                if target[1]:
                    servo_data = target[1]
                    bus_servo_control.set_servos(joints_pub, 0.1, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, int(x_dis))))
                    rospy.sleep(0.1)
                    last_x_dis = x_dis
                    last_y_dis = y_dis
                else:
                    bus_servo_control.set_servos(joints_pub, 0.02, ((6, int(x_dis)), ))
            else:                    
                bus_servo_control.set_servos(joints_pub, 0.02, ((6, int(x_dis)), ))
            
            last_box_rotation_angle = rect[2]
        if distance_pub is not None:
            distance = pixel_y_to_physical_y(last_y)
            distance_pub.publish(distance)
    else:
        count_timeout += 1
        if count_timeout > 20:
            adjust_error = True
            count_timeout = 0
            current_tag = ['tag1', 'tag2', 'tag3']
            detect_color = target_data[0]
    return img

d_map = 0.015
tag_map = [425, 384, 346, 310, 272, 239, 208, 177, 153, 129, 106, 86, 68, 51]
# apriltag夹取策略(apriltag grasping strategy)
def tag_sort(img, target):
    global X, Y
    global state
    global count2
    global count3
    global adjust    
    global approach
    global start_move
    global current_tag   
    global adjust_error
    global last_X, last_Y
    global box_rotation_angle
    global tag_x_dis, tag_y_dis

    centerX = target[1]
    centerY = target[2]

    box_rotation_angle = abs(target[3])
    if box_rotation_angle > 90:
        box_rotation_angle -= 90
    if box_rotation_angle > 45:
        box_rotation_angle =  box_rotation_angle - 90
    if target[3] < 0:
        box_rotation_angle = -box_rotation_angle
    
    distance = math.sqrt(pow(centerX - last_X, 2) + pow(centerY - last_Y, 2)) #对比上次坐标来判断是否移动(comparing coordinates from the previous time to determine movement)
    if distance < 5 and not start_move:
        count2 += 1
        if count2 > 20:
            count2 = 0
            start_move = True          
    else:
        count2 = 0
    if (not approach or adjust) and start_move:  
        tag_x_pid.SetPoint = center_x #设定(setting)
        tag_x_pid.update(centerX) #当前(current)
        dx = tag_x_pid.output
        tag_x_dis += dx #输出(output)
        tag_x_dis = 0 if tag_x_dis < 0 else tag_x_dis          
        tag_x_dis = 1000 if tag_x_dis > 1000 else tag_x_dis
        
        if abs(centerX - last_X) <= 1 and X != -1:
            count3 += 1
            rospy.sleep(0.01)
            if count3 > 30:
                count3 = 0
                if adjust:
                    adjust = False
                else:
                    current_tag = target[0]
                    # 位置映射(position mapping)
                    if tag_map[1] + d_tag_map < centerY <= tag_map[0] + d_tag_map:
                        Y = misc.map(centerY, tag_map[1] + d_tag_map, tag_map[0] + d_tag_map, 0.12 + d_map, 0.12) - 0.005
                    elif tag_map[2] + d_tag_map < centerY <= tag_map[1] + d_tag_map:
                        Y = misc.map(centerY, tag_map[2] + d_tag_map, tag_map[1] + d_tag_map, 0.12 + 2*d_map, 0.12 + d_map)
                    elif tag_map[3] + d_tag_map < centerY <= tag_map[2] + d_tag_map:
                        Y = misc.map(centerY, tag_map[3] + d_tag_map, tag_map[2] + d_tag_map, 0.12 + 3*d_map, 0.12 + 2*d_map)
                    elif tag_map[4] + d_tag_map < centerY <= tag_map[3] + d_tag_map:
                        Y = misc.map(centerY, tag_map[4] + d_tag_map, tag_map[3] + d_tag_map, 0.12 + 4*d_map, 0.12 + 3*d_map)
                    elif tag_map[5] + d_tag_map < centerY <= tag_map[4] + d_tag_map:
                        Y = misc.map(centerY, tag_map[5] + d_tag_map, tag_map[4] + d_tag_map, 0.12 + 5*d_map, 0.12 + 4*d_map)
                    elif tag_map[6] + d_tag_map < centerY <= tag_map[5] + d_tag_map:
                        Y = misc.map(centerY, tag_map[6] + d_tag_map, tag_map[5] + d_tag_map, 0.12 + 6*d_map, 0.12 + 5*d_map)
                    elif tag_map[7] + d_tag_map < centerY <= tag_map[6] + d_tag_map:
                        Y = misc.map(centerY, tag_map[7] + d_tag_map, tag_map[6] + d_tag_map, 0.12 + 7*d_map, 0.12 + 6*d_map)
                    elif tag_map[8] + d_tag_map < centerY <= tag_map[7] + d_tag_map:
                        Y = misc.map(centerY, tag_map[8] + d_tag_map, tag_map[7] + d_tag_map, 0.12 + 8*d_map, 0.12 + 7*d_map)
                    elif tag_map[9] + d_tag_map < centerY <= tag_map[8] + d_tag_map:
                        Y = misc.map(centerY, tag_map[9] + d_tag_map, tag_map[8] + d_tag_map, 0.12 + 9*d_map, 0.12 + 8*d_map)
                    elif tag_map[10] + d_tag_map < centerY <= tag_map[9] + d_tag_map:
                        Y = misc.map(centerY, tag_map[10] + d_tag_map, tag_map[9] + d_tag_map, 0.12 + 10*d_map, 0.12 + 9*d_map)
                    elif tag_map[11] + d_tag_map < centerY <= tag_map[10] + d_tag_map:
                        Y = misc.map(centerY, tag_map[11] + d_tag_map, tag_map[10] + d_tag_map, 0.12 + 11*d_map, 0.12 + 10*d_map)
                    elif tag_map[12] + d_tag_map < centerY <= tag_map[11] + d_tag_map:
                        Y = misc.map(centerY, tag_map[12] + d_tag_map, tag_map[11] + d_tag_map, 0.12 + 12*d_map, 0.12 + 11*d_map)
                    elif tag_map[13] + d_tag_map < centerY <= tag_map[12] + d_tag_map:
                        Y = misc.map(centerY, tag_map[13] + d_tag_map, tag_map[12] + d_tag_map, 0.12 + 13*d_map, 0.12 + 12*d_map)
                    else:
                        Y = 1
                    
                    X = round(-Y * math.tan(math.radians(rotation_angle)), 4)
                    state = 'tag'
                    approach = True
                    adjust_error = False
                    adjust = False
        else:
            count3 = 0
        
        bus_servo_control.set_servos(joints_pub, 0.02, ((6, int(tag_x_dis)), ))
    last_X, last_Y = centerX, centerY
        
    return img

def run():
    global stop_state
    global current_tag
    global adjust_error
    global count_tag_timeout
    global count_adjust_timeout
   
    try:
        image = image_queue.get(block=True, timeout=1)
    except queue.Empty:
        return
    
    if running:

        if len(target_data[1]) != 0:
            apriltagDetect(image) # apriltag检测(apriltag detection)
            
        # 选取策略，优先tag, 夹取超时处理(select strategy, priority tag, handling of grasping timeout)
        if 'tag1' in target_data[1] and 'tag1' in current_tag:
            if tag1[1] != -1:
                count_adjust_timeout = 0
                image = tag_sort(image, tag1)
            else:
                if adjust:
                    count_adjust_timeout += 1
                    if count_adjust_timeout > 50:
                        count_adjust_timeout = 0
                        adjust_error = True
                else:
                    count_tag_timeout += 1
                    if count_tag_timeout > 3:
                        count_tag_timeout = 0
                        if current_tag != 'tag1':
                            current_tag.remove('tag1')
        elif 'tag2' in target_data[1] and 'tag2' in current_tag:
            if tag2[1] != -1:
                count_adjust_timeout = 0
                image = tag_sort(image, tag2)
            else:
                if adjust:
                    count_adjust_timeout += 1
                    if count_adjust_timeout > 50:
                        count_adjust_timeout = 0
                        adjust_error = True
                else:
                    count_tag_timeout += 1
                    if count_tag_timeout > 3:
                        count_tag_timeout = 0
                        if current_tag != 'tag2':
                            current_tag.remove('tag2')
        elif 'tag3' in target_data[1] and 'tag3' in current_tag:
            if tag3[1] != -1:
                count_adjust_timeout = 0
                image = tag_sort(image, tag3)
            else:
                if adjust:
                    count_adjust_timeout += 1
                    if count_adjust_timeout > 50:
                        count_adjust_timeout = 0
                        adjust_error = True
                else:
                    count_tag_timeout += 1
                    if count_tag_timeout > 3:
                        count_tag_timeout = 0
                        if current_tag != 'tag3':
                            current_tag.remove('tag3')
        elif len(target_data[0]) != 0:
            image = color_sort(image, target_data[0])
        else:
            current_tag = ['tag1', 'tag2', 'tag3']
    else:
        if stop_state:
            stop_state = 0
            initMove(delay=False)

    img_h, img_w = image.shape[:2]
    cv2.line(image, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(image, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)
    ros_image = common.cv2_image2ros(image)
    image_pub.publish(ros_image)

image_queue = queue.Queue(maxsize=2)
def image_callback(ros_image):
    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB frame)
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    if image_queue.full():
        # 如果队列已满，丢弃最旧的图像(discard the oldest image if the queue is full)
        image_queue.get()
    # 将图像放入队列(add the image to the queue)
    image_queue.put(bgr_image)

registered = False
def enter_func(msg):
    global lock
    global image_sub
    global running
    global registered
    
    rospy.loginfo("enter object sorting")
    with lock:
        init()
        if not registered:
            registered = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
               
    return [True, 'enter']

heartbeat_timer = None
def exit_func(msg):
    global lock
    global image_sub
    global running
    global registered
    
    rospy.loginfo("exit object sorting")
    with lock:
        running = False
        try:
            if registered:
                registered = False
                if heartbeat_timer is not None:
                    heartbeat_timer.cancel()
                image_sub.unregister()
        except:
            pass
        
    return [True, 'exit']

def start_running():
    global lock
    global running
    
    rospy.loginfo("start running object sorting")
    with lock:
        running = True

def stop_running():
    global lock
    global stop_state
    global running
    
    rospy.loginfo("stop running object sorting")
    with lock:
        running = False
        if (not approach and start_move) or adjust:
            stop_state = 1
        reset()

def set_running(msg):    
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

target_data = ((), ())
def set_target(msg):
    global lock
    global target_data
    
    rospy.loginfo('{} {}'.format(msg.color, msg.tag))
    with lock:
        target_data = (msg.color, msg.tag)

    return [True, 'set_target']

# 心跳
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/object_sorting/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

def shutdown(signal, frame):
    exit_func(None)
    rospy.signal_shutdown('shutdown')
    
def pixel_y_to_physical_y(centerY):
    global d_color_map
    if 298 + d_color_map < centerY <= 424 + d_color_map:
        return misc.map(centerY, 298 + d_color_map, 424 + d_color_map, 0.12, 0.12 - 0.04)
    elif 198 + d_color_map < centerY <= 298 + d_color_map:
        return misc.map(centerY, 198 + d_color_map, 298 + d_color_map, 0.12 + 0.04, 0.12)
    elif 114 + d_color_map < centerY <= 198 + d_color_map:
        return misc.map(centerY, 114 + d_color_map, 198 + d_color_map, 0.12 + 0.08, 0.12 + 0.04)
    elif 50 + d_color_map < centerY <= 114 + d_color_map:
        return misc.map(centerY, 50 + d_color_map, 114 + d_color_map, 0.12 + 0.12, 0.12 + 0.08)
    elif 0 + d_color_map < centerY <= 50 + d_color_map:
        return misc.map(centerY, 0 + d_color_map, 50 + d_color_map, 0.12 + 0.16, 0.12 + 0.12)
    else:
        return 1.0

if __name__ == '__main__':
    # 初始化节点(initialization node)
    rospy.init_node('object_sorting', log_level=rospy.INFO)

    # 舵机发布(servo publish)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    # 图像发布(image publish)
    image_pub = rospy.Publisher('/object_sorting/image_result', Image, queue_size=1)  # register result image publisher
    
    # app通信服务(app communication publish)
    enter_srv = rospy.Service('/object_sorting/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/object_sorting/exit', Trigger, exit_func)
    running_srv = rospy.Service('/object_sorting/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/object_sorting/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service('/object_sorting/heartbeat', SetBool, heartbeat_srv_cb)
    
    # 蜂鸣器(buzzer)
    buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
    # rgb 灯(rgb light)
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    
    signal.signal(signal.SIGINT, shutdown) 
    debug = 0
    if debug:
        rospy.sleep(0.2)
        enter_func(1)

        msg = SetTarget()
        msg.color = ['red', 'green', 'blue']
        msg.tag = ['tag1', 'tag2', 'tag3']
        set_target(msg)

        start_running()
    distance_pub = rospy.Publisher('/object_sorting/target_distance', Float32, queue_size=1)
    while not rospy.is_shutdown():
        run()


