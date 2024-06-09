# 实现三维位姿估计

import cv2  
import numpy as np  
import math
  
# 需要标定, realsense L515 RGB 1080 * 1920  
LIMBUS_R_MM = 6  
FOCAL_LEN_X_PX = 1352.7  # 4 param in slam book
FOCAL_LEN_Y_PX = 1360.6  
FOCAL_LEN_Z_PX = (FOCAL_LEN_X_PX + FOCAL_LEN_Y_PX) / 2  
PRIN_POINT = np.array([979.5840, 552.1356], dtype=np.float64)  

N = 50 # a batch -> how many point used to estimate one point
posBuf = []
dirBuf = []
counter = 0
  
def ellipse_to_limbus(x, y, w, h, angle, limbus_switch=True):  
    """
    caculate 3d pose
    :param x: x-position
    :param y: y-position
    :param w: width of pupil, maj_axis of ellipse
    :param h: height of pupil, min_axis of ellipse
    :param angle: rotate of ellipse
    :return: an image with a circle around the pupil
    """

    global counter

    # 检验无效数据
    if w <= 0 or h <= 0 or w < h:
        return [], False

    # 转换到毫米空间  
    iris_z_mm = (LIMBUS_R_MM * 2 * FOCAL_LEN_Z_PX) / w  
    iris_x_mm = -iris_z_mm * (x - PRIN_POINT[0]) / FOCAL_LEN_X_PX  
    iris_y_mm = iris_z_mm * (y - PRIN_POINT[1]) / FOCAL_LEN_Y_PX  
  
    # 构建三维点  
    limbus_center = np.array([iris_x_mm, iris_y_mm, iris_z_mm], dtype=np.float64)  
    
    # 角度转换为弧度  
    psi = math.pi / 180.0 * (angle + 90)  # z-axis rotation (radians)  
    # if h / w > 1:
    #     print(f"h is {h} and w is {w}")
    tht = math.acos(h / w)   # y-axis rotation (radians)  

    if limbus_switch:  # 什么时候true
        tht = -tht  # ambiguous acos, so sometimes switch limbus  

    # 计算limbus normal  
    limb_normal = np.array([  
        math.sin(tht) * math.cos(psi),  
        -math.sin(tht) * math.sin(psi),  
        -math.cos(tht)  
    ])  

    # 校正弱透视  
    x_correction = math.atan2(-iris_y_mm, iris_z_mm)  
    y_correction = math.atan2(iris_x_mm, iris_z_mm)  
  

    # 创建旋转矩阵（使用Rodrigues公式，但这里直接构建）  
    Ry = np.array([  
        [math.cos(y_correction), 0, math.sin(y_correction)],  
        [0, 1, 0],  
        [-math.sin(y_correction), 0, math.cos(y_correction)]  
    ])  

    Rx = np.array([  
        [1, 0, 0],  
        [0, math.cos(x_correction), -math.sin(x_correction)],  
        [0, math.sin(x_correction), math.cos(x_correction)]  
    ])  

    # 应用旋转  
    limb_normal = np.dot(Ry, limb_normal)  
    limb_normal = np.dot(Rx, limb_normal)  

    # 滤波算法：取中位数
    counter = (counter + 1) % N
    posBuf.append(limbus_center)
    dirBuf.append(limb_normal)
    if counter == 49:
        coords_array = np.array(posBuf)
        pos_medians = np.median(coords_array, axis=0)

        coords_array = np.array(dirBuf)
        dir_medians = np.median(coords_array, axis=0)

        # 写出, 格式为: 
        # x1 y1 z1
        # x2 y2 z2
        # ...
        output_file = "raw_point.txt"
        with open(output_file, "a+") as file:
            for pos in posBuf:
                file.write(f"{pos[0]} {pos[1]} {pos[2]}\n")

        posBuf.clear()
        dirBuf.clear()
        return [pos_medians, dir_medians], True
    else:
        return [], False