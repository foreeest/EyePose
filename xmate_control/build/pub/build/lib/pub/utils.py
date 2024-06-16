import numpy as np
import math

# Geometry Module
  
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
    :return: 3 pos and 3 angle, will return false if input invalid
    """

    global counter

    if np.isnan(x) or np.isnan(y) or np.isnan(w) or np.isnan(h):
        output_file = "bug_log.txt"
        with open(output_file, "a+") as file:
            file.write(f"get some NaN in Geometry module\n")
            file.write(f"{x} {y} {w} {h}\n")
        return [], False

    # 检验无效数据
    if w <= 0 or h <= 0 or w < h:
        output_file = "bug_log.txt"
        with open(output_file, "a+") as file:
            file.write(f"invalid w or h in Geometry module\n")
            file.write(f"{w} {h}\n")
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

# Ellipse Detect Module 

def fit_rotated_ellipse_ransac(data,iter=50,sample_num=10,offset=80.0):

    count_max = 0
    effective_sample = None

    for i in range(iter):
        sample = np.random.choice(len(data), sample_num, replace=False)

        xs = data[sample][:,0].reshape(-1,1)
        ys = data[sample][:,1].reshape(-1,1)
        # float -> float32
        J = np.mat( np.hstack((xs*ys,ys**2,xs, ys, np.ones_like(xs,dtype=np.float32))) )
        Y = np.mat(-1*xs**2)
        if np.linalg.det(J.T * J) == 0:
            return 0, 0, 0, 0, 0
        P = (J.T * J).I * J.T * Y

        # fitter a*x**2 + b*x*y + c*y**2 + d*x + e*y + f = 0
        a = 1.0; b= P[0,0]; c= P[1,0]; d = P[2,0]; e= P[3,0]; f=P[4,0];
        ellipse_model = lambda x,y : a*x**2 + b*x*y + c*y**2 + d*x + e*y + f

        # threshold 
        ran_sample = np.array([[x,y] for (x,y) in data if np.abs(ellipse_model(x,y)) < offset ])

        if(len(ran_sample) > count_max):
            count_max = len(ran_sample) 
            effective_sample = ran_sample

    return fit_rotated_ellipse(effective_sample)


def fit_rotated_ellipse(data):

    xs = data[:,0].reshape(-1,1) 
    ys = data[:,1].reshape(-1,1)
    # float -> float32
    J = np.mat( np.hstack((xs*ys,ys**2,xs, ys, np.ones_like(xs,dtype=np.float32))) ) # 雅各比矩阵
    Y = np.mat(-1*xs**2) # 目标矩阵
    if np.linalg.det(J.T * J) == 0:
        return 0, 0, 0, 0, 0
    P = (J.T * J).I * J.T * Y

    a = 1.0; b= P[0,0]; c= P[1,0]; d = P[2,0]; e= P[3,0]; f=P[4,0]; # 计算椭圆6参数
    if a != c:
        theta = 0.5* np.arctan(b/(a-c))  # 保证 a != c
    else:
        return 0, 0, 0, 0, 0
    
    cx = (2*c*d - b*e)/(b**2-4*a*c)
    cy = (2*a*e - b*d)/(b**2-4*a*c)

    cu = a*cx**2 + b*cx*cy + c*cy**2 -f
    w= np.sqrt(cu/(a*np.cos(theta)**2 + b* np.cos(theta)*np.sin(theta) + c*np.sin(theta)**2))
    h= np.sqrt(cu/(a*np.sin(theta)**2 - b* np.cos(theta)*np.sin(theta) + c*np.cos(theta)**2))

    ellipse_model = lambda x,y : a*x**2 + b*x*y + c*y**2 + d*x + e*y + f

    error_sum = np.sum([ellipse_model(x,y) for x,y in data])
    # print('fitting error = %.3f' % (error_sum))

    if np.isnan(cx) or np.isnan(cy) or np.isnan(w) or np.isnan(h) or np.isnan(theta):
        output_file = "bug_log.txt"
        with open(output_file, "a+") as file:
            file.write(f"get some NaN in ellipse module\n")
            file.write(f"{cx} {cy} {w} {h} {theta}\n")
        return 0, 0, 0, 0, 0
    
    if w <= 0 or h <= 0 or w < h:
        output_file = "bug_log.txt"
        with open(output_file, "a+") as file:
            file.write(f"invalid w or h in ellipse module\n")
            file.write(f"{w} {h}\n")
        return 0, 0, 0, 0, 0

    return (cx,cy,w,h,theta)