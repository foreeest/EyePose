# using realsense l515's rgb camera
import cv2
import numpy as np
from geometry import ellipse_to_limbus

import pyrealsense2 as rs

# 无效值直接返回5个0
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


def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            # depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            # Convert images to numpy arrays
            frame = np.asanyarray(color_frame.get_data())
            color_colormap_dim = frame.shape

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
            image_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(image_gray,(3,3),0)
            ret,thresh1 = cv2.threshold(blur,50,255,cv2.THRESH_BINARY)
            opening = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel)
            closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

            image = 255 - closing
            # because of version
            # _,contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            hull = []

            for i in range(len(contours)):
                hull.append(cv2.convexHull(contours[i], False)) 
                            
            # cnt = sorted(hull, key=cv2.contourArea)
            # maxcnt = cnt[-1]
            for con in hull:
                approx = cv2.approxPolyDP(con, 0.01 * cv2.arcLength(con,True),True)
                area = cv2.contourArea(con)
                if(len(approx) > 10 and area > 1000):
                    cx,cy,w,h,theta = fit_rotated_ellipse_ransac(con.reshape(-1,2))

                    limbus, flag = ellipse_to_limbus(cx, cy, w/2, h/2, theta, True) # 50个点返回一次
                    if flag:
                        postion = limbus[0]
                        direction = limbus[1]
                        print(f"predicted postion is {postion}")
                        print(f"predicted direction is {direction}")

                    cv2.ellipse(frame,(int(cx),int(cy)),(int(w),int(h)),theta*180.0/np.pi,0.0,360.0,(0,255,0),1)
                    cv2.drawMarker(frame, (int(cx),int(cy)),(0, 0, 255),cv2.MARKER_CROSS,2,1)
                    break # only one eye
                    # cv2.imshow('Output',frame)       
            cv2.imshow('Output',frame)        
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break   


    finally: # 最后一定会执行
        # Stop streaming
        pipeline.stop()


if __name__ == "__main__":
  main()