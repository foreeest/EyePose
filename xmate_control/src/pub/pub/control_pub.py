from multiprocessing import Array, Process
from pynput.keyboard import Key, Listener
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray  # 使用已有的消息类型或你的自定义消息类型
import time
import numpy as np
import cv2
import pyrealsense2 as rs

from pub.utils.geometry import fit_rotated_ellipse_ransac, ellipse_to_limbus
# module.submodule.filename

Ctrl_Num = 7  # 控制方向数量
Recive_Num = 13
ctype = 'f'
cbyte = 'b'
# data = [1.1, 2.2, 3.3, 4.4, 5.5]

shm = Array(ctype, Ctrl_Num) # 共享内存，用于发送
shm_recive = Array(ctype, Recive_Num)
temp_share_memory = Array(ctype, Ctrl_Num) # 暂时用来放一下视觉模块输出

def on_press(key):
    try:
        # print(f'字母键 {key.char} 被按下')
        if key.char == "w": # x+
            shm[0] = 1
        if key.char == "s": # x-
            shm[0] = -1
        if key.char == "a": # y+
            shm[1] = 1
        if key.char == "d": # y-
            shm[1] = -1
        if key.char == "q": # z+
            shm[2] = 1
        if key.char == "e": # z-
            shm[2] = -1
          
        if key.char == "j": # alpha+
            shm[3] = 1
        if key.char == "l": # alpha-
            shm[3] = -1
        if key.char == "i": # beta+
            shm[4] = 1
        if key.char == "k": # beta-
            shm[4] = -1
        if key.char == "u": # gamma+
            shm[5] = 1
        if key.char == "o": # gamma-
            shm[5] = -1
    except AttributeError:
        # print(f'特殊键 {key} 被按下')
        print("No control keys")

def on_release(key):
    # print(f'{key} 被释放')
    try:
        if key.char == "w" or key.char == "s": # x0
            shm[0] = 0
        if key.char == "a" or key.char == "d": # y0
            shm[1] = 0
        if key.char == "q" or key.char == "e": # z0
            shm[2] = 0
        if key.char == "j" or key.char == "l": # alpha0
            shm[3] = 0
        if key.char == "i" or key.char == "k": # beta0
            shm[4] = 0
        if key.char == "u" or key.char == "o": # gamma0
            shm[5] = 0
    except AttributeError:
        if key == Key.esc: # 退出
            shm[6] = 1
            time.sleep(2)
            print("检测到ESC按键，终止键盘监听")
            # 停止监听
            return False

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        # 创建发布者
        self.publisher_ = self.create_publisher(Float64MultiArray, 'control_msg', 10)
        # 创建共享内存区域（示例中使用列表作为共享内存）
        self.shared_memory = [0.0] * 7
        self.recive_shared_memory = [0.0] * 13
        # 创建一个线程用于发布消息
        self.thread = threading.Thread(target=self.publish_loop)
        # 启动线程
        self.thread.start()

        # 创建订阅者并注册回调函数
        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            'state_msg',
            self.subscription_callback,
            10
        )

    def publish_loop(self):
        # 持续运行的发布循环
        while rclpy.ok():
            # 从共享内存中读取数据
            for i in range(7):
                self.shared_memory[i]=shm[i]
            data_to_publish = self.shared_memory[:]
            # 创建消息
            msg = Float64MultiArray()
            msg.data = data_to_publish
            # 发布消息
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing: %s' % msg.data)
            # 休眠一段时间
            time.sleep(0.05) # maybe writen with timer

    def subscription_callback(self, msg):
        # 这里处理接收到的消息
        # 锁定互斥锁，更新共享内存
        self.recive_shared_memory[:] = msg.data
        for i in range(13):
            shm_recive[i] = self.recive_shared_memory[i]

    def destroy_node(self):
        # 确保线程结束
        self.thread.join()
        super().destroy_node()

# p1: 启动键盘监听
def start_listen():
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

# p2: print to terminal
def print_terminal():
    while (shm[6] == 0):
        print("shm(to publish):", shm[0], shm[1], shm[2], shm[3], shm[4], shm[5], )
        print("shm(vision output):", temp_share_memory[0], temp_share_memory[1], temp_share_memory[2], temp_share_memory[3], temp_share_memory[4], temp_share_memory[5], )
        print("shm_recieve(recieved):", shm_recive[0], shm_recive[1], shm_recive[2], shm_recive[3], shm_recive[4], shm_recive[5], )
        time.sleep(2)

def publish_shm():
    pass # what is this?  

# p3: 视觉模块
def get_pose_from_realsense():
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

                    limbus, flag = ellipse_to_limbus(cx, cy, w*2, h*2, theta, True) # 50个点返回一次, w和h要输入直径
                    if flag:
                        postion = limbus[0]
                        direction = limbus[1]
                        # write
                        for i in range(3):
                            temp_share_memory[i] = postion[i] # x y z
                            temp_share_memory[i+3] = direction[i] # a b g
                        # print(f"predicted postion is {postion}")
                        # print(f"predicted direction is {direction}")

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

def get_pose_from_camera():
    cap = cv2.VideoCapture(0)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

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

                    limbus, flag = ellipse_to_limbus(cx, cy, w*2, h*2, theta, True) # 50个点返回一次, w和h要输入直径
                    if flag:
                        postion = limbus[0]
                        direction = limbus[1]
                        # write
                        for i in range(3):
                            temp_share_memory[i] = postion[i] # x y z
                            temp_share_memory[i+3] = direction[i] # a b g
                        # print(f"predicted postion is {postion}")
                        # print(f"predicted direction is {direction}")

                    cv2.ellipse(frame,(int(cx),int(cy)),(int(w),int(h)),theta*180.0/np.pi,0.0,360.0,(0,255,0),1)
                    cv2.drawMarker(frame, (int(cx),int(cy)),(0, 0, 255),cv2.MARKER_CROSS,2,1)
                    break # only one eye
                    # cv2.imshow('Output',frame)       
            cv2.imshow('Output',frame)        
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break   

    finally: # 最后一定会执行
        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    # 并行运行
    # p1持续监听键盘; p2 2秒打印一次几个pose； p3 持续从摄像头获取信息； 
    # 主线程不断发布与订阅
    # read from keyboard and write shm
    p1 = Process(target=start_listen)
    # 用于终端输出
    p2 = Process(target=print_terminal) 
    # 视觉子线程, 获取眼球pose
    p3 = Process(target=get_pose_from_realsense)
    # p3 = Process(target=get_pose_from_camera)
    p1.start()
    p2.start()
    p3.start()

    # 主循环
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()
    p1.join()
    p2.join()
    p3.join()

    print('程序结束')


if __name__ == '__main__':
    main()