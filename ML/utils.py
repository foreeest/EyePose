import os

import cv2
import numpy as np
from geometry import ellipse_to_limbus

from config import config


def check_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def rf(low, high):
    """
    return a random float number between [low, high)
    :param low: lower bound
    :param high: higher bound (excluded)
    :return: a float number between [low, high)
    """
    if low >= high:
        return low
    return np.random.uniform(low, high)


def ri(low, high):
    """
    return a random int number between [low, high)
    :param low: lower bound
    :param high: higher bound (excluded)
    :return: an int number between [low, high)
    """
    if low >= high:
        return low
    return np.random.randint(low, high)

# 2 helper function
# 最小二乘法拟合椭圆
def fit_rotated_ellipse(data):

    xs = data[:,0].reshape(-1,1) 
    ys = data[:,1].reshape(-1,1)

    J = np.mat( np.hstack((xs*ys,ys**2,xs, ys, np.ones_like(xs,dtype=np.float32))) )
    Y = np.mat(-1*xs**2)
    P= (J.T * J).I * J.T * Y

    a = 1.0; b= P[0,0]; c= P[1,0]; d = P[2,0]; e= P[3,0]; f=P[4,0];
    theta = 0.5* np.arctan(b/(a-c))  
    
    cx = (2*c*d - b*e)/(b**2-4*a*c)
    cy = (2*a*e - b*d)/(b**2-4*a*c)

    cu = a*cx**2 + b*cx*cy + c*cy**2 -f
    w= np.sqrt(cu/(a*np.cos(theta)**2 + b* np.cos(theta)*np.sin(theta) + c*np.sin(theta)**2))
    h= np.sqrt(cu/(a*np.sin(theta)**2 - b* np.cos(theta)*np.sin(theta) + c*np.cos(theta)**2))

    ellipse_model = lambda x,y : a*x**2 + b*x*y + c*y**2 + d*x + e*y + f

    error_sum = np.sum([ellipse_model(x,y) for x,y in data])
    # print('fitting error = %.3f' % (error_sum))

    return (cx,cy,w,h,theta)

# 迭代找出比较好的代表点
def fit_rotated_ellipse_ransac(data,iter=50,sample_num=10,offset=80.0):

    count_max = 0
    effective_sample = None

    # 如何理解？
    for i in range(iter):
        sample = np.random.choice(len(data), sample_num, replace=False)

        xs = data[sample][:,0].reshape(-1,1) # x 的行
        ys = data[sample][:,1].reshape(-1,1) # y 的行

        J = np.mat( np.hstack((xs*ys,ys**2,xs, ys, np.ones_like(xs,dtype=np.float32))) )
        Y = np.mat(-1*xs**2)
        P= (J.T * J).I * J.T * Y

        # fitter a*x**2 + b*x*y + c*y**2 + d*x + e*y + f = 0
        a = 1.0; b= P[0,0]; c= P[1,0]; d = P[2,0]; e= P[3,0]; f=P[4,0];
        ellipse_model = lambda x,y : a*x**2 + b*x*y + c*y**2 + d*x + e*y + f

        # threshold 
        ran_sample = np.array([[x,y] for (x,y) in data if np.abs(ellipse_model(x,y)) < offset ])

        if(len(ran_sample) > count_max):
            count_max = len(ran_sample) 
            effective_sample = ran_sample

    return fit_rotated_ellipse(effective_sample)

# model输出是一个圆，现在根据这圆，在其附近重新寻找一个椭圆
def getEllipse(img, x, y, w):
    # 截取一个圆附近的区域，注意不要超出图像边界
    # 虹膜r：区域宽：区域高 = 1:5:3
    height, width = img.shape[:2]
    start_point_x = min(x - w * 2.5, 0)
    start_point_y = min(y - w * 1.5, 0)
    roi_w = min(w * 5, width - start_point_x)
    roi_h = min(w * 3, height - start_point_y)
    # eye_roi = img[start_point_y:start_point_y+roi_h, start_point_x:start_point_x+roi_w]
    eye_roi = img

    # nystagmus 方法
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    blur = cv2.GaussianBlur(eye_roi,(3,3),0)
    ret, thresh1 = cv2.threshold(blur,50,255,cv2.THRESH_BINARY) # 二值化
    opening = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    image = 255 - closing
    # 凸包，这想法不错
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    hull = [] # 每个元素是一个凸包点集
    for i in range(len(contours)):
            hull.append(cv2.convexHull(contours[i], False)) 

    # cx, cy, w, h, a = x, y, w, w, 0                  
    # for con in hull:
    #     approx = cv2.approxPolyDP(con, 0.01 * cv2.arcLength(con,True),True)
    #     area = cv2.contourArea(con)
    #     if(len(approx) > 10 and area > 1000): # 过滤一些凸包
    #         cx, cy, w, h, a = fit_rotated_ellipse_ransac(con.reshape(-1,2)) # 变成一个二维数组
    #         break # 只取一个

    # 更Robust，取面积最大的、合法的凸包
    cx, cy, w, h, a = x, y, 10, 10, 0 
    cnt = sorted(hull, key=cv2.contourArea)
    if len(cnt) == 0: # 无凸包
        return x, y, 10, 10, 0 # 图里没有就先随便返回一个
    maxcnt = cnt[-1]
    HAVE_HULL = False # 标记是否找到合法凸包
    for i in range(len(hull) - 1, -1, -1):
        maxcnt = cnt[i]
        approx = cv2.approxPolyDP(maxcnt, 0.01 * cv2.arcLength(maxcnt,True),True)
        area = cv2.contourArea(maxcnt)
        if (len(approx) > 10 and area > 1000):
            HAVE_HULL = True
            break
    if HAVE_HULL:
        cx, cy, w, h, a = fit_rotated_ellipse_ransac(maxcnt.reshape(-1,2)) # --> n * 2
    else:
        return x, y, 10, 10, 0

    # 可能存在无效值
    if w <= 0 or h <= 0 or w < h: # 长轴一定要更长, w>=h才合法
        return x, y, 10, 10, 0

    return cx, cy, w, h, a

def annotator(color, img, x, y, w=10, h=None, a=0):
    """
    draw a circle around predicted pupil
    :param img: input frame
    :param x: x-position
    :param y: y-position
    :param w: width of pupil
    :param h: height of pupil
    :return: an image with a circle around the pupil
    """
    if color is None:
        color = (0, 250, 250)

    c = 1
    if np.ndim(img) == 2:
        img = np.expand_dims(img, -1)
    elif np.ndim(img) == 3:
        c = img.shape[2]

    if c == 1:
        img = np.concatenate((img, img, img), axis=2)

    l1xs = int(x - 3)
    l1ys = int(y)
    l1xe = int(x + 3)
    l1ye = int(y)

    l2xs = int(x)
    l2ys = int(y - 3)
    l2xe = int(x)
    l2ye = int(y + 3)

    img = cv2.line(img, (l1xs, l1ys), (l1xe, l1ye), color, 1)
    img = cv2.line(img, (l2xs, l2ys), (l2xe, l2ye), color, 1)

    # We predict only width!
    if h is None:
        h = w

    limbus, flag = ellipse_to_limbus(x, y, w, h, a, True)
    if flag:
        postion = limbus[0]
        direction = limbus[1]
        print(f"predicted postion is {postion}")
        print(f"predicted direction is {direction}")

    # draw ellipse
    img = cv2.ellipse(img, ((x, y), (w, h), a), color, 1)

    return img


def create_noisy_video(data_path='data/valid_data.csv', length=60, fps=5, with_label=False, augmentor=None):
    """
    create a sample video based random image.
    Of course it is not a valid solution to test the model with already seen images.
    It is just to check the speed of model. based on different FPS
    :param data_path: CSV file for input data
    :param length: length of video in second
    :param fps: number of frame per second
    :param with_label: if true, show true label on the video
    :return: a noisy video (file name) for test purpose.
    """

    # read CSV
    data_list = []
    with open(data_path, "r") as f:
        for line in f:
            #  values: [ img_path, x, y, w, h , a]
            values = line.strip().split(",")
            data_list.append([values[0],  # image path
                              values[1],  # x
                              values[2]])  # y

    # number image to make the video
    images_len = fps * length
    np.random.shuffle(data_list)
    start_idx = np.random.randint(0, len(data_list) - images_len)
    selected_images = data_list[start_idx:start_idx + images_len]

    output_fn = 'video_{}s_{}fps.avi'.format(length, fps)
    video = cv2.VideoWriter(output_fn, cv2.VideoWriter_fourcc(*"XVID"), fps,
                            (config["input_height"], config["input_width"]))

    for i in selected_images:
        img = cv2.imread(i[0], cv2.IMREAD_GRAYSCALE)
        x = float(i[1])
        y = float(i[2])
        # w = float(i[3])
        # h = float(i[4])
        # a = float(i[5])
        label = [x, y]
        if augmentor is not None:
            img, label = augmentor.addNoise(img, label)
            img = np.asarray(img, dtype=np.uint8)

        if with_label:
            img = annotator((0, 250, 0), img, *label)
            font = cv2.FONT_HERSHEY_PLAIN
            texts = i[0].split("/")
            text = texts[2] + "/" + texts[3] + "/" + texts[4]
            img = cv2.putText(img, text, (5, 10), font, 0.8, (0, 250, 0), 1, cv2.LINE_8)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        video.write(img)

    cv2.destroyAllWindows()
    video.release()

    return output_fn


def change_channel(img, num_channel=1):
    """
    Get frame and normalize values between 0 and 1 and then based num channel reshape it to desired channel
    :param frame: the input image, a numpy array
    :param num_channel: desired number of channel
    :return: normalized frame with num_channel
    """
    img = np.expand_dims(img, -1)
    if num_channel == 3:
        img = np.concatenate((img, img, img), axis=2)

    return img


def gray_normalizer(gray):
    """
    get a grayscale image with pixel value 0-255
    and return normalized pixel with value between -1,1
    :param gray: input grayscale image
    :return: normalized grayscale image
    """
    # average mean over all training images ( without noise)
    gray = gray * 1/255
    out_gray = np.asarray(gray - 0.5, dtype=np.float32)
    return out_gray


def gray_denormalizer(gray):
    """
    Get a normalized gray image and convert to value 0-255
    :param gray: normalized grayscale image
    :return: denormalized grayscale image
    """
    # average mean over all training images ( without noise)
    out_gray = gray + 0.5
    out_gray = np.asarray(out_gray * 255, dtype=np.uint8)

    return out_gray


def save_dict(dict, save_path):
    with open(save_path, mode="w") as f:
        for key, val in dict.items():
            f.write(key+";"+str(val)+"\n")
    print("Class dict saved successfully at: {}".format(save_path))


def load_dict(load_path):
    dict = {}
    with open(load_path, mode="r") as f:
        for line in f:
            key, val = line.split(";")
            dict[key] = int(val)

    print("Class dict loaded successfuly at: {}".format(load_path))
    return dict

if __name__ == "__main__":
    ag = Augmentor('data/noisy_videos', config)
    create_noisy_video(with_label=True, augmentor=ag)