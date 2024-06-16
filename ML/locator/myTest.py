# test the roi and the ellipse
import argparse
import os
import time

import cv2
import numpy as np
import tensorflow.compat.v1 as tf

from config import config
from logger import Logger
from models import Simple, NASNET, Inception, GAP, YOLO
from utils import annotator, change_channel, gray_normalizer, getEllipse

def rescale(image):
    """
    If the input video is other than network size, it will resize the input video
    :param image: a frame form input video
    :return: scaled down frame
    """
    scale_side = max(image.shape)
    # image width and height are equal to 192
    scale_value = config["input_width"] / scale_side

    # scale down or up the input image
    scaled_image = cv2.resize(image, dsize=None, fx=scale_value, fy=scale_value)

    # convert to numpy array
    scaled_image = np.asarray(scaled_image, dtype=np.uint8)

    # one of pad should be zero
    w_pad = int((config["input_width"] - scaled_image.shape[1]) / 2)
    h_pad = int((config["input_width"] - scaled_image.shape[0]) / 2)

    # create a new image with size of: (config["image_width"], config["image_height"])
    new_image = np.ones((config["input_width"], config["input_height"]), dtype=np.uint8) * 250

    # put the scaled image in the middle of new image
    new_image[h_pad:h_pad + scaled_image.shape[0], w_pad:w_pad + scaled_image.shape[1]] = scaled_image

    return new_image

def main():
    video_path = 0
    cap = cv2.VideoCapture(video_path)

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # Exit the loop if there are no more frames

        # Convert to grayscale
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Rescale and normalize if necessary
        # if frame.shape[0] != 192:
        #     frame = rescale(frame)
        # image = gray_normalizer(frame)
        # image = change_channel(image, config["input_channel"])

        #temp
        x, y, w , h, a= 0, 0, 0, 0, 0

        USE_ELLIPSE = True
        h, a = w, 0
        if USE_ELLIPSE:
            # get an ellipse according to the circle
            x, y, w, h, a = getEllipse(frame, x, y, w) 

        # Annotate the frame with the prediction
        # and predict 3d pose here
        labeled_frame = annotator((0, 250, 0), frame, x, y, w, h, a)

        # Display the frame
        cv2.imshow('Detection', labeled_frame)

        # Break the loop with the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()