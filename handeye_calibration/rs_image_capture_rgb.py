"""
OpenCV capture images with RealSense camera
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import datetime
import argparse

parser = argparse.ArgumentParser(description='RealSense Camera Calibration')
parser.add_argument('mode', type=int, help='0 - 640x480, 1 - 1920x1080')
args = parser.parse_args()

if args.mode == 0:
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
else:
    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1080

# create a directory to save captured images 
def create_image_directory():
    now = datetime.datetime.now()
    dir_str = now.strftime("%Y-%m-%d-%H%M%S")
    try:
        if not(os.path.isdir(dir_str)):
            os.makedirs(os.path.join(dir_str))
    except OSError as e:
        print("Can't make the directory: %s" % image_dir)
        raise
    return dir_str

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

image_dir = create_image_directory()

print("press SPACE to capture an image or press Esc to exit...")

image_counter = 0

try:
    while(True):
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # display the captured image
        cv2.imshow('Capture Images',color_image)
        pressedKey = (cv2.waitKey(1) & 0xFF)

        # handle key inputs
        if pressedKey == 27:
            break
        elif pressedKey == 32:
            cv2.imwrite(os.path.join(image_dir, str(image_counter) + '.jpg'), color_image)
            print('Image caputured - ' + os.path.join(image_dir, str(image_counter) + '.jpg'))

            image_counter+=1
finally:
    # Stop streaming
    pipeline.stop()

cv2.destroyAllWindows()