import os

from PyQt5 import Qt, QtCore

import cv2
import pyrealsense2 as rs

import numpy as np

DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30

class CameraThread(QtCore.QThread):

    change_pixmap = QtCore.pyqtSignal(Qt.QImage)

    def __init__(self, parent=None, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, fps=DEFAULT_FPS):
        super().__init__()
        
        # Declare RealSense pipelineline 
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # start streaming
        self.pipeline.start(config)
        self.running = True
        print("start realsense!")
        
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        self.align = rs.align(rs.stream.color) # align to color frame

        parent.image_captured.connect(self.save_image)
        parent.window_closed.connect(self.stop)

        self.image_counter = 0 # saving images

    def run(self):
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get aligned depth frames
                depth_frame = aligned_frames.get_depth_frame()

                # depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                # depth_frame = frames.get_depth_frame()
                if not depth_frame or not color_frame:
                    continue

                # convert image to numpy arr
                self.depth_image = np.asanyarray(depth_frame.get_data())
                self.color_image = np.asanyarray(color_frame.get_data())

                # depth image must be converted to 8-bit per pixel first
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # Stack both images horizontally
                images = np.hstack((self.color_image, depth_colormap))

                # convert BGR (opencv) to RGB (QImage)
                # ref: https://stackoverflow.com/a/55468544/6622587
                color_image = cv2.cvtColor(images, cv2.COLOR_BGR2RGB)

                h, w, ch = images.shape
                bytesPerLine = ch * w
                color_qimage = Qt.QImage(color_image.data, w, h, bytesPerLine, Qt.QImage.Format_RGB888)
                color_qimage = color_qimage.scaled(640 * 2, 480, QtCore.Qt.KeepAspectRatio)
                self.change_pixmap.emit(color_qimage)

        finally:
            # stop streaming
            # self.pipeline.stop()
            pass

    @Qt.pyqtSlot()
    def stop(self):
        self.running = False
        self.pipeline.stop()
        print("close realsense.")
        self.quit()

    @Qt.pyqtSlot(str)
    def save_image(self, save_dir):
        depth_image_path = os.path.join(save_dir, "frame-%06d.depth.png"%(self.image_counter))
        color_image_path = os.path.join(save_dir, "frame-%06d.color.jpg"%(self.image_counter))
        cv2.imwrite(depth_image_path, self.depth_image)
        cv2.imwrite(color_image_path, self.color_image)
        print('save image: ' + str(self.image_counter))
        self.image_counter += 1