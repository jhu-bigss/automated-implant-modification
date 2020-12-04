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

    def __init__(self, parent=None, stream_type=rs.stream.color, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, fps=DEFAULT_FPS):
        super().__init__()
        
        # Declare RealSense pipelineline 
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(stream_type, width, height, rs.format.bgr8, fps)
        
        # start streaming
        self.pipeline.start(config)
        self.running = True
        print("start realsense!")
        
        parent.image_captured.connect(self.save_image)
        parent.window_closed.connect(self.stop)

        self.image_counter = 0 # saving images

    def run(self):
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                # depth_frame = frames.get_depth_frame()
                if not color_frame:
                    continue

                # convert image to numpy arr
                self.color_image = np.asanyarray(color_frame.get_data())

                # convert BGR (opencv) to RGB (QImage)
                # ref: https://stackoverflow.com/a/55468544/6622587
                color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

                h, w, ch = color_image.shape
                bytesPerLine = ch * w
                color_qimage = Qt.QImage(color_image.data, w, h, bytesPerLine, Qt.QImage.Format_RGB888)
                color_qimage = color_qimage.scaled(640, 480, QtCore.Qt.KeepAspectRatio)
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
        image_name_absolute_path = os.path.join(save_dir, str(self.image_counter) + '.png')
        cv2.imwrite(image_name_absolute_path, self.color_image)
        print('image saved: ' + str(self.image_counter) + '.png')
        self.image_counter += 1