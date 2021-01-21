import os

from PyQt5 import Qt, QtCore

import cv2
import pyrealsense2 as rs

import numpy as np

class CameraThread(QtCore.QThread):

    change_pixmap = QtCore.pyqtSignal(Qt.QImage)

    def __init__(self, parent=None, width=1920, height=1080, fps=30):
        super().__init__()
        
        # Declare RealSense pipelineline 
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # start streaming
        self.pipeline.start(config)
        self.running = True
        print("start realsense with %d x %d at %d Hz" % (width, height, fps) )
        
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
        color_image_path = os.path.join(save_dir, "frame-%06d.color.jpg"%(self.image_counter))
        cv2.imwrite(color_image_path, self.color_image)
        print('image saved: ' + str(self.image_counter))
        self.image_counter += 1