import os

from PyQt5 import Qt, QtCore

import cv2
import pyrealsense2 as rs
from primesense import openni2
from primesense import _openni2 as c_api

import numpy as np

## Path of the OpenNI redistribution OpenNI2.so or OpenNI2.dll
# Windows
#dist = 'C:\Program Files\OpenNI2\Redist\OpenNI2.dll'
# Linux
dist = os.path.expanduser("~/OpenNI-Linux-x64-2.2/Redist")

clipping_distance = 1000 # mm

class CameraRealsense(QtCore.QThread):

    change_pixmap = QtCore.pyqtSignal(Qt.QImage)

    def __init__(self, parent=None, width=1280, height=720, fps=30):
        super().__init__()
        
        # Check if any availiable device is connected
        realsense_ctx = rs.context()
        connected_devices = []
        if len(realsense_ctx.devices) == 0:
            print("No RealSense device is detected. Therefore, no camera thread started.")
            self.running = False
            return

        # Declare RealSense pipelineline 
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # start streaming
        profile = self.pipeline.start(config)
        self.view_mode_bg_removed = True
        self.running = True
        print("start realsense!")

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale() * 1.017 # Correct depth scale
        # print("Depth Scale is: " , self.depth_scale)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        self.align = rs.align(rs.stream.color) # align to color frame

        parent.view_mode_changed.connect(self.change_view_mode)
        parent.set_save_dir.connect(self.set_save_dir)
        parent.image_capture.connect(self.save_image)
        parent.close_window.connect(self.stop)

        self.save_dir = parent.data_directory
        self.image_counter = 0 # saving images

    def run(self):
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get aligned depth frames
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # convert image to numpy arr
                self.depth_image = (np.asanyarray(depth_frame.get_data()) * self.depth_scale * 1000).astype(np.uint16)
                self.color_image = np.asanyarray(color_frame.get_data())

                if self.view_mode_bg_removed:
                    # Remove background - Set pixels further than clipping_distance to grey
                    # We will be removing the background of objects more than clipping_distance milimeters away
                    grey_color = 153
                    depth_image_3d = np.dstack((self.depth_image, self.depth_image, self.depth_image)) #depth image is 1 channel, color is 3 channels
                    images = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, self.color_image)
                else:
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
                if self.view_mode_bg_removed:
                    color_qimage = color_qimage.scaled(640, 480, QtCore.Qt.KeepAspectRatio)
                else:
                    color_qimage = color_qimage.scaled(640 * 2, 480, QtCore.Qt.KeepAspectRatio)
                    
                self.change_pixmap.emit(color_qimage)

        finally:
            # stop streaming
            # self.pipeline.stop()
            pass

    @Qt.pyqtSlot(bool)
    def change_view_mode(self, view_mode):
        self.view_mode_bg_removed = view_mode

    @Qt.pyqtSlot(str)
    def set_save_dir(self, dir):
        self.save_dir = dir

    @Qt.pyqtSlot()
    def save_image(self):
        depth_image_path = os.path.join(self.save_dir, "frame-%06d.depth.png"%(self.image_counter))
        color_image_path = os.path.join(self.save_dir, "frame-%06d.color.jpg"%(self.image_counter))
        cv2.imwrite(depth_image_path, self.depth_image)
        cv2.imwrite(color_image_path, self.color_image)
        print('save image: ' + str(self.image_counter))
        self.image_counter += 1

    @Qt.pyqtSlot()
    def stop(self):
        self.running = False
        self.pipeline.stop()
        print("close realsense.")
        self.quit()

class CameraPrimesense(QtCore.QThread):

    change_pixmap = QtCore.pyqtSignal(Qt.QImage)

    def __init__(self, parent=None, width=1280, height=720, fps=30):
        super().__init__()
        
        ## initialize openni and check
        openni2.initialize(dist) #'C:\Program Files\OpenNI2\Redist\OpenNI2.dll') # accepts the path of the OpenNI redistribution
        if (openni2.is_initialized()):
            print( "openNI2 initialized" )
        else:
            print( "openNI2 not initialized" )

        ## Register the device
        dev = openni2.Device.open_any()

        ## create the streams stream
        self.rgb_stream = dev.create_color_stream()
        self.depth_stream = dev.create_depth_stream()

        self.rgb_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX=640, resolutionY=480, fps=30))

        ##configure the depth_stream
        #print( 'Get b4 video mode', depth_stream.get_video_mode()
        self.depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, resolutionX=640, resolutionY=480, fps=30))

        ## Check and configure the mirroring -- default is True
        # print( 'Mirroring info1', depth_stream.get_mirroring_enabled()
        self.rgb_stream.set_mirroring_enabled(False)
        self.depth_stream.set_mirroring_enabled(False)

        ## start the stream
        self.rgb_stream.start()
        self.depth_stream.start()

        ## synchronize the streams
        dev.set_depth_color_sync_enabled(True) # synchronize the streams

        ## IMPORTANT: ALIGN DEPTH2RGB (depth wrapped to match rgb stream)
        dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)

        parent.view_mode_changed.connect(self.change_view_mode)
        parent.image_capture.connect(self.save_image)
        parent.close_window.connect(self.stop)

        self.view_mode_bg_removed = False
        self.image_counter = 0 # saving images

        self.running = True
        print("start primesense!")

    def get_rgb(self):
        """
        Returns numpy 3L ndarray to represent the rgb image.
        """
        bgr   = np.fromstring(self.rgb_stream.read_frame().get_buffer_as_uint8(),dtype=np.uint8).reshape(480,640,3)
        rgb   = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        return rgb

    def get_depth(self):
        """
        Returns numpy ndarrays representing the raw and ranged depth images.
        Outputs:
            dmap:= distancemap in mm, 1L ndarray, dtype=uint16, min=0, max=2**12-1
            d4d := depth for dislay, 3L ndarray, dtype=uint8, min=0, max=255    
        Note1: 
            fromstring is faster than asarray or frombuffer
        Note2:     
            .reshape(120,160) #smaller image for faster response 
                    OMAP/ARM default video configuration
            .reshape(240,320) # Used to MATCH RGB Image (OMAP/ARM)
                    Requires .set_video_mode
        """
        dmap = np.fromstring(self.depth_stream.read_frame().get_buffer_as_uint16(),dtype=np.uint16).reshape(480,640)  # Works & It's FAST
        d4d = np.uint8(dmap.astype(float) *255/ 2**12-1) # Correct the range. Depth images are 12bits
        d4d = 255 - cv2.cvtColor(d4d,cv2.COLOR_GRAY2RGB)
        return dmap, d4d

    def run(self):
        try:
            while self.running:
                # Get rgb and aligned depth frames
                self.color_image = self.get_rgb()
                self.depth_image, depth_for_display = self.get_depth()

                if self.view_mode_bg_removed:
                    # Remove background - Set pixels further than clipping_distance to grey
                    grey_color = 153
                    depth_image_3d = np.dstack((self.depth_image, self.depth_image, self.depth_image)) #depth image is 1 channel, color is 3 channels
                    images = np.where((depth_image_3d > 1000 * clipping_distance_in_meters) | (depth_image_3d <= 0), grey_color, self.color_image)
                else:
                    # Stack both images horizontally
                    images = np.hstack((self.color_image, depth_for_display))

                # convert BGR (opencv) to RGB (QImage)
                # ref: https://stackoverflow.com/a/55468544/6622587
                color_image = cv2.cvtColor(images, cv2.COLOR_BGR2RGB)

                h, w, ch = images.shape
                bytesPerLine = ch * w
                color_qimage = Qt.QImage(color_image.data, w, h, bytesPerLine, Qt.QImage.Format_RGB888)
                if self.view_mode_bg_removed:
                    color_qimage = color_qimage.scaled(640, 480, QtCore.Qt.KeepAspectRatio)
                else:
                    color_qimage = color_qimage.scaled(640 * 2, 480, QtCore.Qt.KeepAspectRatio)
                    
                self.change_pixmap.emit(color_qimage)

        finally:
            # stop streaming
            # self.pipeline.stop()
            pass

    @Qt.pyqtSlot(bool)
    def change_view_mode(self, view_mode):
        self.view_mode_bg_removed = view_mode

    @Qt.pyqtSlot(str)
    def save_image(self, save_dir):
        depth_image_path = os.path.join(save_dir, "frame-%06d.depth.png"%(self.image_counter))
        color_image_path = os.path.join(save_dir, "frame-%06d.color.jpg"%(self.image_counter))
        cv2.imwrite(depth_image_path, self.depth_image)
        cv2.imwrite(color_image_path, self.color_image)
        print('save image: ' + str(self.image_counter))
        self.image_counter += 1

    @Qt.pyqtSlot()
    def stop(self):
        self.running = False
        self.rgb_stream.stop()
        self.depth_stream.stop()
        openni2.unload()
        print("close primesense.")
        self.quit()