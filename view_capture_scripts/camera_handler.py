import os
import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
import io
import gphoto2 as gp
import json

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from PIL import Image
from math import atan
from abc import ABC, abstractmethod

class CameraHandler(ABC):
    @abstractmethod
    def is_connected(self):
        return None

    @abstractmethod
    def get_current_rgb_image(self):
        return None

    def load_img(self, img_file_name):
        return cv2.imread(img_file_name)

    def calculate_img_sharpness(self, img):
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        return cv2.Laplacian(gray_img, cv2.CV_64F).var()
    
    def crop_img(self, img, crop_h, crop_w, offset_x, offset_y):

        height = len(img)
        width = len(img[0])
        
        if crop_h is not None and crop_h <= 0:
            raise Exception("Crop height must be larger than 0 pixels")
        
        if crop_w is not None and crop_w <= 0:
            raise Exception("Crop width must be larger than 0 pixels") 

        if crop_h is not None and crop_h < height:
            diff = int((height-crop_h)/2)

            left_row = min(max(diff + offset_y, 0), height)
            right_row = min(max(int(height - diff) + offset_y, 0), height)

            img = img[left_row:right_row, :]

        if crop_w is not None and crop_w < width:
            diff = int((width-crop_w)/2)

            left_col = min(max(diff + offset_x, 0), width)
            right_col = min(max(int(width - diff) + offset_x, 0), width)

            img = img[:, left_col:right_col]

        return img

    def set_camera_properties(self, camera_properties: dict):
        self.camera_properties = camera_properties
    
    def get_camera_properties(self) -> dict:
        return self.camera_properties

class DepthCameraHandler(CameraHandler, ABC):
    @abstractmethod
    def get_current_depth_image(self):
        return None

class ROSCameraHandler(DepthCameraHandler):

    def __init__(self, camera_topic="camera", crop_w=None, crop_h=None):

        self.rgb_image = None
        self.depth_image = None
        self.info = None
        self.time_updated = None
        self.wait_time = 1
        self.num_tries = 3

        self.crop_w = crop_w
        self.crop_h = crop_h

        self.offset_x = 0
        self.offset_y = 0

        self.camera_properties = None

        self.bridge = CvBridge()

        rospy.Subscriber(camera_topic + "/color/image_raw", Image, self.rgb_img_callback)

        rospy.Subscriber(camera_topic + "/aligned_depth_to_color/image_raw", Image, self.depth_img_callback)

        rospy.Subscriber(camera_topic + "/color/camera_info", CameraInfo, self.info_callback)

    def is_connected(self):
        if self.time_updated is not None:
            return True
        
        rospy.sleep(self.wait_time)

        return True if self.time_updated is not None else False

    def rgb_img_callback(self, msg, flip_img=False):
        self.rgb_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)

        if flip_img:
            self.rgb_image = cv2.flip(self.rgb_image, -1)

        self.rgb_image = self.crop_img(self.rgb_image, self.crop_h, self.crop_w, self.offset_x, self.offset_y)

        self.time_updated = rospy.Time.now().to_sec()

    def depth_img_callback(self, msg, flip_img=False):

        converted_image = self.bridge.imgmsg_to_cv2(msg)
        self.depth_image = cv2.convertScaleAbs(converted_image, alpha=0.05) #cv2.applyColorMap(cv2.convertScaleAbs(converted_image, alpha=0.05), cv2.COLORMAP_TURBO)

        if flip_img:
            self.depth_image = cv2.flip(self.depth_image, -1)

        self.depth_image = self.crop_img(self.depth_image, self.crop_h, self.crop_w, self.offset_x, self.offset_y)

        self.time_updated = rospy.Time.now().to_sec()

    def info_callback(self, msg):
        self.info = msg

        self.time_updated = rospy.Time.now().to_sec()
    
    def get_image_width(self):
        if self.info is None:
            return None
        
        return self.info.width

    def get_current_rgb_image(self):
        current_time = rospy.Time.now().to_sec()

        # Attempts to retrieve the camera image, each time checking that the time the 
        # joints were last updated is not earlier than the time the function was executed. 
        # This avoids outdated images being returned
        for i in range(self.num_tries):
            cam_time = self.time_updated

            if cam_time is not None and current_time - cam_time > 0:
                rospy.sleep(self.wait_time)

            return self.rgb_image
        
        return None
    
    def get_current_depth_image(self):
        current_time = rospy.Time.now().to_sec()

        # Attempts to retrieve the camera image, each time checking that the time the 
        # joints were last updated is not earlier than the time the function was executed. 
        # This avoids outdated images being returned
        for i in range(self.num_tries):
            cam_time = self.time_updated

            if cam_time is not None and current_time - cam_time > 0:
                rospy.sleep(self.wait_time)

            return cv2.applyColorMap(self.depth_image, cv2.COLORMAP_BONE)
        
        return None

class RealSenseCameraHandler(DepthCameraHandler):
    context = rs.context()

    def __init__(self, serial_number="", stream_w=1920, stream_h=1080, fps=30, crop_w=None, crop_h=None):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(serial_number)
        self.config.enable_stream(rs.stream.color, stream_w, stream_h, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, min(stream_w, 1280), min(stream_h, 720), rs.format.z16, 6)

        self.profile = self.pipeline.start(self.config)

        self.serial_number = serial_number

        self.crop_h = crop_h
        self.crop_w = crop_w

        self.offset_x = 0
        self.offset_y = 0

    def is_connected(self):
        devices = RealSenseCameraHandler.context.query_devices()

        for device in devices:
            if device.get_info(rs.camera_info.serial_number) == self.serial_number:
                return True
        
        return False

    def get_current_rgb_image(self):

        frames = self.pipeline.wait_for_frames()
        
        colour_frame = frames.get_color_frame()

        colour_data = np.asanyarray(colour_frame.get_data())

        return self.crop_img(colour_data, self.crop_h, self.crop_w, self.offset_x, self.offset_y)
    
    def get_current_depth_image(self):

        frames = self.pipeline.wait_for_frames()
        
        depth_frame = frames.get_depth_frame()

        depth_data = np.array(depth_frame.get_data())

        depth_data = depth_data.astype(np.uint16)

        return self.crop_img(depth_data, self.crop_h, self.crop_w, self.offset_x, self.offset_y)


class GPhotoCameraHandler(CameraHandler):
    def __init__(self, cam_name="", crop_w=None, crop_h=None):
        self.cam_name = cam_name
        self.is_connected = False

        connected_cams = gp.Camera.autodetect(context)

        camera_addr = None

        for name, addr in connected_cams:
            if self.cam_name in name:
                camera_addr = addr
                continue

        if not camera_addr:
            raise Exception(f"Unable to connect to camera with name: {self.cam_name}")

        self.context = gp.Context()
        self.camera = gp.Camera()
        port_info_list = gp.PortInfoList()
        port_info_list.load()
        idx = port_info_list.lookup_path(camera_addr)
        camera.set_port_info(port_info_list[idx])
        self.camera.init()

        self.crop_h = crop_h
        self.crop_w = crop_w

        self.offset_x = 0
        self.offset_y = 0

        self.is_connected = True

    def is_connected(self):
        return self.is_connected

    def get_current_rgb_image(self):

         # Capture an image
        file_path = camera.capture(gp.GP_CAPTURE_IMAGE)
        print(f'Image captured and stored at: {file_path.folder}/{file_path.name}')

        # Download the image
        camera_file = camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL)
        file_data = gp.check_result(gp.gp_file_get_data_and_size(camera_file))

        image = Image.open(io.BytesIO(file_data))
        colour_data = np.array(image)

        return self.crop_img(colour_data, self.crop_h, self.crop_w, self.offset_x, self.offset_y)

def get_camera_handler(camera_handler, camera_topic="", serial_number="", stream_w=1920, stream_h=1080, fps=30, 
                       crop_w=1920, crop_h=1080):
    if camera_handler == "ros":
        return ROSCameraHandler(camera_topic=camera_topic, crop_w=crop_w, crop_h=crop_h)
    elif camera_handler == "realsense":
        return RealSenseCameraHandler(serial_number=serial_number, stream_w=stream_w, stream_h=stream_h, fps=fps, crop_w=crop_w, crop_h=crop_h)
    elif camera_handler == "gphoto":
        return GPhotoCameraHandler(cam_name=serial_number, crop_w=crop_w, crop_h=crop_h)

        """ ADD YOUR METHOD HERE """
    else:
        raise Exception(f"Unknown camera handler type: {camera_handler}")


def load_camera_properties(camera_file_path): 

    if not os.path.isfile(camera_file_path):
        raise Exception("Unable to locate cameras file in config directory")
    
    cam_info = {"k1": 0, "k2": 0, "k3": 0, "k4": 0, "p1": 0, "p2": 0, "is_fisheye": False}

    with open(camera_file_path, "r") as f:

        cam_values = json.load(f)

        try:
            cam_info["w"] = cam_values["w"]
            cam_info["h"] = cam_values["h"]
            cam_info["fl_x"] = cam_values["fl_x"]
            cam_info["fl_y"] = cam_values["fl_y"]
        except Exception:
            raise Exception("Width (w), Height (h) and focal lengths (fl_x & fl_y) must be included in the camera properties")

        cam_info["cx"] = cam_info["w"]/2
        cam_info["cy"] = cam_info["h"]/2

        for cam_info_key in cam_info.keys():
            if cam_info_key in cam_values.keys():
                cam_info[cam_info_key] = cam_values[cam_info_key]

        cam_info["camera_angle_x"] = atan(cam_info["w"] / (cam_info["fl_x"] * 2)) * 2
        cam_info["camera_angle_y"] = atan(cam_info["h"] / (cam_info["fl_y"] * 2)) * 2

        return cam_info