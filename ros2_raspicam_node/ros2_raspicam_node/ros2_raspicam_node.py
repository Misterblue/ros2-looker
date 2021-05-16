# Copyright 2021 Robert Adams
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import queue
import threading
import time
import traceback
import sys

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge

import cv2

class ROS2_raspicam_node(Node):

    def __init__(self):
        super().__init__('ros2_raspicam_node', namespace='raspicam')

        self.set_parameter_defaults( [
            ('compressed_image', Parameter.Type.BOOL, False),
            ('image_topic', Parameter.Type.STRING, 'raspicam_uncompressed'),
            ('image_topic_qos', Parameter.Type.INTEGER, 10),
            ('compressed_image_topic', Parameter.Type.STRING, 'raspicam_compressed'),
            ('compressed_image_topic_qos', Parameter.Type.INTEGER, 10),

            ('camera_frame_rate', Parameter.Type.INTEGER, 2),
            ('camera_image_width', Parameter.Type.INTEGER, 640),
            ('camera_image_height', Parameter.Type.INTEGER, 480),
            # Saturation: -100..100, default 0
            ('camera_awb_mode', Parameter.Type.BOOL, True),
            ('camera_wb_temp', Parameter.Type.INTEGER, 2700),
            # ('camera_wb_red', Parameter.Type.INTEGER, 128),
            # ('camera_wb_green', Parameter.Type.INTEGER, 128),
            # ('camera_wb_blue', Parameter.Type.INTEGER, 128),
            # brightness: 1..100, default 50
            ('camera_brightness', Parameter.Type.INTEGER, 55),
            # Contrast: -100..100, default 0
            ('camera_contrast', Parameter.Type.INTEGER, 0),
            ('camera_hflip', Parameter.Type.BOOL, False),
            ('camera_vflip', Parameter.Type.BOOL, True),

            # ('camera_exif_copyright', Parameter.Type.STRING, 'Copyrightt 2018 MY NAME'),
            # ('camera_user_comment', Parameter.Type.STRING, 'SOMETHING INFORMATIVE'),
            # Exposure compenstation: -25..25, default 0, one step = 1/6 F-stop
            # ('camera_exposure_compenstation', Parameter.Type.INTEGER, 0),
            # off, auto, night, backlight, spotlight, sports, snow, beach, antishake, fireworks
            # ('camera_exposure_mode', Parameter.Type.STRING, 'auto'),
            # the camera is upside down in initial setup
            # ('camera_hflip', Parameter.Type.BOOL, True),
            # ('camera_vflip', Parameter.Type.BOOL, True),
            # 'none', 'negative', 'solarize', 'sketch', 'denoise', 'emboss', 'oilpaint',
            # 'hatch', 'gpen', 'pastel', 'watercolor', 'film', 'blur', 'saturation',
            # 'colorswap', 'washedout', 'posterise', 'colorpoint', 'colorbalance', 'cartoon', 'deinterlace1',
            # 'deinterlace2'
            # ('camera_image_effect', Parameter.Type.STRING, 'none'),
            # 'average' 'spot' 'backlit' 'matrix'
            # ('camera_meter_mode', Parameter.Type.STRING, 'average'),
            # 640/480, 800/600, 1280/720
            # ('camera_saturation', Parameter.Type.INTEGER, 0),
            # Sharpness: -100..100, default 0
            # ('camera_sharpness', Parameter.Type.INTEGER, 10),
            ] )

        self.keepRunning = True

        self.camera = cv2.VideoCapture(0)
        time.sleep(1);  # let camera initialization complete

        self.bridge = CvBridge()

        self.initialize_publisher()
        self.set_camera_parameters()
        self.initialize_capture_queue()

    def destroy_node(self):
        # overlay Node function called when class is being stopped and camera needs closing
        # if hasattr(self, 'publisher') and self.publisher != None:
        #     # nothing to do
        if hasattr(self, 'camera') and self.camera != None:
            self.camera.release()
        super().destroy_node()

    def initialize_publisher(self):
        if self.get_parameter_value('compressed_image'):
            self.publisher = self.create_publisher(CompressedImage,
                                self.get_parameter_value('compressed_image_topic'),
                                self.get_parameter_value('compressed_image_topic_qos') )
        else:
            self.publisher = self.create_publisher(Image,
                                self.get_parameter_value('image_topic'),
                                self.get_parameter_value('image_topic_qos') )
        self.frame_num = 0
        
    def set_camera_parameters(self):
        self.camera.set(cv2.CAP_PROP_CONVERT_RGB, 1.0)
        self.camera.set(cv2.CAP_PROP_FPS, self.get_parameter_value('camera_frame_rate'))

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter_value('camera_image_width'))
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter_value('camera_image_height'))

        if self.get_parameter_value('camera_awb_mode'):
            self.camera.set(cv2.CAP_PROP_AUTO_WB, 1.0)
        else:
            self.camera.set(cv2.CAP_PROP_AUTO_WB, 0)
            self.camera.set(cv2.CAP_PROP_AUTO_WB, self.get_parameter_value('camera_temp'))
        
        self.camera.set(cv2.CAP_PROP_CONTRAST, self.get_parameter_value('camera_contrast'))
        self.camera.set(cv2.CAP_PROP_BRIGHTNESS, self.get_parameter_value('camera_brightness'))

        # self.camera.brightness = self.get_parameter_value('camera_brightness')
        # self.camera.contrast = self.get_parameter_value('camera_contrast')
        # if self.has_parameter('camera_exif_copyright'):
        #     self.camera.exif_tage['IFDO.Copyright'] = self.get_parameter_value('camera_exif_copyright')
        # if self.has_parameter('camera_exif_user_comment'):
        #     self.camera.exif_tage['EXIF.UserComment'] = self.get_parameter_value('camera_exif_user_comment')
        # self.camera.exposure_compensation = self.get_parameter_value('camera_exposure_compenstation')
        # self.camera.exposure_mode = self.get_parameter_value('camera_exposure_mode')
        # self.camera.hflip = self.get_parameter_value('camera_hflip')
        # self.camera.vflip = self.get_parameter_value('camera_vflip')
        # self.camera.image_effect = self.get_parameter_value('camera_image_effect')
        # self.camera.meter_mode = self.get_parameter_value('camera_meter_mode')
        # self.image_width = self.get_parameter_value('camera_image_width')
        # self.image_height = self.get_parameter_value('camera_image_height')
        # self.camera.resolution = ( self.image_width, self.image_height )
        # self.get_logger().debug('CAM: setting capture resolution = %s/%s'
        #        % (self.camera.resolution[0], self.camera.resolution[1]))
        # self.camera.saturation = self.get_parameter_value('camera_saturation')
        # self.camera.sharpness = self.get_parameter_value('camera_sharpness')

    def initialize_capture_queue(self):
        # Create a queue and two threads to capture and then push the images to the topic
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()
        # self.capture_queue = queue.SimpleQueue()  # introduced in Python 3.7

        # thread to capture camera images and place in queue
        self.capture_event = threading.Event()
        self.capturer_thread = threading.Thread(target=self.take_pictures, name='capturer')

        # thread to read queue and send them to the topic
        self.publisher_event = threading.Event()
        self.publisher_thread = threading.Thread(target=self.publish_images, name='publisher')

        self.capturer_thread.start()
        self.publisher_thread.start()

    def stop_workers(self):
        # if workers are initialized and running, tell them to stop and wait until stopped
        if hasattr(self, 'capture_event') and self.capture_event != None:
            self.capture_event.set()
        if hasattr(self, 'publisher_event') and self.publisher_event != None:
            self.publisher_event.set()
        if hasattr(self, 'publisher_thread') and self.publisher_thread.is_alive():
            self.publisher_thread.join()
        if hasattr(self, 'capturer_thread') and self.capturer_thread.is_alive():
            self.capturer_thread.join()


    def take_pictures(self):
        # Take compressed images and put into the queue.
        # 'jpeg', 'rgb'
        # https://docs.opencv.org/3.4/d8/dfe/classcv_1_1VideoCapture.html
        try:
            while self.keepRunning:
                ret, frame = self.camera.read()
                if ret == True:
                    # Flip the image if requested
                    if self.get_parameter_value('camera_vflip'):
                        if self.get_parameter_value('camera_hflip'):
                            frame = cv2.flip(frame, -1)
                        else:
                            frame = cv2.flip(frame, 0)
                    else:
                        if self.get_parameter_value('camera_hflip'):
                            frame = cv2.flip(frame, 1)

                    if self.get_parameter_value('compressed_image'):
                        result, encimg = cv2.imencode('.jpg', frame)
                        if result == True:
                            self.write_compressed_capture(encimg, 'jpeg')
                    else:
                        encimg = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        self.write_capture(encimg, 'rgb8')

                time.sleep(0.5)
        except Exception as err:
            self.get_logger().error('take_pictures: exiting take_pictures because of exception')
            self.get_logger().error(traceback.format_exc())

    def write_capture(self, frame, fmt):
        with self.queue_lock:
            msg = self.bridge.cv2_to_imgmsg(frame, fmt)
            # msg.encoding = fmt
            # msg.height = self.get_parameter_value('camera_image_height')
            # msg.width = self.get_parameter_value('camera_image_width')
            # msg.step = 3 * msg.width
            msg.header.frame_id = str(self.frame_num)
            self.frame_num += 1
            self.get_logger().debug('write_capture: capture frame. size=%s, frame=%s'
                    % (len(frame), msg.header.frame_id) )
            # msg.header.stamp = time.Time
            self.capture_queue.put(msg)

    def write_compressed_capture(self, frame, fmt):
        with self.queue_lock:
            msg = CompressedImage()
            msg.data = np.array(frame).tostring()
            msg.format = fmt
            msg.header.frame_id = str(self.frame_num)
            self.frame_num += 1
            self.get_logger().debug('write_compressed_capture: capture frame. size=%s, frame=%s'
                    % (len(frame), msg.header.frame_id) )
            # msg.header.stamp = time.Time
            self.capture_queue.put(msg)

    def publish_images(self):
        # Loop reading from capture queue and send to ROS topic
        while True:
            if self.publisher_event.is_set():
                break
            try:
                msg = self.capture_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg = None
            if self.publisher_event.is_set():
                break
            if msg != None:
                self.get_logger().debug('CAM: sending frame. frame=%s'
                                    % (msg.header.frame_id) )
                self.publisher.publish(msg)

    def get_parameter_value(self, param):
        # Helper function to return value of a parameter
        ret = None
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            raise Exception('Fetch of parameter that does not exist: ' + param)
        else:
            ret = param_desc.value
        return ret

    def set_parameter_defaults(self, params):
        # If a parameter has not been set externally, set the value to a default.
        # Passed a list of "(parameterName, parameterType, defaultValue)" tuples.
        for (pparam, ptype, pdefault) in params:
            if not self.has_parameter(pparam):
                self.declare_parameter(pparam, pdefault)

    def parameter_set_if_set(self, param, set_function):
        # If there is a parameter set, do set_function with the value
        if self.has_parameter(param):
            set_function(self.get_parameter_value(param))

def main(args=None):
    rclpy.init(args=args)

    camNode = ROS2_raspicam_node()

    try:
        rclpy.spin(camNode)
    except KeyboardInterrupt:
        camNode.get_logger().info('CAM: Keyboard interrupt')
        camNode.keepRunning = False

    camNode.stop_workers()

    camNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
