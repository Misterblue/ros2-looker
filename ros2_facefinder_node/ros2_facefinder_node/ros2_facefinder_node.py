# Copyright 2018 Robert Adams
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

import io
import numpy as np
import queue
import threading
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension

import cv2
from cv_bridge import CvBridge

import dlib
import imageio

class ROS2_facefinder_node(Node):

    def __init__(self):
        super().__init__('ros2_facefinder_node', namespace='raspicam')

        self.set_parameter_defaults( [
            ('image_input_topic', Parameter.Type.STRING, 'raspicam_uncompressed'),
            ('image_input_topic_subqos', Parameter.Type.INTEGER, 10),
            ('image_compressed_input_topic', Parameter.Type.STRING, 'raspicam_compressed'),
            ('image_compressed_input_topic_subqos', Parameter.Type.INTEGER, 10),
            ('face_output_topic', Parameter.Type.STRING, 'found_faces'),
            ('face_output_topic_subqos', Parameter.Type.INTEGER, 10),
            ('face_output_topic_pubqos', Parameter.Type.INTEGER, 10)
            ] )

        self.bridge = CvBridge()

        self.initialize_face_recognizer()
        self.initialize_image_subscriber()
        self.initialize_bounding_box_publisher()
        self.initialize_processing_queue()

    def destroy_node(self):
        # overlay Node function called when class is being stopped
        super().destroy_node()

    def initialize_face_recognizer(self):
        # Get  the detector that will be used herein.
        # Will eventually add face recognizer but this is a start.
        self.detector = dlib.get_frontal_face_detector()

    def initialize_image_subscriber(self):
        # Setup subscription for incoming images.
        self.receiverCompressed = self.create_subscription(
                    CompressedImage, self.get_parameter_value('image_compressed_input_topic'),
                    self.receive_compressed_image,
                    self.get_parameter_value('image_compressed_input_topic_subqos'),
                    )
        self.receiver = self.create_subscription(
                    Image, self.get_parameter_value('image_input_topic'),
                    self.receive_image,
                    self.get_parameter_value('image_input_topic_subqos'),
                    )
        self.frame_num = 0

    def initialize_bounding_box_publisher(self):
        self.bounding_box_publisher = self.create_publisher(Int32MultiArray,
                                        self.get_parameter_value('face_output_topic'),
                                        self.get_parameter_value('face_output_topic_pubqos') )

    def initialize_processing_queue(self):
        # Create a queue and a thread that processes messages in the queue
        self.queue_lock = threading.Lock()

        self.image_queue = queue.Queue()
        # self.image_queue = queue.SimpleQueue()  # introduced in Python 3.7

        # thread to read images placed in the queue and process them
        self.processor_event = threading.Event()
        self.processor = threading.Thread(target=self.process_images, name='facefinder')

        self.processor.start()

    def stop_workers(self):
        # if workers are initialized and running, tell them to stop and wait until stopped
        if hasattr(self, 'processor_event') and self.processor_event != None:
            self.processor_event.set()
        if hasattr(self, 'processor') and self.processor.is_alive():
            self.processor.join()

    def receive_image(self, msg):
        # Called to process message received by subscription. Queue the message.
        if msg != None and hasattr(msg, 'data'):
            self.get_logger().debug('FFinder: receive_image. dataLen=%s' % (len(msg.data)) )
            self.image_queue.put(msg)

    def receive_compressed_image(self, msg):
        # Called to process message received by subscription. Queue the message.
        if msg != None and hasattr(msg, 'data'):
            self.get_logger().debug('FFinder: receive_compressed_image. dataLen=%s' % (len(msg.data)) )
            self.image_queue.put(msg)

    def process_images(self):
        # Take images from the queue and find the faces therein.
        # This runs on its own thread so as not to block reception.
        while True:
            if self.processor_event.is_set():
                break
            try:
                msg = self.image_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg = None
            if self.processor_event.is_set():
                break
            if type(msg) != type(None):
                self.get_logger().debug('FFinder: process_image frame=%s, dataLen=%s'
                                    % (msg.header.frame_id, len(msg.data)) )

                # prepare the image and make suitable for face finding
                img, iwidth, iheight = self.convert_image(msg)

                # if there is a good image, find any faces
                if type(img) != type(None):
                    face_bounding_boxes = self.find_faces(img)
                    self.publish_bounding_boxes(face_bounding_boxes,
                                iwidth, iheight)

    def convert_image(self, msg):
        # Convert the passed sensor message into a proper Python image. I.e., use imageio
        #    to do any uncompression, etc.
        # Note that the returned array is a subclass of numpy.array that includes a
        #    '.meta' dictionary which returns 'height' and 'width' of the converted image.
        # TODO: add any logic needed to prepare either compressed or uncompressed images.
        img = None
        width = 10
        height = 10

        # as of 20181016, the msg.data is returned as a list of ints. Convert to bytearray.
        # converted_data = []
        # with CodeTimer(self, 'convert to byte array'):
        #     converted_data = bytearray(msg.data)

        try:
            if hasattr(msg, 'encoding'):
                # the uncompressed image format is specificed by 'encoding'
                img = self.bridge.imgmsg_to_cv2(msg)
                iwidth = msg.width
                iheight = msg.height
            else:
                # imageio.imread returns a numpy array where img[h][w] => [r, g, b]
                with CodeTimer(self, 'decompress image'):
                    img = imageio.imread(io.BytesIO(image.data))
                    iwidth = len(img[0])
                    iheight = len(img)
                    self.get_logger().debug('FFinder: imread image: h=%s, w=%s' % (len(img), len(img[0])))
        except Exception as e:
            self.get_logger().error('FFinder: exception uncompressing image. %s: %s'
                            % (type(e), e.args) )
            img = None
        return img, width, height;

    def find_faces(self, img):
        # Given and image, find the faces therein and return the bounding boxes.
        # Returns an array of 'dlib.rectangle'.
        with CodeTimer(self, 'detect faces'):
            detected = self.detector(img, 0)
        if len(detected) > 0:
            self.get_logger().info('FFinder: detected %s faces' % (len(detected)))
            for i, d in enumerate(detected):
                self.get_logger().info("   face %s: Left: %s Top: %s Right: %s Bottom: %s" %
                        (i, d.left(), d.top(), d.right(), d.bottom()) )
        return detected

    def publish_bounding_boxes(self, bbs, image_width, image_height):
        # Given a list of bounding boxes, publish same.
        # The detector returns an array of dlib.rectangle's.
        if len(bbs) > 0:
            msg = Int32MultiArray()
            msg.layout.data_offset = 0
            msg.layout.dim.append(
                MultiArrayDimension(label='height', size=len(bbs) + 1, stride=4 * (len(bbs) +1) ))
            msg.layout.dim.append(
                MultiArrayDimension(label='width', size=4, stride=1) )
            msg.layout.dim.append(
                MultiArrayDimension(label='channel', size=1, stride=1) )
            data = []
            # The first entry is the size of the source image
            data.append(0)
            data.append(0)
            data.append(image_width)
            data.append(image_height)
            for rect in bbs:
                data.append(rect.left())
                data.append(rect.top())
                data.append(rect.right())
                data.append(rect.bottom())
            msg.data = data
            self.bounding_box_publisher.publish(msg)
        return

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

class CodeTimer:
    # A little helper class for timing blocks of code.
    # Use like: with CodeTImer(logger, 'name'):
    #                Do_some_statements
    # This will call 'logger' after the block of statements is complete with
    #    a message containing the 'name' and the CPU time used by the statements.
    def __init__(self, parent, name=None):
        self.parent = parent
        self.name = " '"  + name + "'" if name else ''

    def __enter__(self):
        self.start = time.time()

    def __exit__(self, exc_type, exc_value, traceback):
        self.took = time.time() - self.start
        self.parent.get_logger().debug('Code block' + self.name + ' took: ' + str(self.took) + ' ms')

def main(args=None):
    rclpy.init(args=args)

    ffNode = ROS2_facefinder_node()

    try:
        rclpy.spin(ffNode)
    except KeyboardInterrupt:
        ffNode.get_logger().info('FFinder: Keyboard interrupt')

    ffNode.stop_workers()

    ffNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
