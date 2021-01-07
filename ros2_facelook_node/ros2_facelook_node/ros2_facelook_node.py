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
import queue
import threading
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension

from ros2_adafruit_pwmhat_msgs.msg import PWMPinAngle, PWMAngle

class ROS2_facelook_node(Node):

    def __init__(self):
        super().__init__('ros2_facelook_node', namespace='raspicam')

        self.set_parameter_defaults( [
            ('bounding_box_topic', Parameter.Type.STRING, 'found_faces'),
            ('pwm_topic', Parameter.Type.STRING, '/pwmhatter/angle'),
            ('angle_step', Parameter.Type.DOUBLE, 1.0),
            ('delta_magnification', Parameter.Type.DOUBLE, 10.0),
            ('max_angle', Parameter.Type.DOUBLE, 80.0),
            ] )

        self.initialize_pwm_publisher()
        self.initialize_processing_queue()
        self.initialize_bounding_box_subscriber()

    def destroy_node(self):
        # overlay Node function called when class is being stopped and camera needs closing
        super().destroy_node()

    def initialize_bounding_box_subscriber(self):
        # Setup subscription for incoming bounding box info
        self.receiver = self.create_subscription(Int32MultiArray,
                        self.get_parameter_value('bounding_box_topic'),
                        self.receive_bounding_box)

    def initialize_processing_queue(self):
        # Create a queue and a thread that processes messages in the queue
        self.queue_lock = threading.Lock()

        self.bbox_queue = queue.Queue()
        # self.bbox_queue = queue.SimpleQueue()  # introduced in Python 3.7

        # thread to read images placed in the queue and process them
        self.processor_event = threading.Event()
        self.processor = threading.Thread(target=self.process_bounding_boxes, name='bounding box')

        self.processor.start()

    def initialize_pwm_publisher(self):
        # initialize 'known' angle so first request will be sure to go out
        self.pan_angle = 10000
        self.tilt_angle = 10000

        self.pwmmer = PWMmer(self,
                            self.get_parameter_value('pwm_topic'),
                            -self.get_parameter_value('max_angle'),
                            self.get_parameter_value('max_angle'),
                            self.get_logger())
        self.send_pwm_commands(self.pan_angle, self.tilt_angle)

    def stop_workers(self):
        # if workers are initialized and running, tell them to stop and wait until stopped
        if hasattr(self, 'processor_event') and self.processor_event != None:
            self.processor_event.set()
        if hasattr(self, 'processor') and self.processor.is_alive():
            self.processor.join()

    def receive_bounding_box(self, msg):
        if type(msg) != type(None) and hasattr(msg, 'data'):
            self.get_logger().debug('FLooker: receive_bbox. dataLen=%s' % (len(msg.data)))
            self.bbox_queue.put(msg)
        else:
            self.get_logger().error('FLooker: receive_bbox. no data attribute')

    def process_bounding_boxes(self):
        # Take bounding boxes from the queue and send angle commands to the camera

        # Initialize camera position
        self.get_logger().debug('FLooker: Initializing camera to 0,0')
        self.send_pwm_commands(0, 0)

        # Loop for each bounding box info and update the camera movement
        while True:
            if self.processor_event.is_set():
                break
            try:
                msg = self.bbox_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg = None
            if self.processor_event.is_set():
                break
            if type(msg) != type(None):
                # Bounding boxes come in a two dimensional array:
                #   Row 0 => ( 0, 0, imageAreaWidth, imageAreaHeight)
                #   Row n => ( bb_right, bb_top, bb_width, bb_height )
                bboxes = AccessInt32MultiArray(msg)
                width = bboxes.get(0, 2)
                widthhalf = width / 2
                height = bboxes.get(0, 3)
                heighthalf = height / 2
                self.get_logger().debug('FLooker: process_bounding_boxes. image=%s/%s' % (width, height) )

                # loop over all bounding boxes and computer the average center
                wcenter = 0
                wheight = 0
                hcenter = 0
                hwidth = 0
                for ii in range(1, bboxes.rows):
                    wcenter = wcenter + ((bboxes.get(ii, 2) - bboxes.get(ii, 0)) / 2) + bboxes.get(ii,0)
                    wheight = wheight + bboxes.get(ii,3)
                    hcenter = hcenter + ((bboxes.get(ii, 3) - bboxes.get(ii, 1)) / 2) + bboxes.get(ii,1)
                    hwidth = hwidth + bboxes.get(ii,2)
                waverage = wcenter / ( bboxes.rows - 1) # average horizontal center of all boxes
                wheight = wheight / ( bboxes.rows - 1)  # average height of all boxes
                haverage = hcenter / ( bboxes.rows - 1) # average vertical center of all boxes
                hwidth = hwidth / ( bboxes.rows - 1)    # average width of all boxes
                self.get_logger().debug('FLooker: process_bounding_boxes. averageCenter=%s/%s, averageSize=%s/%s'
                                % (waverage, haverage, hwidth, wheight) )

                # positive deltas mean above the middle and negative deltas mean below the middle
                wdelta = (width / 2) - waverage
                hdelta = (height / 2) - haverage
                self.get_logger().debug('FLooker: process_bounding_boxes. deltas=%s/%s'
                                % (wdelta, hdelta) )

                if (wdelta <= -widthhalf
                        or wdelta >= widthhalf
                        or hdelta <= -heighthalf
                        or hdelta >= heighthalf):
                    self.get_logger().error('FLooker: deltas wrong! dim=%s/%s, avg=%s/%s, delta=%s/%s'
                                % ( width, height, waverage, haverage, wdelta, hdelta) )
                else:
                    target_pan_angle = (self.pan_angle
                        + (self.get_parameter_value('angle_step')
                            * self.sign(wdelta)
                            * abs(wdelta) / self.get_parameter_value('delta_magnification')
                            )
                        )
                    target_tilt_angle = (self.tilt_angle
                        - (self.get_parameter_value('angle_step')
                            * self.sign(hdelta)
                            * abs(hdelta) / self.get_parameter_value('delta_magnification')
                            )
                        )
                    self.send_pwm_commands(target_pan_angle, target_tilt_angle)

    def send_pwm_commands(self, target_pan_angle, target_tilt_angle):
        # Send command to PWM channels if the desired angle has changed.
        # Note: uses and updates self.pan_angle and self.tilt_angle.
        if target_pan_angle != self.pan_angle:
            if self.pwmmer.setPWM('pan', target_pan_angle):
                self.get_logger().debug('FLooker: sending chan=%s, angle=%s' % ('pan', target_pan_angle))
                self.pan_angle = target_pan_angle
            else:
                self.get_logger().error('FLooker: target pan angle failed! targets=%s/%s'
                                % (target_pan_angle, target_tilt_angle) )
        if target_tilt_angle != self.tilt_angle:
            if self.pwmmer.setPWM('tilt', target_tilt_angle):
                self.get_logger().debug('FLooker: sending chan=%s, angle=%s' % ('tilt', target_tilt_angle))
                self.tilt_angle = target_tilt_angle
            else:
                self.get_logger().error('FLooker: target tilt angle failed! targets=%s/%s'
                                % (target_pan_angle, target_tilt_angle) )
                    
    def get_parameter_or(self, param, default):
        # Helper function to return value of a parameter or a default if not set
        ret = None
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            ret = default
        else:
            ret = param_desc.value
        return ret

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
        parameters_to_set = []
        for (pparam, ptype, pdefault) in params:
            if not self.has_parameter(pparam):
                parameters_to_set.append( Parameter(pparam, ptype, pdefault) )
        if len(parameters_to_set) > 0:
            self.set_parameters(parameters_to_set)

    def has_parameter(self, param):
        # Return 'True' if a parameter by that name is specified
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            return False
        return True

    def sign(self, val):
        # Helper function that returns the sign of the passed value (1 or -1).
        # Defined here so we don't have to require numpy.
        return 1 if val >= 0 else -1

class PWMmer:
    # Small class to hold current state of PWM channel
    def __init__(self, node, topic, minVal, maxVal, logger=None):
        self.node = node
        self.topic = topic
        self.minVal = minVal
        self.maxVal = maxVal
        self.logger = logger
        self.channels = {}
        self.logger.debug('PWMmer: init: topic=%s, min=%s, max=%s' %
                    (topic, str(minVal), str(maxVal)))

        self.publisher = self.node.create_publisher(PWMAngle, topic)

    def setPWM(self, channel, angle):
        # Send the message to set the given PWM channel
        ret = True
        if not channel in self.channels:
            self.channels[channel] = self.minVal - 1000

        if angle != self.channels[channel]:
            if angle >= self.maxVal or angle <= self.minVal:
                self.logger.error('PWMmer: angle out of range. channel=%s, angle=%s'
                                % (channel, angle) )
                ret = False
            else:
                msg = PWMAngle()
                msg.chan = str(channel)
                msg.angle = float(angle)
                msg.angle_units = PWMAngle.DEGREES
                self.publisher.publish(msg)
                self.channels[channel] = angle
                ret = True

        return ret


class AccessInt32MultiArray:
    # Wrap a multi-access array with functions for 2D access
    def __init__(self, arr):
        self.arr = arr
        self.columns = self.ma_get_size_from_label('width')
        self.rows = self.ma_get_size_from_label('height')

    def rows(self):
        # return the number of rows in the multi-array
        return self.rows

    def get(self, row, col):
        # return the entry at column 'ww' and row 'hh'
        return self.arr.data[col + ( row * self.columns)]

    def ma_get_size_from_label(self, label):
        # Return dimension size for passed label (usually 'width' or 'height')
        for mad in self.arr.layout.dim:
            if mad.label == label:
                return int(mad.size)
        return 0
        

class CodeTimer:
    # A little helper class for timing blocks of code
    def __init__(self, logger, name=None):
        self.logger = logger
        self.name = " '"  + name + "'" if name else ''

    def __enter__(self):
        self.start = time.clock()

    def __exit__(self, exc_type, exc_value, traceback):
        self.took = (time.clock() - self.start) * 1000.0
        self.logger('Code block' + self.name + ' took: ' + str(self.took) + ' ms')

def main(args=None):
    rclpy.init(args=args)

    ffNode = ROS2_facelook_node()

    try:
        rclpy.spin(ffNode)
    except KeyboardInterrupt:
        ffNode.get_logger().info('FLooker: Keyboard interrupt')

    ffNode.stop_workers()

    ffNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
