# ros2-looker

ROS2 nodes for doing face looking with adafruit-pwmhat.

Several years ago, when ROS2 was young, I started a simple test project made
up of several nodes. These used to be in separate, now archived, repositories
([picam], [pwmhat], [findface], [looker]).

This incarnation is a revisiting of the project for ROS2-foxy that merges all
the nodes into one 'package'.

This consists of four nodes:

- raspicam: a node that captures and publishes still images from a Raspberry Pi camera
- facefind: receives images and outputs bounding boxes for faces that appear in the image
- facelook: receives the bounding boxes and outputs PWM commands to move the camera to center the faces
- pwmhat: receives commands that are sent to an AdaFruit PWM hat for a Raspberry Pi

This odd construction is because I wanted to test out ROS2, try defining my own messages,
and otherwise make a project beyond my skills to get me into ROS2.

### Raspicam

A Python ROS2 node for generating images from the Raspberry Pi camera.
This uses the wonderful [PiCamera] to drive the camera while this node
pacakage takes the JPEG images and sends them out over a ROS2 topic.

| Topic                         | Message Format                   | Data                                                       |
| ----------------------------- | -------------------------------- | ---------------------------------------------------------- |
| /raspicam/raspicam_compressed | sensor_msgs.msg.CompressedImage_ | data: image<br>format: "jpeg"<br>header.frame_id: frameNum |

Since this project is being worked on, it does not output a raw image, output camera info,
or accept parameters. That will happen in a later step.

Internally, this starts two threads one of which captures an image from the
camera which it puts into a FIFO queue while the other thread reads from the
queue and publishes the image to the topic.

### FaceFind

A Python ROS2 node that receives ```sensor_msg/Image``` messages and outputs face bounding boxes.
This bounding boxes are output as a ```std_msg/Int32MultifactorArray``` on topic ``` ```.

### FaceLook

A Python ROS2 node that receives ```std_msg/Int32MultifactorArray``` that contain face bounding
box information and then outputs PWM commands to stepping motors that control the pan and
tilt of the camera. The goal is to keep the seen faces as close to the center if the image as possible.

### PWMHat

A Python ROS2 node for control of the [Adafruit 16-Channel PWM/Servo HAT].

When run on the Raspberry Pi 3 that has the PWMHAT, this node subscribes to the topics:

| Topic                     | Message format                              | Data                                                           |
|:------------------------- | ------------------------------------------- | -------------------------------------------------------------- |
| /pwmhatter/angle          | ros2_adafruit_pwmhat_msgs/PWMAngle          | chan: channelName<br>angle: angleNumber<br>angle-unit: DEGREES |
| /pwmhatter/pinAngle       | ros2_adafruit_pwmhat_msgs/PWMPinAngle       | pin: pinNumber<br>angle: angleNumber<br>angle-unit: DEGREES    |
| /pwmhatter/pulseLength    | ros2_adafruit_pwmhat_msgs/PWMPulseLength    | chan: channelName<br>pulse-length: pulseMS                     |
| /pwmhatter/pinPulseLength | ros2_adafruit_pwmhat_msgs/PWMPinPulseLength | pin: pinNumber<br>pulse-length: pulseMS                        |

The angle could be either degrees or radians and the '''angle-unit''' variable can be
the constant "DEGREES" or "RADIANS" with the default being degrees if not specified.

The stepping motor parameters are hard coded for the SG90 stepping motor which has
a documented PWM pulse width from 1ms to 2ms but experimentation has shown that the
real range for 180 degree movement is from 0.6ms to 2.4ms.
The angle range parameters are for -90 degrees to +90 degrees.
Other stepping motor definitions can be added and someday there will be an
external parameter file (see note at bottom about ROS2, parameters, and Python).

## Building

## Running

## Notes

If you look at the code in (ros2_adafruit_pwmhat_node) you will see that the compiled in parameters are for the SG90 stepping motor. Others will be added someday.

[pwmhat]: https://github.com/Misterblue/ros2_adafruit_pwmhat_node
[raspicam]: https://github.com/Misterblue/ros2_raspicam_node
[facelook]: https://github.com/Misterblue/ros2_facelook_node
[facefinder]: https://github.com/Misterblue/ros2_facefinder_node
[PiCamera]: https://picamera.readthedocs.io/en/release-1.13/
[Adafruit 16-Channel PWM/Servo HAT]: https://www.adafruit.com/product/2327


