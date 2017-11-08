#!/usr/bin/env python
"""Standalone script for extracting images from rosbagged video feeds.

Parameters are set within the script itself, and must be the same for all
ROSbags in the specified directory.

Example:
    [bag directory] is the directory your .bag files are in.
    [image directory] is the directory you would like image files written to.
    Both paths should be absolute.

        $ python rosrip.py [bag directory] [image directory]

Attributes:
    FEED_TOPIC (string): The name of the ROS topic over which the images you
        would like to extract were broadcast in the .bag file(s).
    COMPRESSED (boolean): Whether the images were broadcast as CompressImage
        messages instead of Image messages.
    SAMPLING_FREQUENCY (int): How often you would like to sample a frame, in
        seconds. Technically an interval rather than a frequency.
    PREFIX (string): String prepended to all written image file names.
    SUFFIX (string): String appended to all written image file names.
    EXTENSION (string): Extension for the image format you would like,
        typically '.png' or '.jpg'/'.jpeg'.
    ROSBAG_DIR (string): Absolute path to the directory your ROS bags are in.
    IMAGE_DIR (string): Absolute path to the directory images should be
        written to.

Todo:
    * Accept relative or absolute paths.
    * Intelligently determine COMPRESSED parameter from bag info.
    * Specify other parameters in .yaml file, not in code.

"""


import rosbag
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys, time, os, string

# PARAMETERS
FEED_TOPIC = '/camera/left/image_raw/compressed'
COMPRESSED = True
SAMPLING_FREQUENCY = 0.5    # in seconds
PREFIX = ''                 # prepended to all saved image files
SUFFIX = ''                 # appended to all saved image files
EXTENSION = '.png'          # should be either '.jpg' or '.png'
ROSBAG_DIR = sys.argv[1]    # directory containing all rosbags to be played
IMAGE_DIR = sys.argv[2]     # directory images should be written to
# DO NOT CHANGE CODE BELOW THIS LINE

if not ROSBAG_DIR.endswith('/'):
    ROSBAG_DIR += '/'
if not IMAGE_DIR.endswith('/'):
    IMAGE_DIR += '/'


BRIDGE = CvBridge()
def write_image_to_file(data):
    """
    Writes a sensor_msgs/Image to an image file.
    Encoding is specified by global parameters above.
    """
    try:
        if COMPRESSED:
            image = BRIDGE.compressed_imgmsg_to_cv2(data, "bgr8")
        else:
            image = BRIDGE.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    tsecs = time.gmtime(data.header.stamp.secs)
    nsecs = str(data.header.stamp.nsecs)
    while len(nsecs) != 9:
        nsecs = '0' + nsecs
    timestamp = time.strftime("%Y-%m-%dT%H%M%S-" + str(nsecs), tsecs)
    filename = IMAGE_DIR + PREFIX + timestamp + SUFFIX + EXTENSION
    cv2.imwrite(filename, image)
    cv2.imshow('Writing image:', image)
    cv2.waitKey(1)

BAG_LIST = [b for b in os.listdir(ROSBAG_DIR) if b.endswith('.bag')]

for item in BAG_LIST:
    bag = rosbag.Bag(ROSBAG_DIR + item)
    last_write = None
    for topic, msg, t in bag.read_messages(topics=[FEED_TOPIC]):
        current_timestamp = t.secs + t.nsecs / 1e9
        if last_write is None:
            write = True
        else:
            write = current_timestamp - last_write > SAMPLING_FREQUENCY
        if write:
            write_image_to_file(msg)
            last_write = current_timestamp

    bag.close()
