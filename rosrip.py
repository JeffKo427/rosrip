#!/usr/bin/env python
import rosbag
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys, time, os, string

# PARAMETERS
FEED_TOPIC = '/stereo_camera/left/image_rect_color/compressed'
COMPRESSED = True
SAMPLING_FREQUENCY = 5.0    # in seconds
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
    """docstring"""
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
