Standalone script for extracting images from rosbagged video feeds.

Parameters are set within the script itself, and must be the same for all
ROSbags in the specified directory.

# Example:
[bag directory] is the directory your .bag files are in.
[image directory] is the directory you would like image files written to.
Both paths should be absolute.

    $ python rosrip.py [bag directory] [image directory]

# Attributes:
* FEED_TOPIC (string): The name of the ROS topic over which the images you would like to extract were broadcast in the .bag file(s).
* COMPRESSED (boolean): Whether the images were broadcast as CompressedImage messages instead of Image messages.
* SAMPLING_FREQUENCY (int): How often you would like to sample a frame, in seconds. Technically an interval rather than a frequency.
* PREFIX (string): String prepended to all written image file names.
* SUFFIX (string): String appended to all written image file names.
* EXTENSION (string): Extension for the image format you would like, typically '.png' or '.jpg'/'.jpeg'.
* ROSBAG_DIR (string): Absolute path to the directory your ROS bags are in.
* IMAGE_DIR (string): Absolute path to the directory images should be written to.

# Dependencies:
* ROS (obviously)
* [cv_bridge](http://wiki.ros.org/cv_bridge) package
