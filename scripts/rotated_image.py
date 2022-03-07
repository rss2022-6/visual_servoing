#!/usr/bin/env python
import rospy
import imutils

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class RotatedImage():
    """
    A class for flipping ZED camera images.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /flipped_image (Image) : the same RGB image, rotated 180 degrees.
    """
    def __init__(self):
        # Subscribe to ZED camera RGB frames
        self.image_pub = rospy.Publisher("/rotated_image", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        image = imutils.rotate(image, 180)

        image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.image_pub.publish(image_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('RotatedImage', anonymous=True)
        RotatedImage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
