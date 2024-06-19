import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Initialize the CvBridge class
bridge = CvBridge()

def image_callback(ros_image):
    try:
        # Convert the ROS Image message to a CV2 Image
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Could not convert image: {e}")
        return

    # Check image dimensions
    height, width, channels = cv_image.shape
    rospy.loginfo(f"Image received with dimensions: {width}x{height}")

    # Verify the pixel at column 1280 (this should be within bounds)
    if width >= 1280:
        pixel_value = cv_image[0, 1279]  # OpenCV uses 0-based index
        rospy.loginfo(f"Pixel value at (0, 1279): {pixel_value}")
    else:
        rospy.logwarn(f"Image width {width} is less than 1280")

def main():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/spot_image", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
