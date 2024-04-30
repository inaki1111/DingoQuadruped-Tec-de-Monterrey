#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def main():
    rospy.init_node('camera_node', anonymous=True)
    image_pub = rospy.Publisher("camera/image", Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Error opening video camera")
        return

    try:
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                try:
                    image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                    image_pub.publish(image_msg)
                except cv2.error as e:
                    rospy.logerr("CvBridge Error: {0}".format(e))
            rate.sleep()
    finally:
        cap.release()
        rospy.loginfo("Camera released successfully")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
