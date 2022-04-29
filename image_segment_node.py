#!/usr/bin/env python
import rospy
#from me416_lab 
import image_processing as ip
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
#convert OpenCV to Image using cv_bridge
bridge = CvBridge()

global pub1,pub2,msg,img_processed
def callback(rosmsg):
    global pub1,pub2,msg,img_processed
    #convert CompressedImage to OpenCV image
    np_arr = np.fromstring(rosmsg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #resize image
    #img = cv2.resize(img,(320,240))
    #img = img[1:10,:,:]
    #run img_classify to segment test track line from the background
    lb,ub=ip.classifier_parameters()
    img_segmented = cv2.inRange(img,lb,ub)
    #run img_centroid_horizontal to compute centroid
    x_centroid = ip.image_centroid_horizontal(img_segmented)
    #run img_line_vertical to add the line on the segmented image
    img_segmented_line = ip.image_line_vertical(ip.image_one_to_three_channels(img_segmented),x_centroid)
    img_processed = bridge.cv2_to_imgmsg(img_segmented_line, "bgr8")
    #publish segmented image to topic /image/segmented
    pub1.publish(img_processed)
    x_centroid = float(x_centroid)   #PointStamped.point.x is a float
    t = rospy.Time.now()
    msg = PointStamped()
    msg.point.x=x_centroid     #update x field of PointStamped
    msg.header.stamp = t       #add current time to header.stamp
    pub2.publish(msg)          #publish to /image/centroid

def main():
    global pub1,pub2,msg, img_processed
    rospy.init_node('main')
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, callback=callback, queue_size=1, buff_size=2**18)
    pub1 = rospy.Publisher('/image/segmented', Image, queue_size=10)
    pub2 = rospy.Publisher('/image/centroid', PointStamped, queue_size=10)
    while not rospy.is_shutdown(): 
        rospy.spin()
if __name__ == '__main__':
    main()
