#!/usr/bin/env python3
import cv2
import rospy
import edge_detection_msgs.srv
from cv_bridge import CvBridge

bridge = CvBridge()
imgs = ['edge_detection/data/Image_1.png',
        'edge_detection/data/Image_2.png',
        'edge_detection/data/Image_3.png',
        'edge_detection/data/Image_4.png',
        'edge_detection/data/Image_5.png']



if __name__ == "__main__":
    rospy.init_node("test_edge_detector_client")
    detect_edges = rospy.ServiceProxy('detect_edges', edge_detection_msgs.srv.DetectEdges)

    # Display edge detection for each image
    for img in imgs:
        try:
            rospy.wait_for_service('detect_edges', timeout=10)
            req = edge_detection_msgs.srv.DetectEdgesRequest()
            req.path_to_image = img
            rospy.loginfo('DetectEdges requested')
            rsp = detect_edges(req)
            img_edges = bridge.imgmsg_to_cv2(rsp.edge_img, desired_encoding="passthrough")
            cv2.imshow('Edge Detection', img_edges)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        except:
            rospy.logerr("No response from detect_edges!")
