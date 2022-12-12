#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import edge_detection_msgs.srv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud


bridge = CvBridge()


class EdgeDetector:

    def __init__(self):
        self.pub_edge_points = rospy.Publisher("edge_points", PointCloud, queue_size=10)
        self.sub_img_input = rospy.Subscriber("/camera/color/image_raw", Image, self._read_rgb_img_cb)
        self.sub_img_input = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self._read_rgb_img_cb)
        self.current_rgb_img = Image()
        self.current_depth_img = Image()
        self.current_point_cloud = PointCloud()
        self.got_rgb_img = False
        self.got_depth_img = False


    def _read_rgb_img_cb(self, msg):
        self.current_rgb_img = msg
        self.current_rgb_img = bridge.imgmsg_to_cv2(self.current_rgb_img,
                                                    desired_encoding="passthrough")
        self.current_rgb_img = edge_detector._overlay_edges(self.current_rgb_img)
        self.current_rgb_img = bridge.cv2_to_imgmsg(self.current_rgb_img,
                                                    encoding="passthrough")                                        
        self.got_rgb_img = True


    def _read_rgb_img_cb(self, msg):
        self.current_depth_img = msg
        self.got_depth_img = True


    def _detect_edges(self, img):
        """
        Detects edges in open cv image and returns them highlighted in green
        """
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to graycsale
        img_blur = cv2.bilateralFilter(img_gray, 9, 120, 120)  # Blur the image for better edge detection
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
        _,alpha = cv2.threshold(edges,0,255,cv2.THRESH_BINARY)  # extract aplha channeö
        rgb_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)  # convert back to rgb
        rgb_edges *= np.array((0,1,0),np.uint8)  # turn edges green
        b, g, r = cv2.split(rgb_edges)
        rgba = [b,g,r, alpha]
        edges_tarnsp = cv2.merge(rgba,4)  # megre color and aplha channels
        return edges_tarnsp


    def _overlay_edges(self, img):
        """
        Detects edges in a given open cv image and returns the input image with edges highlighted green on top
        """
        edges = self._detect_edges(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)
        img_edges = cv2.add(img, edges)
        self.current_img = img_edges
        return img_edges


    def _convert_rgbd_to_pointcloud(rgbd_img):
        # TODO
        pass


    def detect_edges(self, req):
        rospy.loginfo('DetectEdges called')
        img = cv2.imread(req.path_to_image) 
        img_edges = self._overlay_edges(img)
        img_edges = bridge.cv2_to_imgmsg(img_edges, encoding="passthrough")
        response = edge_detection_msgs.srv.DetectEdgesResponse()
        response.edge_img = img_edges
        return response


if __name__ == "__main__":
    rospy.init_node("edge_detector_server")
    edge_detector = EdgeDetector()
    detect_edges = rospy.Service('detect_edges',
                                    edge_detection_msgs.srv.DetectEdges,
                                    edge_detector.detect_edges)

    
    if edge_detector.got_rgb_img and edge_detector.got_depth_img:
        edge_detector.got_rgb_img = False
        edge_detector.got_depth_img = False
        # TODO: add rgbd to pointcloud conversion function here
        edge_detector.pub_edge_points(edge_detector.current_point_cloud)

    rospy.spin()
