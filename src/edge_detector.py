#!/usr/bin/env python3
import cv2
import numpy as np


class EdgeDetector:

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

    def overlay_edges(self, img):
        """
        Detects edges in a given open cv image and returns the input image with edges highlighted green on top
        """
        edges = self._detect_edges(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)
        img_edges = cv2.add(img, edges)
        return img_edges



if __name__ == "__main__":
    images = ['edge_detection/data/Image_1.png',
            'edge_detection/data/Image_2.png',
            'edge_detection/data/Image_3.png',
            'edge_detection/data/Image_4.png',
            'edge_detection/data/Image_5.png']

    edge_detector = EdgeDetector()

    for image in images:
        # Display original image
        img = cv2.imread(image) 
        cv2.imshow('Original', img)
        cv2.waitKey(0)

        # Display Edge Detection Image
        img_edges = edge_detector.overlay_edges(img)
        cv2.imshow('Canny Edge Detection', img_edges)
        cv2.waitKey(0)

        cv2.destroyAllWindows()
