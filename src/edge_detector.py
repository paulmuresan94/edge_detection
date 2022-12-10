#!/usr/bin/env python3
import cv2

images = ['/home/paulm/Documents/neura_edge_detection/edge_detection/data/Image_1.png',
            '/home/paulm/Documents/neura_edge_detection/edge_detection/data/Image_2.png',
            '/home/paulm/Documents/neura_edge_detection/edge_detection/data/Image_3.png',
            '/home/paulm/Documents/neura_edge_detection/edge_detection/data/Image_4.png',
            '/home/paulm/Documents/neura_edge_detection/edge_detection/data/Image_5.png']

for image in images:
    # Read the original image
    img = cv2.imread(image) 
    # Display original image
    cv2.imshow('Original', img)
    cv2.waitKey(0)

    # Convert to graycsale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Blur the image for better edge detection
    img_blur = cv2.bilateralFilter(img_gray, 9, 120, 120)

    # Canny Edge Detection
    edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
    # Display Canny Edge Detection Image
    cv2.imshow('Canny Edge Detection', edges)
    cv2.waitKey(0)

    cv2.destroyAllWindows()
