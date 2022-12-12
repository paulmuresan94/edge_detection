# Edge Detection

## Introduction
This is my solution to the neura edge detection test. 

DISCLAIMER: Unfortunately I couldnt execute RViz correctly in my VM due to VRAM limitations (max. 128 MB for Windows host). Therefore I implemented only task 1 & 2 as far as possible without RViz. Due to time constrains I couldnt fix a bug and implement the rgbd to point cloud conversion. With a little more time I could implement those too. 

## Dependencies
- OpenCV 4.2.0
- Numpy 1.23.5
- ROS Noetic
- [edge_detection_msgs](https://github.com/paulmuresan94/edge_detection_msgs)

## Installation
- Download and install all dependencies. Add the package edge_detection_msgs and apply `catkin_make` and rebuild your enviroment wiht `. devel/setup.bash`.
- Packages can be started now. `test_edge_detector.py` in *edge_detection* represents the client and `edge_detector.py` the server. Please note the last chapter for this functionality. 

## Implementation steps
- First I implemented the edge detection only with open cv and tested it with the given test data to verify the basic functionality.
- Then I inserted the implemented algorithm in a ROS compatible format for the next implementation steps.
- I implemented the .srv in a separate package, so that it can be imported independently. 
- Afterwards I added the server and the client functionality to the edge detection package and had to cope with a bug (next chapter).
- With the remaining time I analyzed the bag file with tools given by ROS and implemented the needed subscribers/publisher.

## Possible improvements
- Bug regading service: `test_edge_detector.py` leads to a crash of `edge_detector.py` when started under normal condition (`rosrun`). Strangly, when `edge_detector.py` is started in visual studio code in debug mode both behave as expected (except right before shut down).
- RGBD to pointd cloud conversion needs to be added.
- Architecturally speaking: service isn't the best choise, because it blocks all subsribers and publisher of given node. Therefore I would recommend (for this case) to replace service with action.
