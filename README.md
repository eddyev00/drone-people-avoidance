# Smart Drone Navigation: People Avoidance Algorithm with YOLO & MiDaS

[![Drone People Avoidance Demo](http://img.youtube.com/vi/LktCZMAJIaI/0.jpg)](https://youtu.be/LktCZMAJIaI?si=TmuFmULtceWwwfze)

Implementation of an intelligent people avoidance algorithm designed for drones. The algorithm leverages computer vision techniques to enable drones to navigate through crowded spaces while avoiding collisions with people.

1. Object Detection with YOLOv8:
The drone identifies the person's location and bounding box using YOLOv8.

2. Depth Estimation with MiDaS:
MiDaS provides crucial spatial information by estimating depth within the detected bounding box.

3. Proximity Calculation:
A filtered median value from depth pixels determines when the drone should come to a halt.

4. Adaptive Path Continuation:
The drone triggers a flight course change if the person doesn't move away after a predefined time, ensuring safety.

5. ROS Integration:
The system is powered by the Robot Operating System (ROS), ensuring communication and coordination between different components.

https://www.youtube.com/watch?v=LktCZMAJIaI
