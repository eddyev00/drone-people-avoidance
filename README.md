# Smart Drone Navigation: People Avoidance Algorithm with YOLO & MiDaS

<p align="center">
    <img width="500" src="https://github.com/eddyev00/drone-people-avoidance/assets/155014106/19eb9f16-0f59-4325-89be-e2eff1bef17a" alt="Material Bread logo">
</p>




Implementation of an intelligent people avoidance algorithm designed for drones. The algorithm leverages computer vision techniques to enable drones to navigate through crowded spaces while avoiding collisions with people.

1. **Object Detection with YOLOv8**:
The drone identifies the person's location and bounding box using YOLOv8.

2. **Depth Estimation with MiDaS**:
MiDaS provides crucial spatial information by estimating depth within the detected bounding box.

3. **Proximity Calculation**:
A filtered median value from depth pixels determines when the drone should come to a halt.

4. **Adaptive Path Continuation**:
The drone triggers a flight course change if the person doesn't move away after a predefined time, ensuring safety.

5. **ROS Integration**:
The system is powered by the Robot Operating System (ROS), ensuring communication and coordination between different components.

6. **Optitrack for Positioning:**
   Optitrack is employed to precisely position the drone within the arena, enhancing overall navigation accuracy.

# Watch Demo

[![Drone People Avoidance Demo](http://img.youtube.com/vi/LktCZMAJIaI/0.jpg)](https://youtu.be/LktCZMAJIaI?si=TmuFmULtceWwwfze)

