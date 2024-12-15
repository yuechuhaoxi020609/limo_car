# Hybrid Control System Match from HRI-lab

## Run

```bash
source devel/setup.bash
roslaunch limo_bringup limo_start.launch
roslaunch limo_bringup limo_navigation_diff.launch
rosrun limo_recognition exploring.py
roslaunch astra_camera dabai_u3.launch
rosrun limo_recognition object_detect.py
```
