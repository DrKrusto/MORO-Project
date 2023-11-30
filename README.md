## Installation

git pull
```
git pull https://github.com/DrKrusto/MORO-Project
```

Download the file UE1.bag and put it inside the repo<br>
https://moodle.technikum-wien.at/mod/resource/view.php?id=1484507

# Work split
Line follower main_detection: Evan Roussin
Line followr main_motion: Markus Luftensteiner
Autonomous drive + path planning: Baptiste Lorenzi

# Switching between steering methods in main_motion
Depending on the operating mode set in the server one of the following topics is used to steer the robot:
- /line_position (String)
 - "centre"
 - "right"
 - "left"
 - "stop"
- /line_x_position (Float32)
 - positive value: turn right
 - negative value: turn left
 - 0: go forward
