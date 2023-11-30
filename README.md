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
Depending on whether a bang-bang mode or proportional mode is wanted, one publishes the following to the following topics:
- /motion (String)
 - "forward"
 - "backward"
 - "clockwise"
 - "counterclockwise"
 - "stop"
- /steering (Float32)
 - positive value: turn right
 - negative value: turn left
 - 0: go forward
