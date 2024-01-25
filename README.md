## Installation

```
git pull https://github.com/DrKrusto/MORO-Project
```

Download the file UE1.bag and put it inside the repo<br>
https://moodle.technikum-wien.at/mod/resource/view.php?id=1484507

# Launching the project
There is a script called "start.sh" at the root of the project which can be invoked like this to execute the simulation and code: ./start.sh

# Work split
- Line follower main_detection: Evan Roussin
- Line followr main_motion: Markus Luftensteiner
- Autonomous drive + path planning: Baptiste Lorenzi

# Switching between steering methods in main_motion
Depending on the operating mode set in the server ("steering_method" being "bangbang" or "proportional") the callback function of one of the following topics is used to steer the robot:
- /line_position (Int32)
 - positive value: turn right
 - negative value: turn left
 - 0: go forward
- /line_x_position (Float32)
 - positive value: turn right
 - negative value: turn left
 - 0: go forward

To switch the callback function used the function "switch_steering_method" can be called to switch between "bangbang" and "proportional".
