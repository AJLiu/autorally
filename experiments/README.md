# Experiments

## Scripts

- `autoRallyTrackGazeboRandomPose`
This launch script takes in two arguments `num_runs` and `num_obstacles`.
The script will generate `num_obstacles` number of obstacles randomly picked from list of obstacles in `obstacle_types`. It will then randomly place the car `num_runs` number of times in the track and not in an obstacle, and append the numpy arrays (left_image, right_image) to a list. 

```
roslaunch autorally_gazebo autoRallyTrackGazeboSimNoJoystick.launch
roslaunch experiments autoRallyTrackGazeboRandomPose.launch num_runs:= num_obstacles:=
```

By default the launch script sets `num_runs=50`, `num_obstacles=30`

### Known issues

Gazebo hangs when trying to delete obstacles, so resetting obstacles does not work. If you want to run the script multiple times, restart the gazebo simulation.