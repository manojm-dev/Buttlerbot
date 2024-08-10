# Navigation


## Steps

1) Launch simulation

```
ros2 launch butlerbot_gazebo gazebo.launch.py 
```

2) Launch rviz
```
ros2 launch butlerbot_description visualize.launch.py 
```

3) Edit slam toolbox config and launch slam toolbox

- change slam toolbox mode in config file `butlerbot_localization/config/mapper_params_online_async.yaml`

```
mode: localization
```

- and launch it
```
ros2 launch butlerbot_localization slam_online_async.launch.py 
```

4) Launch the navigation
```
ros2 launch butlerbot_navigation navigation.launch.py
```

5) Give goal using `2D Goal Pose` 

## [Video](https://drive.google.com/file/d/1KVi663ozBQsKZ5WcuzwBk9A3OM_IcRl1/view?usp=drive_link)