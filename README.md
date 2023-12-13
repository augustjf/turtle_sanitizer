# Turtle Sanitizer

## Run
To run code:
```
ros2 run turtle_sanitizer turtle_sanitizer
```

In order to play rosbag containing the map, run in another terminal:
```
ros2 bag play map_bag/map_bag.db3 -l
```

Image of map will be published on topic /map_image. Can be viewed in Rviz. To see histogram, uncommet the line calling the histogram function in the map_timer_callback-function. Histogram will be saved in the root of the workspace.