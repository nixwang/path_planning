# path_planning
Quadcopter path planning using RRT* and minimum jerk trajectory generation

![path_planning](https://user-images.githubusercontent.com/4923897/27253141-7f34cf18-538c-11e7-9fab-bad4d44f7c6c.gif)

Red represents minimum jerk trajectory while green represents a bspline trajectory through the visibility graph generated from RRT*


先启动launch文件

启动包，即odom_callback

启动发布话题节点
rostopic pub /clicked_point geometry_msgs/PointStamped \ '{header: {frame_id: "map"},point: {x: 23.79252910614,y: 0.434817790985,z: 20.00178050994873}}'

要先建完图？
rostopic pub /clicked_point geometry_msgs/PointStamped \ '{header: {frame_id: "world"},point: {x: 3.79252910614,y: 0.034817790985,z: 0.00178050994873}}'
rostopic pub /clicked_point geometry_msgs/PointStamped \ '{header: {frame_id: "world"},point: {x: -13.79252910614,y: 0.034817790985,z: 25.00178050994873}}'
