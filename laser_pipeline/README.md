* Create catkin package

```
catkin_create_pkg my_pcl_tutorial pcl_conversions pcl_ros roscpp sensor_msgs 
```

* Add dependencies to package.xml

```
<build_depend>libpcl-all-dev</build_depend>
<exec_depend>libpcl-all</exec_depend>
```