# Subscribing PointClouds

* input: ```sensor_msgs::PointCloud2ConstPtr& input```

* Create an empty PointXYZ::Ptr

```
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>)
```
* Convert ROS message to PCL 

```
pcl::fromROSMsg(*input, *cloud);
```


