# jsk\_rviz\_plugins

## Introduction
jsk\_rviz\_plugins is a package to provide original rviz plugins.

You can use this rviz plugins just launch rviz.

## rviz\_plugins

### Displays
#### AmbientSound
#### Diagnostics
#### FootStep
#### Normal

##### What Is This

This will show the Normal which is subcribed from topic (sensor_msgs::PointCloud2).
The normal is assumed to have the features x,y,z,normal\_x,normal\_y,normal\_z.

![Normal Plugin](cfg/image/normal_sample.png "Normal Plugins in RViz")

##### Samples
Plug the depth sensor which could be launched by openni.launch and execute below command.

```
roslaunch jsk_rviz_plugins normal_sample.launch
```

---

#### OverlayText
#### PieChart
#### Plotter2D
##### What Are These


These will show text or graph on the rviz main view.

![Plotter2D PieChart OverlayText Plugin](cfg/image/overlay_sample.png "Overlay Plugins in RViz")

##### Samples

Just run below commands

```
roslaunch jsk_rviz_plugins overlay_sample.launch
```

---

#### PolygonArray
![PolygonArray](images/polygon_array.png)

Visualize `jsk_pcl_ros/PolygonArray` message

##### Properties
* `Topic`

  Name of topic of `jsk_pcl_ros/PolygonArray`
* `auto color`

  If it's true, color of polygons are automatically changed
* `Color`

  Color of polygons, only enabled if `auto color` is false
* `Alpha`

  Transparency of polygons
* `only border`

  Draws only edges of polygons.
* `show normal`

  Show normal of polygons.
* `nromal length`

  Lenght of normal [m].

#### TorusArray
![TorusArray](images/torus_array.png)

Visualize `jsk_pcl_ros/TorusArray` message

##### Properties
* `Topic`

  Name of topic of `jsk_pcl_ros/TorusArray`
* `auto color`

  If it's true, color of polygons are automatically changed
* `Color`

  Color of polygons, only enabled if `auto color` is false
* `Alpha`

  Transparency of polygons
* `uv-smooth`

  Smoothness the surface
* `show normal`

  Show normal of toruses.
* `nromal length`

  Lenght of normal [m].


#### Pictogram
![Pictogram](images/pictogram.png)

[movie](https://www.youtube.com/watch?v=AJe1uQupsUE)

Pictogram is a rviz plugin to visualize icons.
Pictogram plugin uses [Entypo](http://entypo.com/) and [FontAwesome](http://fortawesome.github.io/Font-Awesome/).

You need to use `jsk_rviz_plugins/Pictogram` message to use it.

You can find mapping with `character` and icons at [here](http://fortawesome.github.io/Font-Awesome/icons/) and [here](http://entypo.com/characters/).

### Panels
#### CancelAction
![CancelAction](images/cancel_action.png)

This will publish action_msg/GoalID to `topic_name`/cancel.
You can choose multiple cancel goals.

#### RecordAction
![RecordAction](images/record_action.png)

This will publish jsk_rviz_plugins/RecordCommand to /record_command.
Set the target name.

#### EmptyServiceCallInterfaceAction
#### ObjectFitOperatorAction
![ObjectFitOperatorAction](images/object_fit_operator_action.png)

This will publish jsk_rviz_plugins/ObjectFitCommand to /object_fit_command".
If you check `reversed`, the reversed version will publish.

#### PublishTopic
![PublishTopic](images/publish_topic.png)

This will publish std_msgs/Empty to the topic you designate.
#### RobotCommandInterfaceAction
![RobotCommandInterfaceAction](images/robot_command_interface_action.png)

This will call service to /eus_command with jsk_rviz_plugins/EusCommand srv.

#### SelectPointCloudPublishAction
![SelectPointCloudPublishAction](images/select_point_cloud_publish_action.png)

This will publish sensor_msgs/PointCloud2 to /selected_pointcloud.
1. First, push `Select`Button and select the pointcloud region(Note that you need to choose only pointcloud. Don't include other parts).
2. Secord, push the SelectPointCloudPublishAction button.
3. Then the selected pointcloud will be published.

#### TransformableMarkerOperatorAction
![TransformableMarkerOperatorAction](images/transformable_marker_operator_action_insert.png)
![TransformableMarkerOperatorAction](images/transformable_marker_operator_action_erase.png)

This will call service to /request_marker_operate to insert/erase transformable_object
