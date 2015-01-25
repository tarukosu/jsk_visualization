#!/bin/usr/env roseus
(ros::roseus "topic_publish_with_request")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "std_msgs")

(defun empty-cb(msg)
  (if *latest-pose*
      (ros::publish *republish-pose-topic* *latest-pose*)
      )
  )

(defun pose-cb(msg)
  (setq *latest-pose* msg)
  )

(setq *latest-pose* nil)
(setq *republish-pose-topic* "/bounding_box_cropper_pose")
(ros::advertise *republish-pose-topic* geometry_msgs::PoseStamped 5)
;;(ros::subscribe "/publish_topic_request" std_msgs::Empty #'empty-cb)
(ros::subscribe "/selected_point_cloud" std_msgs::Empty #'empty-cb)
(ros::subscribe "/marker_6dof/pose" geometry_msgs::PoseStamped #'pose-cb)

(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
