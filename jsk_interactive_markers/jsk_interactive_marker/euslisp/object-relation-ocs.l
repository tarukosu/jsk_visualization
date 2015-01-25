#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive_marker")
(ros::roseus "object_relation_ocs")
(require "models/room73b2-scene.l")
(require "package://tarukosu_drc_valve_task/euslisp/rotational-joint-model.l")
(require "package://tarukosu_drc_valve_task/euslisp/cylinder-model.l")
(require "package://jsk_interactive_marker/euslisp/object-relation.l")
(ros::roseus-add-msgs "jsk_rviz_plugins")

(setq *ocs* t)

(while (ros::ok)
  (x::window-main-one)
  (ros::spin-once)
  )
(ros::spin)
