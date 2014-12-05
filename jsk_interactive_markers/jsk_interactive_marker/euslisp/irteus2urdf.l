#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
;;(load "package://hrpsys_gazebo_tutorials/euslisp/hrpsys-gazebo-utils.l")
(ros::roseus "irteus2urdf")

(defclass irteus2urdf
  :slots (topic-name)
  )


(defmethod irteus2urdf
  (:init
   (tname)
   (setq topic-name tname)
   (ros::advertise topic-name std_msgs::String)
   )

  (:irteus2urdf
   (model model-name)
   (setq tmp-dir "/tmp")
   ;;
   (cond
    ((not (subclassp (class model) cascaded-link))
     (format t "invalid class, not childclass of cascaded-link~%"))
    ((null (send model :links)) (format t "please setq links~%"))
    (t
     (setq name (send model :name))
     (if (or (not (stringp (send model :name)))
     	     (zerop (length (send model :name))))
     	 (send model :name
     	       (remove #\: (format nil "~A" (send model :name)))))
     (collada::eus2collada model tmp-dir)
     (cond
      ((probe-file (format nil "~A/~A.dae" tmp-dir (send model :name)))
       (format t "generate ~A.dae~%convert to urdf~%" (send model :name))
       (unix:system
	(format nil
		(concatenate
		 string
		 "rosrun hrpsys_gazebo_tutorials"
		 " eus2urdf_for_gazebo_pyscript.py ~a ~a/~a.dae")
;;		(send model :name) tmp-dir (send model :name)
		model-name tmp-dir (send model :name)
;;		(send model :name) (send model :name)
		))
       (return-from :irteus2urdf t)
       )
      (t (format nil "dae error~%")))
     ;;(send model :name name)
     )))



  (:irteus2urdfmsg
   (model)
   (setq model-name (format nil "dynamic_~A" (send model :name)))

   (when (send self :irteus2urdf model model-name)
     
   ;; (convert-rbrain-model-to-collada model :model-name model-name :output-directory tmp-dir)
   ;; (unix:system (format nil "rosrun hrpsys_gazebo_tutorials eus2urdf_for_gazebo_pyscript.py ~a ~a/~a-robot.dae" model-name tmp-dir model-name))

     (let ((urdf-file (format nil "~A/environment_models/~A/model.urdf" (ros::resolve-ros-path "package://hrpsys_gazebo_tutorials") model-name))
	   (eof (gensym))
	   (msg (instance std_msgs::String :init))
	   urdf-list
	   lline
	   )

       (with-open-file (f urdf-file :direction :input)
		       (while (not (eq eof (setq lline (read-line f nil eof))))
			 (push lline urdf-list)
			 )
		       )
       (setq urdf-list (reverse urdf-list))
       (send msg :data (apply #'concatenate string urdf-list))
       msg
       ))
   )

  (:publish-urdf-model
   (model)
   (ros::publish topic-name (send self :irteus2urdfmsg model))
   )
  )

(load "models/arrow-object.l")
(defun test
  ()
  (setq *irteus2urdf* (instance irteus2urdf :init "/jsk_model_marker_interface/object/set_urdf"))
  (setq *model* (arrow))

  (bench
   (send *irteus2urdf* :publish-urdf-model *model*)
   )
  )

