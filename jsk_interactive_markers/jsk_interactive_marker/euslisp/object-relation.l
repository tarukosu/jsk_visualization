#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive_marker")
(ros::roseus "object_relation")
(require "models/room73b2-scene.l")
(require "package://tarukosu_drc_valve_task/euslisp/rotational-joint-model.l")
(require "package://tarukosu_drc_valve_task/euslisp/cylinder-model.l")
(ros::roseus-add-msgs "jsk_rviz_plugins")

(setq *model* (room73b2-bottle))

(setq *target-color* (float-vector 0 0.8 1.0))

(setq *send-object* nil)

(defclass send-object
  :slots
  (model grasp-frame-id    grasp-model    grasp-index    grasp-link
	 ))

(defmethod send-object
  (:init
   (m gf gm gi gl)
   (setq model m)
   (setq grasp-frame-id gf)
   (setq grasp-model gm)
   (setq grasp-index gi)
   (setq grasp-link gl)

   )
  )

(defclass object-part
  :slots
  (marker-id frame-id model index link-index color distance
	     grasp-coords grasp-coords-offset rotation-axis))

(defmethod object-part
  (:init
   (&optional fid (m nil) i li c dist &key (id 0))
   (setq marker-id id)
   (setq frame-id fid)
   (setq model m)
   (setq index i)
   (setq link-index li)
   (setq color c)
   (setq distance dist)
   (setq grasp-coords-offset (make-coords))
   )
  (:distance
   ()
   distance
   )
  (:set-distance
   (dist)
   (setq distance dist)
   )

  
  (:set-marker-id
   (mid)
   (setq marker-id mid))

  (:link-index
   ()
   link-index
   )

  (:index
   ()
   index
   )

  (:model
   ()
   model
   )

  (:frame-id
   ()
   fid
   )

  (:publish-marker
   ()
   (when model
     (cond
      ((derivedp model face)
       (publish-face model :frame-id frame-id :name (send model :name) :color color)
       )
      )
     )
   )

  (:publish-grasp-marker
   ()
   (when model
     ;;(publish-face grasp-model :frame-id frame-id :name (send grasp-model :name))
     (send self :publish-grasp-distance-marker)
     (publish-grasp-interactive-marker
      (send self :grasp-coords)
      :frame-id frame-id :name (format nil "grasp_control/~A/~A/~A" "face" index marker-id))
     )
   )

  (:update-grasp-marker
   (feedback)
   (unless model
     (return-from :update-grasp-marker))
   (setq marker-config (split-string (send feedback :marker_name)))

   (print marker-id)
   (print (elt marker-config 3))

   (unless (equal (format nil "~A" marker-id) (elt marker-config 3))
     (return-from :update-grasp-marker))

   (setq feedback-coords (ros::tf-pose->coords (send feedback :pose)))
   (setq grasp-coords (send self :grasp-coords))
   (setq local-vec (send grasp-coords :inverse-transform-vector (send feedback-coords :worldpos)))
   (print local-vec)
   (setq thre 1)
   (setq update-distance (- (/ (elt local-vec 0) 20.0)))

   (when (> (abs (elt local-vec 0)) thre)
     (setq distance (+ distance update-distance))
     (when (< distance 0)
       (setq distance 0)
       )
     )
   (send self :publish-grasp-distance-marker)

   (when (equal (send feedback :event_type) visualization_msgs::InteractiveMarkerFeedback::*MOUSE_UP*)
     (send self :publish-grasp-marker)
     )
   )

  (:publish-grasp-distance-marker
   ()
   (print model)
   (when model
     (cond
      ((derivedp model face)
       (ros::publish
	*grasp-marker-topic*
	(object->marker-msg
	 (make-prism (cdr (send model :vertices)) (scale distance (send model :normal)))
	 (instance std_msgs::Header :init :frame_id frame-id)
	 :color color
	 :alpha 0.6
	 :id marker-id)
	))
      ((derivedp model edge)
       (ros::publish
	*grasp-marker-topic*
	(edge->marker-msg model :color color :alpha 0.6 :id marker-id :frame-id frame-id :radius distance)
	))
      ((derivedp model line)
       (ros::publish
	*grasp-marker-topic*
	(edge->marker-msg model :color color :alpha 0.6 :id marker-id :frame-id frame-id :radius distance)
	))

      )
     )
   )

  (:delete-grasp-distance-marker
   ()
   (ros::publish
    *grasp-marker-topic*
    (instance visualization_msgs::Marker :init :header (instance std_msgs::Header :init :frame_id frame-id) :id marker-id :action visualization_msgs::Marker::*DELETE*)
    )
   )


  (:rotation-axis
   ()
   (cond
    (rotation-axis
     rotation-axis
     )
    ((derivedp model line)
     :z
     )
    (t
     t))
   )

  (:change-grasp-coords
   ()
   ;;line
   (cond
    ((derivedp model line)
     (print "grasp-coords-offset")
     (print grasp-coords-offset)
     (send grasp-coords-offset :rotate (deg2rad -20) :z)
     )
    )
   )

  (:set-grasp-coords
   (cds)
   (setq grasp-coords cds)
   (cond
    ;;line
    ((derivedp model line)
     (setq grasp-coords cds)
     )
    )
   )

  (:set-grasp-coords-offset
   (cds)
   (setq grasp-coords-offset cds)
   )

  (:set-rotation-axis
   (axis)
   (setq rotation-axis axis)
   )

  (:grasp-coords
   ()
   ;;(setq offset (float-vector 68 0 0))
   (setq offset (float-vector 64 0 0))
   (when model
     (cond
      (grasp-coords
       (send (send grasp-coords :copy-worldcoords) :transform grasp-coords-offset)
       )
      ;;face
      ((derivedp model face)

       (setq max-len 0)
       (dolist (v (send model :edges))
	 (when (> (send v :length) max-len)
	   (setq max-len (send v :length))
	   (setq z-vec (send v :direction))))
       (setq cds (make-coords :pos (cadr (send model :centroid))
			      :rot (xz-vec->rotation (v- (send model :normal)) z-vec)
			      ))
       (send cds :locate offset)
       )
      ;;edge
      ((derivedp model edge)
       (setq pface-vec (float-vector 0 0 0))
       (setq nface-vec (float-vector 0 0 0))

       (when (car (send model :faces))
	 (setq pface-vec (send (car (send model :faces)) :normal)))

       (when (cadr (send model :faces))
	 (setq nface-vec (send (cadr (send model :faces)) :normal)))

       (setq x-vec (v- (midpoint 0.5 pface-vec nface-vec)))
       
       (setq cds (make-coords :pos  (midpoint 0.5 (car (send model :vertices)) (cadr (send model :vertices)))
			      :rot (xz-vec->rotation 
				    x-vec
				    (send model :direction))
			      ))
       (send cds :locate offset)
       )
      ;;line
      ((derivedp model line)
       (setq cds (make-coords :pos  (midpoint 0.5 (car (send model :vertices)) (cadr (send model :vertices)))
		    :rot (z-vec->rotation (send model :direction))
		    ))
       )
      )
     )
   )

  (:coords
   ()
   (when model
     (make-coords :pos (cadr (send model :centroid))
		  :rot (z-vec->rotation (send model :normal)))
     )
   )
  )


(defclass object-relation
  :slots
  (model frame-id relation
	 selected-part
	 grasp-larm-part
	 grasp-rarm-part
	 selection-frame-id selection-model selection-color selection-link-index
	 selection-flag
	 ))

(defmethod object-relation
  (:init
   (&optional m f r)
   (setq model m)
   (setq frame-id f)
   (setq relation r)
   ;;(setq grasp-distance 100)
   (setq selected-part (instance object-part :init))
   (setq grasp-larm-part (instance object-part :init frame-id))
   (setq grasp-rarm-part (instance object-part :init frame-id))
   (setq selected-part (instance object-part :init ))

   (send grasp-larm-part :set-marker-id 0)
   (send grasp-rarm-part :set-marker-id 1)
   (send grasp-larm-part :delete-grasp-distance-marker)
   (send grasp-rarm-part :delete-grasp-distance-marker)

   (cond
    ((equal relation "target")
     (setq selected-color (float-vector 1.0 0.6 0.6))
     (setq selection-color (float-vector 1.0 0 0))
     (setq grasp-color (float-vector 1.0 0.8 0.0))
     )
    ((equal relation "reference")
     (setq selected-color (float-vector 0.6 1.0 0.6))
     (setq selection-color (float-vector 1.0 0 0))
     )
    )
   (ros::advertise (format nil "/jsk_model_marker_interface/~A/set_pose" (send model :name))
		   geometry_msgs::PoseStamped)

   )

  (:grasp-part
   (arm)
   (cond
    ((equal arm :larm)
       grasp-larm-part
       )
    ((equal arm :rarm)
       grasp-rarm-part
       )))

  (:grasp-larm-part
   ()
   grasp-larm-part
   )

  (:grasp-rarm-part
   ()
   grasp-rarm-part
   )

  (:send-object
   ()
   (instance send-object :init model grasp-frame-id grasp-model grasp-index grasp-link)
   )

  (:set-selection
   (f m flag link-index)
   (setq selection-frame-id f)
   (setq selection-model (send (copy-object m) :move-to (make-coords) :world))
   (send selection-model :clear-assoc)
   (setq selection-flag flag)
   (setq selection-link-index link-index)
   )

  (:model
   ()
   model
   )

  (:name
   ()
   (when model
     (send model :name)
     )
   )

  (:frame-id
   ()
   frame-id
   )

  (:publish-marker
   ()
   (unless model
     (return-from :publish-marker))
   (cond
    (selection-flag
     (send self :publish-selection-marker selection-flag)
     )
    (t
     (send self :publish-selected-marker)
     )
    ))

  (:publish-selection-marker
   (prefix)
   (unless selection-model
     (return-from :publish-selection-marker))

   ;;point
   (setq point-num 0)
   (dolist (p (flatten (send-all (send selection-model :bodies) :vertices)))
     (publish-point p :frame-id selection-frame-id :name  (send self :get-marker-name prefix "point" point-num) :color selection-color)
     (setq point-num (+ point-num 1))
     )


   ;;axis
   (setq index 0)
   (dolist (b (send-all (send selection-model :bodies) :box))
     (setq center (send b :center))
     (setq dia-scale 1.3)
     ;;(setq dia (scale dia-scale (scale 0.5 (send b :diagonal))))
     (setq dia (scale 0.5 (send b :diagonal)))
     (setq x-vec (float-vector (elt dia 0) 0 0))
     (setq y-vec (float-vector 0 (elt dia 1) 0))
     (setq z-vec (float-vector 0 0 (elt dia 2)))

     (dolist (vec (list x-vec y-vec z-vec))
       (cond
	((< (* (- dia-scale 1.0) (norm vec)) 30)
	 (setq vec (scale (/ (+ 30 (norm vec)) (norm vec)) vec)))
	(t
	 (setq vec (scale dia-scale vec))
	 ))
       (publish-line (make-line (v- center vec) (v+ center vec))
		     :frame-id selection-frame-id :name  (send self :get-marker-name prefix "line" index) :color #f(1 1 0))
       (setq index (+ index 1))
       )
     )

   ;;edge
   (setq edge-num 0)
   (dolist (e (flatten (send-all (send selection-model :bodies) :edges)))
     (publish-line e :frame-id selection-frame-id :name  (send self :get-marker-name prefix "edge" edge-num) :color #f(1.0 0.6 0))
     (setq edge-num (+ edge-num 1))
     )

   ;;(return-from :publish-selection-marker)

   ;;face
   (setq face-num 0)
   (dolist (f (send selection-model :faces))
     (publish-face f :frame-id selection-frame-id :name (send self :get-marker-name prefix "face" face-num) :color #f(0.5 1 0.1))
     (setq face-num (+ face-num 1))
     ))

  (:publish-selected-marker
   ()
   (send selected-part :publish-marker)
   (send grasp-larm-part :publish-grasp-marker)
   (send grasp-rarm-part :publish-grasp-marker)
   )

  (:update-grasp-marker
   (feedback)
   (unless model
     (return-from :update-grasp-marker))
   (send grasp-larm-part :update-grasp-marker feedback)
   (send grasp-rarm-part :update-grasp-marker feedback)
   ;;(publish-all-markers)
   )

  (:get-marker-name
   (prefix marker-type index)
   (format nil "~A/~A/~A" prefix marker-type index)
   )

  (:get-selection-model
   (selection-type index)

   (cond
    ((equal selection-type "face")
     (setq m (elt (send selection-model :faces) index)))
    ((equal selection-type "edge")
     (setq m (elt (flatten (send-all (send selection-model :bodies) :edges)) index)))
    ((equal selection-type "line")
     ;;(setq m (elt (flatten (send-all (send selection-model :bodies) :edges)) index)))
     (setq line-index 0)
     (dolist (b (send-all (send selection-model :bodies) :box))
       (setq center (send b :center))
       (setq dia-scale 1.3)
       (setq dia (scale dia-scale (scale 0.5 (send b :diagonal))))

       (setq x-vec (float-vector (elt dia 0) 0 0))
       (setq y-vec (float-vector 0 (elt dia 1) 0))
       (setq z-vec (float-vector 0 0 (elt dia 2)))

       (dolist (vec (list x-vec y-vec z-vec))
	 (when (equal index line-index)
	   (setq m (make-line (v- center vec) (v+ center vec)))
	   )
	 (setq line-index (+ line-index 1))
	 )
       )
     )
    )
   m
   )

  (:selection
   (prefix marker-type index)
   (unless selection-model
     (return-from :selection))
   
   ;;(setq m (elt (send selection-model :faces) index))
   (setq m (send self :get-selection-model marker-type index))
   (send m :name (send self :get-marker-name prefix marker-type index))

   (cond
    ((equal prefix "target")
     (setq selected-part
	   (instance object-part :init selection-frame-id m index selection-link-index (float-vector 1.0 0.6 0.6)))
     (setq selection-flag nil)
     )
    ((equal prefix "reference")
     (setq selected-part
	   (instance object-part :init selection-frame-id m index selection-link-index (float-vector 0.6 0.6 1.0)))
     )
    ((equal prefix "grasp_point_larm")
     (setq grasp-larm-part
	   (instance object-part :init selection-frame-id m index selection-link-index (float-vector 0.4 0.7 0.7)  20 :id 0))
     )
    ((equal prefix "grasp_point_rarm")
     (setq grasp-rarm-part
	   (instance object-part :init selection-frame-id m index selection-link-index (float-vector 0.7 0.4 0.4)  20 :id 1))
     )

    (t
     (setq grasp-index index)
     (setq grasp-model m)

     ))
   (setq selection-flag nil)
   )

  (:selected-model
   ()
   (send selected-part :model)
   )

  (:selected-coords
   ()
   (setq tmp-coords (send model :copy-worldcoords))
   (send model :move-to (make-coords) :world)
   (setq cds (send (elt (send model :links) (send selected-part :link-index)) :copy-worldcoords))
   (send cds :transform (send selected-part :coords))
   (send model :move-to tmp-coords :world)
   cds
   )

  ;; (:grasp-coords
  ;;  ()
  ;;  (when grasp-model
  ;;    (make-coords :pos (cadr (send grasp-model :centroid))
  ;; 		  :rot (z-vec->rotation (send grasp-model :normal)))
  ;;    )
  ;; )


  (:move-to
   (cds cds-frame-id)
   (setq ps (instance geometry_msgs::PoseStamped :init))
   (send ps :header :frame_id cds-frame-id)
   (send ps :pose (ros::coords->tf-pose cds))

   (print-ros-msg ps)
   (ros::publish (format nil "/jsk_model_marker_interface/~A/set_pose" (send model :name)) ps)
   )

  (:set-controls
   (cds &key (controls-list (list t t nil nil nil t)) (show nil))
   (send cds :locate #f(0 0 0))
   (setq req (instance jsk_interactive_marker::SetInteractiveMarkerControlsRequest :init))

   (setq imcs nil)
   

   ;; x-translation
   (when (elt controls-list 0)
     (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     (send imc :always_visible t)
     (setq control-cds (send cds :copy-worldcoords))
     (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
     (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*MOVE_AXIS*)
     (push imc imcs)
     )
   (when (elt controls-list 3)
     (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     (send imc :always_visible t)
     (setq control-cds (send cds :copy-worldcoords))
     (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
     (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*ROTATE_AXIS*)
     (push imc imcs)
     ;;(setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     ;;(send imc :always_visible t)
     )


   ;; y-translation
   (when (elt controls-list 1)
     (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     (send imc :always_visible t)
     (setq control-cds (send (send cds :copy-worldcoords) :rotate pi/2 :z))
     ;;(setq cds (make-coords :angle pi/2 :axis :z))
     (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
     (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*MOVE_AXIS*)
     (push imc imcs)

     ;;(setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     ;;(send imc :always_visible t)
     )
   (when (elt controls-list 4)
     (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     (send imc :always_visible t)
     (setq control-cds (send (send cds :copy-worldcoords) :rotate pi/2 :z))
     ;;(setq cds (make-coords :angle pi/2 :axis :z))
     (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
     (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*ROTATE_AXIS*)
     (push imc imcs)

     ;;(setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     ;;(send imc :always_visible t)
     )
   ;; z-rotation
   (when (elt controls-list 2)
     (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     (send imc :always_visible t)
     (setq control-cds (send (send cds :copy-worldcoords) :rotate pi/2 :y))
     (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
     (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*MOVE_AXIS*)
     (push imc imcs)
     )
   (when (elt controls-list 5)
     (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
     (send imc :always_visible t)
     (setq control-cds (send (send cds :copy-worldcoords) :rotate pi/2 :y))
     (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
     (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*ROTATE_AXIS*)
     (push imc imcs)
     )
   (print-ros-msg imcs)

   (ros::service-call (format nil "/jsk_model_marker_interface/~A/set_root_controls" (send model :name) ) (instance jsk_interactive_marker::SetInteractiveMarkerControlsRequest :init :controls imcs))

   (when show
     (ros::service-call (format nil "/jsk_model_marker_interface/~A/show_control" (send model :name)) (instance std_srvs::EmptyRequest :init))
     )
   )
  )

;;utils

(defun split-string (str &optional (split-char "/"))
  (setq string-list nil)
  (setq tmp-char-list nil)
  (dotimes (i (length str))
    (setq tmp-char (format nil "~C" (char str i)))
    (cond
     ((equal tmp-char split-char)
      (when tmp-char-list
	(setq tmp-char-list (reverse tmp-char-list))
	(push (reduce #'(lambda (val1 val2) (concatenate string val1 val2)) tmp-char-list) string-list))
      (setq tmp-char-list nil)
      )
     (t
      (push tmp-char tmp-char-list)
      )))
  (when tmp-char-list
    (setq tmp-char-list (reverse tmp-char-list))
    (push (reduce #'(lambda (val1 val2) (concatenate string val1 val2)) tmp-char-list) string-list))
  (reverse string-list)
  )

(defun z-vec->rotation (vec)
  (setq not-parallel-vec #f(1 0 0))
  (if (> (v. not-parallel-vec vec) 0.9)
      (setq not-parallel-vec #f(0 1 0)))
  (setq y-vec (normalize-vector (v* vec not-parallel-vec)))
  (setq x-vec (normalize-vector (v* y-vec vec)))

  (matrix (float-vector (elt x-vec 0) (elt y-vec 0) (elt vec 0))
	  (float-vector (elt x-vec 1) (elt y-vec 1) (elt vec 1))
	  (float-vector (elt x-vec 2) (elt y-vec 2) (elt vec 2)))
  )
(defun xz-vec->rotation (x-vec z-vec)
  (setq y-vec (normalize-vector (v* z-vec x-vec)))

  (matrix (float-vector (elt x-vec 0) (elt y-vec 0) (elt z-vec 0))
	  (float-vector (elt x-vec 1) (elt y-vec 1) (elt z-vec 1))
	  (float-vector (elt x-vec 2) (elt y-vec 2) (elt z-vec 2)))
  )

;;utils


(defun model-interactive-marker-feedback (msg)
  (publish-all-markers)
  )


(defun interactive-marker-feedback (msg)
  ;;(clear-all-markers)
  ;;(print-ros-msg msg)
  (setq marker-config (split-string (send msg :marker_name)))
  (print marker-config)

  (setq already-selected (and (send *target-object* :selected-model) (send *reference-object* :selected-model)))
  (setq marker-type (elt marker-config 1))

  (cond
   ((equal (elt marker-config 0) "target")
    (send *target-object* :selection (elt marker-config 0) marker-type (read-from-string (elt marker-config 2)))
    (clear-all-markers)
    (publish-all-markers)
    )
   ((equal (elt marker-config 0) "reference")
    (send *reference-object* :selection (elt marker-config 0) marker-type (read-from-string (elt marker-config 2)))
    (clear-all-markers)
    (publish-all-markers)
    )
   ((or (equal (elt marker-config 0) "grasp_point_larm") (equal (elt marker-config 0) "grasp_point_rarm"))
    (send *target-object* :selection (elt marker-config 0) marker-type (read-from-string (elt marker-config 2)))
    (clear-all-markers)
    (publish-all-markers)
    )
   ((equal (elt marker-config 0) "grasp_control")
    (send *target-object* :update-grasp-marker msg)
    )
   )

  (when (and (not already-selected) (send *target-object* :selected-model) (send *reference-object* :selected-model))

    (setq target-coords (send *target-object* :selected-coords))
    (setq reference-coords (send *reference-object* :selected-coords))
    (send target-coords :rotate pi :x)

    (send *target-object* :move-to (send target-coords :transformation reference-coords :world) (send *reference-object* :frame-id))

    (send *target-object* :set-controls (make-coords) :controls-list (list t t t nil nil t) :show t)
    ;;(send *target-object* :set-controls (make-coords) :controls-list (list t t nil nil nil t))
    )
  )


(defun publish-all-markers
  ()
  (send *target-object* :publish-marker)
  (send *reference-object* :publish-marker)
  )

(defun clear-all-markers
  ()
  (ros::publish *clear-marker-topic* (instance std_msgs::Empty :init))
  )

(defun object-relation-callback (msg)
  (print-ros-msg msg)

  (setq model-list (mapcar #'read-from-string (split-string (send msg :model) " ")))
  (cond
   ((functionp (car model-list))
    (setq model (apply (car model-list) (cdr model-list)))
    (send model :name (send msg :name))
    )
   (t
    (return-from object-relation-callback nil))
   )

  (setq frame-id (send msg :link))

  (cond
   ((> (length (send model :links)) 1)
    (setq selection-link-name (intern (car (split-string (elt (split-string (send msg :link)) 2) "_")) *keyword-package*))
    (dotimes (i (length (send model :links)))
      (setq l (elt (send model :links) i))
      (when (string-equal (send l :name) selection-link-name)
	(setq selection-m l)
	(setq selection-li i)
	(setq split-link (split-string (send msg :link)))
	(setq frame-id (format nil "~A/~A/~A_link" (elt split-link 0) (elt split-link 1) (string (send (car (send model :links)) :name))))
	(return)
	))
    )
   (t
    (setq selection-m model)
    (setq selection-li 0)
    )
   )
  (clear-all-markers)

  ;; test code for object-relation-fc.l
  (setq mm-msg (instance jsk_interactive_marker::MakeModel :init))
  (send mm-msg :model (send msg :model))
  ;; test code for object-relation-fc.l


  (cond
   ((equal (send msg :relation) "target")
    ;;target and reference are different
    (when (equal (send msg :name) (send *reference-object* :name))
      (setq *reference-object* (instance object-relation :init))
      )
    (unless (equal (send msg :name) (send *target-object* :name))
      (setq *target-object* (instance object-relation :init model frame-id (send msg :relation)))
      (ros::publish *publish-make-model-topic* mm-msg)
      
      (send *target-object* :set-controls (make-coords) :controls-list (list t t t t t t))
      ;;(send *target-object* :delete-grasp-distance-marker)
      )
    (send *target-object* :set-selection (send msg :link) selection-m (send msg :relation) selection-li)
    )
   ((equal (send msg :relation) "reference")
    ;;target and reference are different
    (when (equal (send msg :name) (send *target-object* :name))
      (setq *target-object* (instance object-relation :init))
      )
    (setq *reference-object* (instance object-relation :init model frame-id (send msg :relation)))
    (send *reference-object* :set-selection (send msg :link) selection-m (send msg :relation) selection-li)
    )

   ;;select "Set Grasp Point" menu
   ((or (equal (send msg :relation) "grasp_point_larm")
	(equal (send msg :relation) "grasp_point_rarm"))
    (unless (equal (send msg :name) (send *target-object* :name))
      (setq *target-object* (instance object-relation :init model frame-id "target"))
      (ros::publish *publish-make-model-topic* mm-msg)
      )
    (send *target-object* :set-selection (send msg :link) selection-m (send msg :relation) selection-li)
    )
   ((equal (send msg :relation) "grasp_larm")
    (grasp-object :larm)
    )
   ((equal (send msg :relation) "grasp_rarm")
    (grasp-object :rarm)
    )
   ((equal (send msg :relation) "release_larm")
    (release-object :larm)
    )
   ((equal (send msg :relation) "release_rarm")
    (release-object :rarm)
    )
   ((equal (send msg :relation) "reset")
    (send *target-object* :set-controls (make-coords) :controls-list (list t t t t t t))
    
    )
   ((equal (send msg :relation) "set_pose")
    (set-pose :rarm)
    (set-pose :larm)
    )
   )
  (publish-all-markers)
  )

(defun set-pose (arm)
  (unless (or (and *larm-grasp* (equal arm :larm))
	      (and *rarm-grasp* (equal arm :rarm)))
    (return-from set-pose))
  ;; test code for object-relation-fc.l
  (setq model (send *target-object* :model))
  (setq mm-msg (instance jsk_interactive_marker::MakeModel :init))
  (send mm-msg :pose (ros::coords->tf-pose-stamped (send model :worldcoords) "update"))
  (send mm-msg :type "set_pose")
  (ros::publish *publish-make-model-topic* mm-msg)
  (return-from set-pose)

  (unless model
    (return-from set-pose))
  (send model :move-to
	(pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) (send *target-object* :frame-id)) *robot-frame-id*) :world)

  (setq grasp-part (send *target-object* :grasp-part arm))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  
  ;;(send grasp-part :set-grasp-coords (send grasp-part :grasp-coords))
  (setq link-coords (send (elt (send model :links) (send grasp-part :link-index)) :copy-worldcoords))
  (send grasp-part :set-grasp-coords (send link-coords :transformation (send *pr2* arm :end-coords)))
)

(setq *publish-make-model-topic* "/make_model")
(ros::advertise *publish-make-model-topic*
		jsk_interactive_marker::MakeModel)


(defun grasp-object (arm &key (ik-check nil) (record t))
  ;; test code for object-relation-fc.l
  (when *ocs*
    (setq model (send *target-object* :model))
    (send model :move-to
	  (pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) (send *target-object* :frame-id)) *robot-frame-id*) :world)

    (setq grasp-part (send *target-object* :grasp-part arm))

    (setq mm-msg (instance jsk_interactive_marker::MakeModel :init))
    (send mm-msg :pose (ros::coords->tf-pose-stamped (send model :worldcoords) "update"))
    (cond
     ((equal arm :larm)
      (send mm-msg :type "grasp_larm")
      )
     ((equal arm :rarm)
      (send mm-msg :type "grasp_rarm")
      )
     )
    (send mm-msg :grasp_type (format nil "~A" (send (class (send grasp-part :model)) :name)))
    (send mm-msg :grasp_index (send grasp-part :index))
    (send mm-msg :grasp_link_index (send grasp-part :link-index))
    (send mm-msg :grasp_distance (send grasp-part :distance))

    (ros::publish *publish-make-model-topic* mm-msg)
    (return-from grasp-object)
    )

	


  ;; test code for object-relation-fc.l

  (when (and (equal *recording* "model") record)
    (print "recording")
    (push (list 'grasp-object arm) *recorded-movement*)
    )

  (print "GRASP")
  (setq grasp-part (send *target-object* :grasp-part arm))
  (setq model (send *target-object* :model))
  (send model :move-to
	(pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) (send *target-object* :frame-id)) *robot-frame-id*) :world)

  (send grasp-part :set-grasp-coords (send grasp-part :grasp-coords))

  
  (when (send grasp-part :model)
    (setq prev-av (send *pr2* :angle-vector))
    ;;(objects (list *pr2* grasp-coords model))
    

    (dotimes (i 10)
      (setq grasp-coords (send (elt (send model :links) (send grasp-part :link-index)) :copy-worldcoords))
      (send grasp-coords :transform (send grasp-part :grasp-coords))

      (setq pre-grasp-coords (send grasp-coords :copy-worldcoords))
      (setq pre-grasp-coords (send pre-grasp-coords :locate (float-vector -100 0 0)))

      ;;look at
      (send *pr2* :head :look-at (send grasp-coords :worldpos))

      (cond
       ((equal arm :larm)
	(setq rotation-axis (list (send grasp-part :rotation-axis) t))
	(setq target-coords (list grasp-coords (send *pr2* :rarm :end-coords :copy-worldcoords))))
       ((equal arm :rarm)
	(setq rotation-axis (list t (send grasp-part :rotation-axis)))
	(setq target-coords (list (send *pr2* :larm :end-coords :copy-worldcoords) grasp-coords)))
       )

      (cond
       ((equal arm :larm)
	(setq pre-target-coords (list pre-grasp-coords (send *pr2* :rarm :end-coords :copy-worldcoords))))
       ((equal arm :rarm)
	(setq pre-target-coords (list (send *pr2* :larm :end-coords :copy-worldcoords) pre-grasp-coords)))
       )

      ;;move slowly
      (setq middle-target-coords-list nil)
      (setq middle-target-time-list nil)
      (setq distance (send grasp-part :distance))
      (print distance)
      (setq last-flag nil)
      (dotimes (i 100)
	(setq middle-grasp-coords (send grasp-coords :copy-worldcoords))
	(print "dist")
	(print distance)
	(setq middle-grasp-coords (send middle-grasp-coords :locate (float-vector (- distance) 0 0)))
	(setq distance (- distance 20))
	(print distance)

	(if (equal distance -20)
	    (setq last-flag t)
	  )
	(when (< distance 0)
	  (setq distance 0)
	  ;;(setq last-flag t)
	  )
	
	(cond
	 ((equal arm :larm)
	  (setq middle-target-coords (list middle-grasp-coords (send *pr2* :rarm :end-coords :copy-worldcoords))))
	 ((equal arm :rarm)
	  (setq middle-target-coords (list (send *pr2* :larm :end-coords :copy-worldcoords) middle-grasp-coords)))
	 )
	(push middle-target-coords middle-target-coords-list)
	(push (* (+ i 1) 1000) middle-target-time-list)
	(when last-flag
	  (return))

	;; (when (< distance 0)
	;;     (return))
	)

      (print middle-target-time-list)


      (setq pre-grasp-av
	    (send *pr2* :inverse-kinematics pre-target-coords
		  :move-target (send *pr2* :arms :end-coords)
		  ;;:debug-view t
		  :look-at-target t
		  ;;:rotation-axis rotation-axis
		  :use-torso t
		  :collision-avoidance-link-pair
		  (mapcan #'(lambda (j) (mapcar #'(lambda (i) (list i j))
						(send *pr2* :link-list (send *pr2* :r_wrist_roll_link_lk) (send *pr2* :r_forearm_roll_link_lk))))
			  (send *pr2* :link-list (send *pr2* :l_wrist_roll_link_lk) (send *pr2* :l_forearm_roll_link_lk)))
		  :avoid-collision-distance 100
		  ))

      (unless pre-grasp-av
	(send grasp-part :change-grasp-coords)
	)


      (setq grasp-av
	    (send *pr2* :inverse-kinematics target-coords
		  :move-target (send *pr2* :arms :end-coords)
		  :look-at-target t
		  :use-torso t
		  ;;:rotation-axis rotation-axis
		  :collision-avoidance-link-pair
		  (mapcan #'(lambda (j) (mapcar #'(lambda (i) (list i j))
						(send *pr2* :link-list (send *pr2* :r_wrist_roll_link_lk) (send *pr2* :r_forearm_roll_link_lk))))
			  (send *pr2* :link-list (send *pr2* :l_wrist_roll_link_lk) (send *pr2* :l_forearm_roll_link_lk)))
		  :avoid-collision-distance 100
		  ))
      (unless grasp-av
	(send grasp-part :change-grasp-coords)
	)
      (when (and pre-grasp-av grasp-av)
	(return)
	)
      )

    (cond
     ((and pre-grasp-av grasp-av (not (equal (send grasp-part :rotation-axis) t)))
      (setq link-coords (send (elt (send model :links) (send grasp-part :link-index)) :copy-worldcoords))
      ;;(send grasp-part :set-grasp-coords (send link-coords :transformation (send *pr2* arm :end-coords)))
      (send grasp-part :set-rotation-axis t)
      (print grasp-part)
      (grasp-object arm :record nil)
      )
     (ik-check
      (cond
       ((equal arm :larm)
	(setq *larm-grasp* t)
	)
       ((equal arm :rarm)
	(setq *rarm-grasp* t)
	))
      (and pre-grasp-av grasp-av)
      
      )
     ((and pre-grasp-av grasp-av)
      (print middle-target-coords-list)
      (setq middle-av-list nil)
      (dolist (target-coords middle-target-coords-list)
	(push 
	 (send *pr2* :inverse-kinematics target-coords
	       :move-target (send *pr2* :arms :end-coords)
	       :look-at-target t
	       :use-torso t
	       :collision-avoidance-link-pair
	       (mapcan #'(lambda (j) (mapcar #'(lambda (i) (list i j))
					     (send *pr2* :link-list (send *pr2* :r_wrist_roll_link_lk) (send *pr2* :r_forearm_roll_link_lk))))
		       (send *pr2* :link-list (send *pr2* :l_wrist_roll_link_lk) (send *pr2* :l_forearm_roll_link_lk)))
	       :avoid-collision-distance 100
	       ) 
	 middle-av-list))
      ;;(setq middle-av-list (reverse middle-av-list))
      (setq middle-target-time-list (reverse middle-target-time-list))
            

      (send *ri* :stop-grasp arm :wait t)
      (send *ri* :angle-vector pre-grasp-av 5000)
      (send *ri* :wait-interpolation)
      (send *ri* :angle-vector-sequence middle-av-list middle-target-time-list)
      ;;(send *ri* :angle-vector grasp-av 2000)
      (send *ri* :wait-interpolation)
      (send *ri* :start-grasp arm)

      (when (equal *recording* "robot")
	(print "recording robot")
	(push (list 'send-stop-grasp arm) *recorded-movement*)
	(push (list 'send-angle-vector pre-grasp-av 5000) *recorded-movement*)
	(push (list 'send-angle-vector-sequence middle-av-list middle-target-time-list) *recorded-movement*)
	
	;; (dotimes (i (length middle-av-list))
	;;   (push (list 'send-angle-vector (elt middle-av-list i) (elt middle-target-time-list i)) *recorded-movement*)
	;;   )
	(push (list 'send-start-grasp arm) *recorded-movement*)
	)


      (cond
       ((equal arm :larm)
	(setq *larm-grasp* t)
	)
       ((equal arm :rarm)
	(setq *rarm-grasp* t)
	))
      )
     (t
      (send *pr2* :angle-vector prev-av)
      )
     )
    )
  )
(defun release-object (arm &key (ik-check nil))
  ;; test code for object-relation-fc.l
  (setq mm-msg (instance jsk_interactive_marker::MakeModel :init))

  (cond
   ((equal arm :larm)
    (send mm-msg :type "release_larm")
    )
   ((equal arm :rarm)
    (send mm-msg :type "release_rarm")
    )
   )
  (ros::publish *publish-make-model-topic* mm-msg)
  (return-from release-object)

  ;; test code for object-relation-fc.l

  (when (equal *recording* "model")
    (print "recording")
    (push (list 'release-object arm) *recorded-movement*)
    )

  (print "RELEASE")

  (setq release-coords (send *pr2* arm :end-coords :copy-worldcoords))
  (setq release-coords (send release-coords :locate (float-vector -100 0 0)))

  (when (send grasp-part :model)
    (objects (list *pr2* grasp-coords model))
    (setq release-av
	  (send *pr2* :inverse-kinematics release-coords
		:move-target (send *pr2* arm :end-coords)
		:look-at-target t
		:use-torso t))
    (cond
     (ik-check
      (cond
       ((equal arm :larm)
	(setq *larm-grasp* nil)
	)
       ((equal arm :rarm)
	(setq *rarm-grasp* nil)
	))
      release-av
      )
     (release-av
      (send *ri* :stop-grasp arm :wait t)
      (send *ri* :angle-vector release-av 2000)
      (send *ri* :wait-interpolation)

      (when (equal *recording* "robot")
	(print "recording robot")
	(push (list 'send-stop-grasp arm) *recorded-movement*)
	(push (list 'send-angle-vector release-av 2000) *recorded-movement*)
	)

      (cond
       ((equal arm :larm)
	(setq *larm-grasp* nil)
	)
       ((equal arm :rarm)
	(setq *rarm-grasp* nil)
	))
      )
     )
    )
  )

(defun object-move-callback (msg)
  (print-ros-msg msg)
  (setq hoge msg)
  (unless (send *target-object* :model)
    (return-from object-move-callback))
  (setq model (send *target-object* :model))
  (setq start-model (copy-object (send *target-object* :model)))
  (setq goal-model (copy-object (send *target-object* :model)))

  (send goal-model :move-to (pose-stamped->coords (send msg :pose_goal) *robot-frame-id*) :world)
  ;;update joint state
  (apply-joint_state (send msg :joint_state_goal) goal-model)


  ;;test code for object-relation-fc.l
  (setq mm-msg (instance jsk_interactive_marker::MakeModel :init))
  (send mm-msg :pose (ros::coords->tf-pose-stamped (send goal-model :worldcoords) "update"))
  (send mm-msg :type "move")
  (send mm-msg :joint_state (send msg :joint_state_goal))
  (ros::publish *publish-make-model-topic* mm-msg)
  (return-from object-move-callback)
  ;;test code for object-relation-fc.l


  (setq js-diff (v- (send goal-model :angle-vector) (send model :angle-vector)))
  (setq cds-diff (send (send model :worldcoords) :transformation (send goal-model :worldcoords)))
  
  (object-move cds-diff js-diff :record *recording*)
  )


(defun recorded-movement (record)
  (setq suc-flag nil)
  ;;(setq *recording* nil)

  (setq *start-larm-grasp* *larm-grasp*)
  (setq *start-rarm-grasp* *rarm-grasp*)
  (setq model  (send *target-object* :model))
  (setq *start-model* (copy-object (send *target-object* :model)))

  (while (not suc-flag)
    (setq suc-flag t)
    (setq *larm-grasp* *start-larm-grasp*)
    (setq *rarm-grasp* *start-rarm-grasp*)

    (send model :move-to (send *start-model* :worldcoords) :world)
    (send model :angle-vector (send *start-model* :angle-vector))

    
    (dolist (move record)
      (print (cdr move))
      (setq suc (apply (symbol-function (car move)) (append (cdr move) (list :ik-check t))))

      (print suc)
      (unless suc
	(send (send *target-object* :grasp-part :rarm) :change-grasp-coords)
	(send (send *target-object* :grasp-part :larm) :change-grasp-coords)
	(setq suc-flag nil)
	(return)
	)
      )
    (print "suc-flag")
    (print suc-flag)
    )

  (setq *larm-grasp* *start-larm-grasp*)
  (setq *rarm-grasp* *start-rarm-grasp*)

  (send model :move-to (send *start-model* :worldcoords) :world)
  (send model :angle-vector (send *start-model* :angle-vector))

  (dolist (move record)
    (setq suc (apply (symbol-function (car move)) (append (cdr move)))))
  )

(setq *recorded-movement* nil)
(setq *recorded-movement-list* nil)

;;debug
(setq *recording* nil)
;;(setq *recording* "robot")

;;(load "package://jsk_interactive_marker/euslisp/target-object.l")
;;(load "package://jsk_interactive_marker/euslisp/target-object-grasp.l")
;;debug

(defun record-callback
  (msg)
  (cond
   ;;record
   ((equal (send  msg :command) 0)
    (print "recording")
    (setq *recording* "robot")
    )
   ;;stop record
   ((equal (send  msg :command) 1)
    (push (list (send msg :target) (reverse *recorded-movement*)) *recorded-movement-list* )
    (setq *recorded-movement* nil)
    (setq *recording* nil)
    )
   ;;play record
   ((equal (send  msg :command) 2)
    (setq play-movement (cadr (assoc (send msg :target) *recorded-movement-list* :test #'string-equal)))
    (recorded-movement play-movement)
    
    )
   )
  )

(defun send-angle-vector (av time &key (ik-check nil))
  (cond
   (ik-check
    t
    )
   (t
    (send *ri* :angle-vector av time)
    (send *ri* :wait-interpolation)
    )
   )
  )


(defun send-angle-vector-sequence (avs times &key (ik-check nil))
  (cond
   (ik-check
    t
    )
   (t
    (send *ri* :angle-vector-sequence avs times)
    (send *ri* :wait-interpolation)
    )
   )
  )

(defun send-start-grasp (arm &key (ik-check nil))
  (cond 
   (ik-check
    t
    )
   (t
    (send *ri* :start-grasp arm)
    )
  )
  )

(defun send-stop-grasp (arm &key (ik-check nil))
  (cond
   (ik-check
    t
    )
   (t
    (send *ri* :stop-grasp arm :wait t)
    )
  ))


;;move button
(defun object-move (cds-diff js-diff &key (ik-check nil) (record nil))
  (when (equal *recording* "model")
    (print "recording")
    (push (list 'object-move cds-diff js-diff) *recorded-movement*)
    )

  (print *target-object*)
  (print (send *target-object* :model))

  (unless (send *target-object* :model)
    (return-from object-move))
  (setq model (send *target-object* :model))
  (setq start-model (copy-object (send *target-object* :model)))
  (setq goal-model (copy-object (send *target-object* :model)))
  (print cds-diff)
  (print js-diff)
  (send goal-model :transform cds-diff)
  (send goal-model :angle-vector (v+ (send goal-model :angle-vector) js-diff))
  ;;(send goal-model :move-to (pose-stamped->coords (send msg :pose_goal) *robot-frame-id*) :world)

  ;;update joint state
  ;;(apply-joint_state (send msg :joint_state_goal) goal-model)

  (objects (list *pr2* start-model goal-model))

  (setq interpolation-num 10)
  (let* ((start-coords (send start-model :copy-worldcoords))
	 (goal-coords (send goal-model :copy-worldcoords))
	 avs)
    (dotimes (i interpolation-num)
      (setq interpolation-coords (midcoords (/ (+ i 1.0) interpolation-num) start-coords goal-coords))
      (setq interpolation-model (copy-object (send *target-object* :model)))
      (send interpolation-model :move-to interpolation-coords :world)

      (send interpolation-model :angle-vector
	    (midpoint (/ (+ i 1.0) interpolation-num)
		      (send start-model :angle-vector) (send goal-model :angle-vector)))

      (x::window-main-one)

      (unless (send grasp-part :link-index)
	(return-from object-move))

      (setq target-coords
	    (mapcar
	     #'(lambda (grasp-part arm grasp?)
		 (if grasp?
		     (progn
		       (setq grasp-coords (send (elt (send interpolation-model :links) (send grasp-part :link-index)) :copy-worldcoords))
		       (send grasp-coords :transform (send grasp-part :grasp-coords))
		       grasp-coords)
		   (send *pr2* arm :end-coords :copy-worldcoords)
		   ))
	     (list (send *target-object* :grasp-larm-part) (send *target-object* :grasp-rarm-part)) (list :larm :rarm) (list *larm-grasp* *rarm-grasp*))
	    )
      (objects (append  (list *pr2*) target-coords))
      (print target-coords)

      (setq ik-suc
	    (send *pr2* :inverse-kinematics target-coords
		  :debug-view nil
		  :look-at-target t
		  :move-target (send *pr2* :arms :end-coords)
		  :collision-avoidance-link-pair
		  (mapcan #'(lambda (j) (mapcar #'(lambda (i) (list i j))
						(send *pr2* :link-list (send *pr2* :r_wrist_roll_link_lk) (send *pr2* :r_forearm_roll_link_lk))))
			  (send *pr2* :link-list (send *pr2* :l_wrist_roll_link_lk) (send *pr2* :l_forearm_roll_link_lk)))
		  :avoid-collision-distance 100

		  :use-torso t))
      (cond
       (ik-suc
	(push (send *pr2* :angle-vector) avs)
	(print avs)
	;;(send *ri* :angle-vector (send *pr2* :angle-vector))
	;;(send *ri* :wait-interpolation)
	(send model :move-to (send interpolation-model :copy-worldcoords) :world)
	(send model :angle-vector (send interpolation-model :angle-vector))
	)
       (t
	(return)
	;;(return-from object-move-callback)
	))
      )

    (cond
     (ik-check
      (print "aaaaaaaaaaaaa")
      (print (length avs))
      (print interpolation-num)
      (equal (length avs) interpolation-num)
      )
     (t
      (when avs
	(print avs)
	(setq avs (reverse avs))
	(setq a avs)
	(send *ri* :angle-vector-sequence avs 1000)
	(send *ri* :wait-interpolation)

	(when (equal *recording* "robot")
	  (print "recording angle vector")
	  (push (list 'send-angle-vector-sequence avs 1000) *recorded-movement*)
	  ;;(dolist (av avs)
	  ;;(push (list 'send-angle-vector av 1000) *recorded-movement*)
	  ;;)
	  )

	)
      )
     )
    )
  )

(setq *larm-grasp* nil)
(setq *rarm-grasp* nil)

(setq *robot-frame-id* "/base_footprint")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(objects (list *pr2*))

(defun edge->marker-msg (e &key (radius 10) (frame-id "/edge") (color (float-vector 1 0 0)) (id 0) (alpha 1))
  (setq c (make-cylinder radius (send e :length)))
  (setq cylinder-coords (make-coords :pos (car (send e :vertices)) :rot (z-vec->rotation (send e :direction))))
  (send c :move-to cylinder-coords)
  (cylinder->marker-msg c (instance std_msgs::Header :init :frame_id frame-id)
			:color color
			:alpha alpha
			:id id)
  )

(defun publish-line (l &key (name "line") (frame-id "eng2/7f/73B2") (color (float-vecor 1 0 0)))
  (setq c (make-cylinder 2 (send l :length)))
  (setq cylinder-coords (make-coords :pos (car (send l :vertices)) :rot (z-vec->rotation (send l :direction))))
  (send c :move-to cylinder-coords)
  ;;(setq marker (cylinder->marker-msg c (send im :header) :alpha 0.2))
  (publish-interactive-marker
   (cylinder->marker-msg c (instance std_msgs::Header :init :frame_id frame-id)
			 :color color)
   :name name :frame-id frame-id)
  )

(defun publish-point (point &key (name "line") (frame-id "eng2/7f/73B2") (color (float-vecor 1 1 0)) (alpha 0.6))
  (setq p (make-sphere 3))
  (setq point-coords (make-coords :pos point))
  (send p :move-to point-coords)
  (publish-interactive-marker
   (sphere->marker-msg p (instance std_msgs::Header :init :frame_id frame-id)
			 :color color :alpha alpha)
   :name name :frame-id frame-id)
  )

;;override to add offset

(defun faces->marker-msg (faces header &key
				(offset (float-vector 0 0 0))
                                ((:color col) (float-vector 1 0 0))
                                ((:id idx) 0) ns lifetime)
  ;;(send-all faces :worldcoords) ;; make sure to update vertices
  (let* ((tri-faces
          (mapcan #'(lambda (f) (geo::face-to-triangle f))
                  faces))
         (tri-vertices
          (mapcan #'(lambda (f) (cdr (send f :vertices)))
                  tri-faces))
         (points
          (mapcar #'(lambda (p)
                      (ros::pos->tf-point (v+ p offset)))
                  tri-vertices))
         (msg (instance visualization_msgs::Marker :init
                        :type visualization_msgs::Marker::*TRIANGLE_LIST*
                        :header header
                        :id idx)))
    (send msg :pose (ros::coords->tf-pose (make-coords)))
    (send msg :scale (ros::pos->tf-translation
                      (float-vector 1000 1000 1000)))
    (send msg :color (vector->rgba col 1.0))

    (send msg :points points)
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))


(defun publish-face (f &key (name "line") (frame-id "eng2/7f/73B2") (color (float-vecor 1 0 0)))
  (publish-interactive-marker
   (faces->marker-msg (list f) (instance std_msgs::Header :init :frame_id frame-id)
		      :offset (scale 1 (send f :normal))
		      :color color)
   :name name :frame-id frame-id)
  )

(defun publish-interactive-marker (markers &key (name "line") (frame-id "eng2/7f/73B2"))
  (setq im (instance visualization_msgs::InteractiveMarker :init))

  (send im :name name)
  (send im :header :frame_id frame-id)

  (send im :pose (ros::coords->tf-pose (make-coords)))

  ;;(setq markers nil)
  (when (atom markers)
    (setq markers (list markers)))

  (setq controls nil)
  (setq control (instance visualization_msgs::InteractiveMarkerControl :init))
  (send control :markers markers)
  (send control :always_visible t)

  (send control :interaction_mode visualization_msgs::InteractiveMarkerControl::*BUTTON*)
  (push control controls)

  (send im :controls controls)
  (ros::publish *add-marker-topic* im)
  )

(defun publish-grasp-interactive-marker (cds &key (name "line") (frame-id "eng2/7f/73B2"))
  (setq im (instance visualization_msgs::InteractiveMarker :init))

  (send im :name name)
  (send im :header :frame_id frame-id)
  (send im :scale 0.2)

  (send im :pose (ros::coords->tf-pose cds))

  (setq controls nil)
  (setq control (instance visualization_msgs::InteractiveMarkerControl :init))
  (send control :always_visible t)
  (send control :interaction_mode visualization_msgs::InteractiveMarkerControl::*MOVE_AXIS*)
  ;;(send control :orientation (ros::rot->tf-quaternion (send (make-coords :angle pi/2 :axis :y) :worldrot)))
  (send control :orientation (ros::rot->tf-quaternion (send (make-coords) :worldrot)))
  (push control controls)

  (send im :controls controls)
  (ros::publish *add-marker-topic* im)
  )

(defun pose-stamped->coords
  (pose-stamped frame-id)
  (let ((cds (ros::tf-pose-stamped->coords pose-stamped)))
    (while t
      (setq frame-id->ps-frame-id
	    (send *tfl* :lookup-transform
		  frame-id (send pose-stamped :header :frame_id) (ros::time 0)))
      (cond
       (frame-id->ps-frame-id
	(send cds :transform frame-id->ps-frame-id :world)
	(send cds :name frame-id)
	(return)
	)
       (t
	(ros::ros-info "~A -> ~A cannot transform yet" frame-id (send pose-stamped :header :frame_id))
	(unix:sleep 1)
	(ros::sleep)
	)
       )
      )
    cds
    )
  )



(setq *tfl* (instance ros::transform-listener :init))
(setq *add-marker-topic* "/interactive_marker_manager/add_marker")
(setq *grasp-marker-topic* "/interactive_marker_manager/grasp_marker")
(setq *clear-marker-topic* "/interactive_marker_manager/clear_marker")
(setq *marker-feedback-topic* "/interactive_marker_manager/feedback")
(setq *object-relation-topic* "/jsk_model_marker_interface/relation")
(setq *object-move-topic* "/jsk_model_marker_interface/object_relation_move_model")
(setq *model-feedback-topic* "/jsk_model_marker_interface/feedback")
(setq *record-topic* "/record_command")

(setq *target-object* (instance object-relation :init))
;;(load "package://jsk_interactive_marker/euslisp/target-object-grasp.l")
(setq *reference-object* (instance object-relation :init))


(ros::subscribe *marker-feedback-topic*
		visualization_msgs::InteractiveMarkerFeedback #'interactive-marker-feedback)

(ros::subscribe *object-relation-topic*
		jsk_interactive_marker::ObjectRelation #'object-relation-callback)

(ros::subscribe *object-move-topic*
		jsk_interactive_marker::MoveModel #'object-move-callback)

(ros::subscribe *model-feedback-topic*
		visualization_msgs::InteractiveMarkerFeedback #'model-interactive-marker-feedback)

(ros::subscribe *record-topic*
		jsk_rviz_plugins::RecordCommand #'record-callback)


(ros::advertise *add-marker-topic*
		visualization_msgs::InteractiveMarker)

(ros::advertise *grasp-marker-topic*
		visualization_msgs::Marker)

(ros::advertise *clear-marker-topic*
		std_msgs::Empty)


(clear-all-markers)
;; (while (ros::ok)
;;   (x::window-main-one)
;;   (ros::spin-once)
;;   )
;; (ros::spin)


;; #f(50.0476 57.9131 72.8416 68.8176 -120.775 18.746 -27.978 181.044 -14.0341 34.9127 -90.4354 -76.1615 -52.4834 -24.1792 190.927 0.0 0.0)


#|
(defun test-code
  ()
  (send *pr2*
	:inverse-kinematics (list (make-coords :pos #f(500 0 500)) (make-coords :pos #f(500 0 500)))
	:debug-view t
	:move-target (send *pr2* :arms :end-coords)
	:link-list (list
		    (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
		    (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent)))
	;; :collision-links
	;; (list (list (send (send *pr2* :larm :end-coords :parent) :parent)
	;; 		  (send (send *pr2* :rarm :end-coords :parent) :parent)))

	:collision-avoidance-link-pair
	(send *pr2* :collision-avoidance-link-pair-from-link-list
	      (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
	      :collision-avoidance-links (list (send (send *pr2* :rarm :end-coords) :parent)
					       (send (send (send *pr2* :rarm :end-coords) :parent) :parent))))


  (send *pr2*
	:inverse-kinematics (list (make-coords :pos #f(500 0 750) :angle 0 :axis :z) (make-coords :pos #f(500 0 700) :angle pi/2 :axis :z))
	:debug-view t
	:move-target (send *pr2* :arms :end-coords)
	:collision-avoidance-link-pair
	(mapcan #'(lambda (j) (mapcar #'(lambda (i) (list i j))
				      (send *pr2* :link-list (send *pr2* :r_wrist_roll_link_lk) (send *pr2* :r_forearm_roll_link_lk))))
		(send *pr2* :link-list (send *pr2* :l_wrist_roll_link_lk) (send *pr2* :l_forearm_roll_link_lk)))
	:avoid-collision-distance 100
	)
  ;; :avoid-collision-null-gain 3
  ;; :avoid-collision-joint-gain 3)

)

|#