#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive_marker")
(ros::roseus "object_relation")
(require "models/room73b2-scene.l")

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
  (marker-id frame-id model index link-index color distance))

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
   )
  (:set-marker-id
   (mid)
   (setq marker-id mid))

  (:link-index
   ()
   link-index
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
     (publish-face model :frame-id frame-id :name (send model :name) :color color)
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
	(edge->marker-msg model :color color :alpha 0.6 :id marker-id :frame-id frame-id)
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

  (:grasp-coords
   ()
   (when model
     (cond
      ;;face
      ((derivedp model face)

       (setq max-len 0)
       (dolist (v (send model :edges))
	 (when (> (send v :length) max-len)
	   (setq max-len (send v :length))
	   (setq z-vec (send v :direction))))
       (make-coords :pos (cadr (send model :centroid))
		    :rot (xz-vec->rotation (v- (send model :normal)) z-vec)
		    ))
      ;;edge
      ((derivedp model edge)
       (setq pface-vec (float-vector 0 0 0))
       (setq nface-vec (float-vector 0 0 0))

       (when (car (send model :faces))
	 (setq pface-vec (send (car (send model :faces)) :normal)))

       (when (cadr (send model :faces))
	 (setq nface-vec (send (cadr (send model :faces)) :normal)))

       (setq x-vec (v- (midpoint 0.5 pface-vec nface-vec)))
       
       (make-coords :pos  (midpoint 0.5 (car (send model :vertices)) (cadr (send model :vertices)))
		    :rot (xz-vec->rotation 
			  x-vec
			  (send model :direction))
		    )
       )
      ;;line
      ((derivedp model line)
       (make-coords :pos  (midpoint 0.5 (car (send model :vertices)) (cadr (send model :vertices)))
		    :rot (z-vec->rotation (send model :direction))
		    )
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
   (setq grasp-distance 100)
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

  (:grasp-larm-part
   ()
   grasp-larm-part
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
   
   (setq index 0)
   (dolist (b (send-all (send model :bodies) :box))
     (setq center (send b :center))
     (setq dia-scale 1.3)
     (setq dia (scale dia-scale (scale 0.5 (send b :diagonal))))

     (setq x-vec (float-vector (elt dia 0) 0 0))
     (setq y-vec (float-vector 0 (elt dia 1) 0))
     (setq z-vec (float-vector 0 0 (elt dia 2)))

     (dolist (vec (list x-vec y-vec z-vec))
       (publish-line (make-line (v- center vec) (v+ center vec))
		     :frame-id selection-frame-id :name  (send self :get-marker-name prefix "line" index) :color selection-color)
       (setq index (+ index 1))
       )
     )

   ;;edge
   (setq edge-num 0)
   (dolist (e (flatten (send-all (send selection-model :bodies) :edges)))
     ;;(publish-line e :frame-id (send selection-model :name) :name (format nil "~A/edge/~A" relation edge-num))
     (publish-line e :frame-id selection-frame-id :name  (send self :get-marker-name prefix "edge" edge-num) :color selection-color)
     (setq edge-num (+ edge-num 1))
     ;;))
     )

   (setq face-num 0)
   (dolist (f (send selection-model :faces))
     (publish-face f :frame-id selection-frame-id :name (send self :get-marker-name prefix "face" face-num) :color selection-color)
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
	   (instance object-part :init selection-frame-id m index selection-link-index (float-vector 0.4 0.7 0.7)  100 :id 0))
     )
    ((equal prefix "grasp_point_rarm")
     (setq grasp-rarm-part
	   (instance object-part :init selection-frame-id m index selection-link-index (float-vector 0.7 0.4 0.4)  100 :id 1))
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
   (setq cds (send (elt (send model :links) (send selected-part :link-index)) :copy-worldcoords))
   (send cds :transform (send selected-part :coords))
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
   (cds)
   (send cds :locate #f(0 0 0))
   (setq req (instance jsk_interactive_marker::SetInteractiveMarkerControlsRequest :init))

   (setq imcs nil)
   
   (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
   (send imc :always_visible t)

   ;; x-translation
   (setq control-cds (send cds :copy-worldcoords))
   (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
   (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*MOVE_AXIS*)
   (push imc imcs)

   (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
   (send imc :always_visible t)
   ;; y-translation
   (setq control-cds (send (send cds :copy-worldcoords) :rotate pi/2 :z))
   ;;(setq cds (make-coords :angle pi/2 :axis :z))
   (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
   (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*MOVE_AXIS*)
   (push imc imcs)

   (setq imc (instance visualization_msgs::InteractiveMarkerControl :init))
   (send imc :always_visible t)
   ;; z-rotation
   ;;(setq cds (make-coords :angle pi/2 :axis :y))
   (setq control-cds (send (send cds :copy-worldcoords) :rotate pi/2 :y))
   (send imc :orientation (ros::rot->tf-quaternion (send control-cds :worldrot)))
   (send imc :interaction_mode visualization_msgs::InteractiveMarkerControl::*ROTATE_AXIS*)
   (push imc imcs)
   (print-ros-msg imcs)

   (ros::service-call (format nil "/jsk_model_marker_interface/~A/set_root_controls" (send model :name) ) (instance jsk_interactive_marker::SetInteractiveMarkerControlsRequest :init :controls imcs))

   (ros::service-call (format nil "/jsk_model_marker_interface/~A/show_control" (send model :name)) (instance std_srvs::EmptyRequest :init))
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

    (send *target-object* :set-controls target-coords)
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
  (cond
   ((functionp (read-from-string (send msg :model)))
      (setq model (funcall (read-from-string (send msg :model))))
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

  (cond
   ((equal (send msg :relation) "target")
    ;;target and reference are different
    (when (equal (send msg :name) (send *reference-object* :name))
      (setq *reference-object* (instance object-relation :init))
      )
    (unless (equal (send msg :name) (send *target-object* :name))
      (setq *target-object* (instance object-relation :init model frame-id (send msg :relation)))
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
      )
    (send *target-object* :set-selection (send msg :link) selection-m (send msg :relation) selection-li)
    )
   ((equal (send msg :relation) "grasp_larm")
    (print "GRASP LARM!!")
    (setq grasp-part (send *target-object* :grasp-larm-part))
    (setq model (copy-object (send *target-object* :model)))
    (send model :move-to
	  (pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) (send *target-object* :frame-id)) *robot-frame-id*))

    (setq grasp-coords (send (elt (send model :links) (send grasp-part :link-index)) :copy-worldcoords))
    (send grasp-coords :transform (send grasp-part :grasp-coords))

    ;;(setq grasp-coords (make-cascoords :coords (send grasp-part :grasp-coords)))
    ;;(send model :assoc grasp-coords)

    (when (send grasp-part :model)
      (objects (list *pr2* grasp-coords))
      (send *pr2* :inverse-kinematics grasp-coords
	    :move-target (send *pr2* :larm :end-coords)
	    :use-torso t)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
      (send *ri* :start-grasp :larm)
      
      )
    )
   )
  (publish-all-markers)
  )

(setq *robot-frame-id* "/base_footprint")

(load "package://pr2eus/pr2-interface.l")
;; (pr2-init)
;; (objects (list *pr2*))

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
  (setq c (make-cylinder 10 (send l :length)))
  (setq cylinder-coords (make-coords :pos (car (send l :vertices)) :rot (z-vec->rotation (send l :direction))))
  (send c :move-to cylinder-coords)
  ;;(setq marker (cylinder->marker-msg c (send im :header) :alpha 0.2))
  (publish-interactive-marker
   (cylinder->marker-msg c (instance std_msgs::Header :init :frame_id frame-id)
		      :color color)
   :name name :frame-id frame-id)



  ;; (publish-interactive-marker
  ;;  (faces->marker-msg (list f) (instance std_msgs::Header :init :frame_id frame-id)
  ;; 		      :offset (scale 1 (send f :normal))
  ;; 		      :color color)
  ;;  :name name :frame-id frame-id)


  ;; (setq im (instance visualization_msgs::InteractiveMarker :init))
  ;; (send im :name name)
  ;; (send im :header :frame_id frame-id)
  ;; (send im :pose (ros::coords->tf-pose (make-coords)))

  ;; (setq markers nil)

  ;; (setq c (make-cylinder 10 (send l :length)))
  ;; (setq cylinder-coords (make-coords :pos (car (send l :vertices)) :rot (z-vec->rotation (send l :direction))))
  ;; (send c :move-to cylinder-coords)
  ;; (setq marker (cylinder->marker-msg c (send im :header) :alpha 0.2))
  ;; (push marker markers)

  ;; (setq controls nil)
  ;; (setq control (instance visualization_msgs::InteractiveMarkerControl :init))
  ;; (send control :markers markers)

  ;; (send control :interaction_mode visualization_msgs::InteractiveMarkerControl::*BUTTON*)
  ;; (push control controls)

  ;; (send im :controls controls)
  ;; (ros::publish *add-marker-topic* im)
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
  (send im :scale 0.3)

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


(setq *target-object* (instance object-relation :init))
(setq *reference-object* (instance object-relation :init))


(ros::subscribe *marker-feedback-topic*
		visualization_msgs::InteractiveMarkerFeedback #'interactive-marker-feedback)

(ros::subscribe *object-relation-topic*
		jsk_interactive_marker::ObjectRelation #'object-relation-callback)

(ros::advertise *add-marker-topic*
		visualization_msgs::InteractiveMarker)

(ros::advertise *grasp-marker-topic*
		visualization_msgs::Marker)

(ros::advertise *clear-marker-topic*
		std_msgs::Empty)


(clear-all-markers)
(while (ros::ok)
  (x::window-main-one)
  (ros::spin-once)
  )
(ros::spin)