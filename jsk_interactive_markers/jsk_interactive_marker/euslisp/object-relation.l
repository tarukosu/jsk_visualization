#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive_marker")
(ros::roseus "object_relation")
(require "models/room73b2-scene.l")

(setq *model* (room73b2-bottle))

(defclass object-relation
  :slots
  (model frame-id relation
	 selected-model
	 selected-index
	 ))

(defmethod object-relation
  (:init
   (&optional m f r)
   (setq model m)
   (setq frame-id f)
   (setq relation r)

   (ros::advertise (format nil "/jsk_model_marker_interface/~A/set_pose" (send model :name))
		   geometry_msgs::PoseStamped)

   )
  (:frame-id
   ()
   frame-id
   )
  (:publish-marker
   ()
   (if selected-model
       (send self :publish-selected-marker)
     (send self :publish-object-marker)
   ))

  (:publish-object-marker
   ()
   (unless model
     (return-from :publish-object-marker))
   (setq edge-num 0)
   (dolist (b (flatten (send-all (send model :bodies) :edges)))
     ;;(publish-line e :frame-id (send model :name) :name (format nil "~A/edge/~A" relation edge-num))
     (setq edge-num (+ edge-num 1))
     ;;))
     )

   (setq face-num 0)
   (dolist (f (send model :faces))
     (publish-face f :frame-id frame-id :name (send self :get-marker-name face-num))
     (setq face-num (+ face-num 1))
     ))

  (:publish-selected-marker
   ()
   (publish-face selected-model :frame-id frame-id :name (send selected-model :name))
   )

  (:get-marker-name
   (index)
   (format nil "~A/face/~A" relation index)
   )
  
  (:selection
   (index)
   (unless model
     (return-from :selection))
   (setq selected-index index)
   (setq selected-model (elt (send model :faces) selected-index))
   (send selected-model :name (send self :get-marker-name selected-index))
   

   )
  (:selected-model
   ()
   selected-model
   )

  (:selected-coords
   ()
   (when selected-model
     (make-coords :pos (cadr (send selected-model :centroid))
		  :rot (z-vec->rotation (send selected-model :normal)))
     )
  )

  (:move-to
   (cds cds-frame-id)
   (setq ps (instance geometry_msgs::PoseStamped :init))
   (send ps :header :frame_id cds-frame-id)
   (send ps :pose (ros::coords->tf-pose cds))

   (print-ros-msg ps)
   (ros::publish (format nil "/jsk_model_marker_interface/~A/set_pose" (send model :name)) ps)
   )
  )

;;utils

(defun split-string (str)
  (setq split-char "/")
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
;;utils


(defun interactive-marker-feedback (msg)
  (clear-all-markers)
  (print-ros-msg msg)
  (setq marker-config (split-string (send msg :marker_name)))

  (cond
   ((equal (elt marker-config 0) "target")
    (send *target-object* :selection (read-from-string (elt marker-config 2)))
    )
   ((equal (elt marker-config 0) "reference")
    (send *reference-object* :selection (read-from-string (elt marker-config 2)))
    )
   )
  (publish-all-markers)
  ;;(print *target-object-setting*)

  (when (and  (send *target-object* :selected-model) (send *reference-object* :selected-model))
    (setq target-coords (send *target-object* :selected-coords))
    (setq reference-coords (send *reference-object* :selected-coords))
    (send target-coords :rotate pi :x)

    (send *target-object* :move-to (send target-coords :transformation reference-coords :world) (send *reference-object* :frame-id))
    (send *target-object* :move-to (send target-coords :transformation reference-coords :world) (send *reference-object* :frame-id))
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
  (cond 
   ((functionp (read-from-string (send msg :model)))
      (setq model (funcall (read-from-string (send msg :model))))
      (send model :name (send msg :name))
      )
   (t
    (return-from object-relation-callback nil))
   )

  (ros::publish *clear-marker-topic* (instance std_msgs::Empty :init))

  (cond
   ((equal (send msg :relation) "target")
    (setq *target-object* (instance object-relation :init model (send msg :link) (send msg :relation)))
    )
   ((equal (send msg :relation) "reference")
    (setq *reference-object* (instance object-relation :init model (send msg :link) (send msg :relation)))
    )
   )
  (publish-all-markers)
)


(defun publish-line (l &key (name "line") (frame-id "eng2/7f/73B2"))
  (setq im (instance visualization_msgs::InteractiveMarker :init))
  ;;(send im :name (send l :name))
  (send im :name name)
  ;;(send im :header :frame_id "map")
  (send im :header :frame_id frame-id)
  (send im :pose (ros::coords->tf-pose (make-coords)))


  (setq markers nil)

  (setq c (make-cylinder 10 (send l :length)))
  (setq cylinder-coords (make-coords :pos (car (send l :vertices)) :rot (z-vec->rotation (send l :direction))))
  (send c :move-to cylinder-coords)
  (setq marker (cylinder->marker-msg c (send im :header) :alpha 0.2))
  (push marker markers)

  (setq controls nil)
  (setq control (instance visualization_msgs::InteractiveMarkerControl :init))
  (send control :markers markers)

  (send control :interaction_mode visualization_msgs::InteractiveMarkerControl::*BUTTON*)
  (push control controls)

  (send im :controls controls)
  (ros::publish *add-marker-topic* im)
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

(defun publish-face (f &key (name "line") (frame-id "eng2/7f/73B2"))
  (setq im (instance visualization_msgs::InteractiveMarker :init))

  (send im :name name)
  (send im :header :frame_id frame-id)

  (send im :pose (ros::coords->tf-pose (make-coords)))

  (setq markers nil)

  (setq marker (faces->marker-msg (list f) (send im :header) :offset (scale 1 (send f :normal))))
  (push marker markers)

  (setq controls nil)
  (setq control (instance visualization_msgs::InteractiveMarkerControl :init))
  (send control :markers markers)

  (send control :interaction_mode visualization_msgs::InteractiveMarkerControl::*BUTTON*)
  (push control controls)

  (send im :controls controls)
  (ros::publish *add-marker-topic* im)
)

(setq *add-marker-topic* "/interactive_marker_manager/add_marker")
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

(ros::advertise *clear-marker-topic*
		std_msgs::Empty)


(clear-all-markers)
(ros::spin)

#|
(setq im (instance visualization_msgs::InteractiveMarker :init))
(send im :name "aaa")
;;(send im :header :frame_id "map")
(send im :header :frame_id "eng2/7f/73B2")
(send im :pose (ros::coords->tf-pose (make-coords)))


(setq markers nil)
(setq marker (cube->marker-msg (make-cube 1000 2000 3000) (send im :header) :alpha 0.2))
(push marker markers)

(setq controls nil)
(setq control (instance visualization_msgs::InteractiveMarkerControl :init))
(send control :markers markers)

(send control :interaction_mode visualization_msgs::InteractiveMarkerControl::*BUTTON*)
(push control controls)

;;(send im :markers markers)
(send im :controls controls)
(ros::publish *add-marker-topic* im)
|#

#|
[visualization_msgs/InteractiveMarker]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
string name
string description
float32 scale
visualization_msgs/MenuEntry[] menu_entries
  uint8 FEEDBACK=0
  uint8 ROSRUN=1
  uint8 ROSLAUNCH=2
  uint32 id
  uint32 parent_id
  string title
  string command
  uint8 command_type
visualization_msgs/InteractiveMarkerControl[] controls
  uint8 INHERIT=0
  uint8 FIXED=1
  uint8 VIEW_FACING=2
  uint8 NONE=0
  uint8 MENU=1
  uint8 BUTTON=2
  uint8 MOVE_AXIS=3
  uint8 MOVE_PLANE=4
  uint8 ROTATE_AXIS=5
  uint8 MOVE_ROTATE=6
  uint8 MOVE_3D=7
  uint8 ROTATE_3D=8
  uint8 MOVE_ROTATE_3D=9
  string name
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
  uint8 orientation_mode
  uint8 interaction_mode
  bool always_visible
  visualization_msgs/Marker[] markers
    uint8 ARROW=0
    uint8 CUBE=1
    uint8 SPHERE=2
    uint8 CYLINDER=3
    uint8 LINE_STRIP=4
    uint8 LINE_LIST=5
    uint8 CUBE_LIST=6
    uint8 SPHERE_LIST=7
    uint8 POINTS=8
    uint8 TEXT_VIEW_FACING=9
    uint8 MESH_RESOURCE=10
    uint8 TRIANGLE_LIST=11
    uint8 ADD=0
    uint8 MODIFY=0
    uint8 DELETE=2
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string ns
    int32 id
    int32 type
    int32 action
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    geometry_msgs/Vector3 scale
      float64 x
      float64 y
      float64 z
    std_msgs/ColorRGBA color
      float32 r
      float32 g
      float32 b
      float32 a
    duration lifetime
    bool frame_locked
    geometry_msgs/Point[] points
      float64 x
      float64 y
      float64 z
    std_msgs/ColorRGBA[] colors
      float32 r
      float32 g
      float32 b
      float32 a
    string text
    string mesh_resource
    bool mesh_use_embedded_materials
  bool independent_marker_orientation
  string description

|#