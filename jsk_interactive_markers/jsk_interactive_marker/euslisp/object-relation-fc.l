#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive_marker")
(ros::roseus "object_relation_fc")
(require :preemptive-task-execution "package://jsk_demo_common/euslisp/preemptive-task-execution.l")
(require "models/room73b2-scene.l")
(require "package://tarukosu_drc_valve_task/euslisp/rotational-joint-model.l")
(require "package://tarukosu_drc_valve_task/euslisp/cylinder-model.l")
(require "package://jsk_interactive_marker/euslisp/object-relation.l")
(ros::roseus-add-msgs "jsk_rviz_plugins")

(setq *target-color* (float-vector 0 0.8 1.0))
(setq *send-object* nil)




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

  (cond
   ((equal (send msg :relation) "target")
    ;;target and reference are different
    (when (equal (send msg :name) (send *reference-object* :name))
      (setq *reference-object* (instance object-relation :init))
      )
    (unless (equal (send msg :name) (send *target-object* :name))
      (setq *target-object* (instance object-relation :init model frame-id (send msg :relation)))
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
  (setq model (send *target-object* :model))
  (print "set-pose")

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

(defun grasp-object (arm &key (ik-check nil) (record t))
  (when (and (equal *recording* "model") record)
    (print "recording")
    (push (list 'grasp-object arm) *recorded-movement*)
    )

  (print "GRASP")
  (setq grasp-part (send *target-object* :grasp-part arm))
  (setq model (send *target-object* :model))
  ;; (send model :move-to
  ;; 	(pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) (send *target-object* :frame-id)) *robot-frame-id*) :world)

  (send grasp-part :set-grasp-coords (send grasp-part :grasp-coords))

  
  (when (send grasp-part :model)
    (setq prev-av (send *pr2* :angle-vector))
    ;;(objects (list *pr2* grasp-coords model))
    

    (dotimes (i 4)
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

      
      (cond
       ((equal arm :larm)
	(setq *look-at-target* (car pre-target-coords))
	)
       ((equal arm :rarm)
	(setq *look-at-target* (cadr pre-target-coords))
	))
       

      (setq pre-grasp-av
	    (send *pr2* :inverse-kinematics pre-target-coords
		  :move-target (send *pr2* :arms :end-coords)
		  ;;:debug-view t
		  :look-at-target *look-at-target*
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

      (cond
       ((equal arm :larm)
	(setq *look-at-target* (car target-coords))
	)
       ((equal arm :rarm)
	(setq *look-at-target* (cadr target-coords))
	))
       
      (setq grasp-av
	    (send *pr2* :inverse-kinematics target-coords
		  :move-target (send *pr2* :arms :end-coords)
		  :look-at-target *look-at-target*
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
	(cond
	 ((equal arm :larm)
	  (setq *look-at-target* (car target-coords))
	  )
	 ((equal arm :rarm)
	  (setq *look-at-target* (cadr target-coords))
	  ))

	(push 
	 (send *pr2* :inverse-kinematics target-coords
	       :move-target (send *pr2* :arms :end-coords)
	       :look-at-target *look-at-target*
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
      (let (pick-up-ac pick-up-task1)
	(setq pick-up-ac (action action1
				 (motion motion1
					 (send *ri* :angle-vector pre-grasp-av 5000)
					 (send *ri* :wait-interpolation))
				 (motion motion2
					 (send *ri* :angle-vector-sequence middle-av-list middle-target-time-list)
					 (send *ri* :wait-interpolation)
					 )
				 )
	      )
	(setq pick-up-task1 (task (list pick-up-ac)))
	(execute-task pick-up-task1 :loop-check-error-p nil)
	)
      
      ;;(send *ri* :angle-vector grasp-av 2000)
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
      (let (release-ac release-task1)
	(setq release-ac (action action1
				 (motion motion1
					 (send *ri* :angle-vector release-av 2000)
					 (send *ri* :wait-interpolation)
					 )
				 )
	      )
	(setq release-task1 (task (list release-ac)))
	(execute-task release-task1  :loop-check-error-p nil)
	)
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
    (let (send-angle-ac send-angle-task1)
      (setq send-angle-ac (action action1
				  (motion motion1
					  (send *ri* :angle-vector av time)
					  (send *ri* :wait-interpolation)
					  )
				  )
	    )
      (setq send-angle-task1 (task (list send-angle-ac)))
      (execute-task send-angle-task1  :loop-check-error-p nil)
      )
    )
   )
  )


(defun send-angle-vector-sequence (avs times &key (ik-check nil))
  (cond
   (ik-check
    t
    )
   (t
    (if (equal (length avs) 10)
	(progn
	  (let (av1 av2 av3 av4 av5 av6 av7 av8 av9 av0 av10 tm ac task1 )
	    (setq av1 (elt avs 0))  (setq av2 (elt avs 1))  (setq av3 (elt avs 2))
	    (setq av4 (elt avs 3))  (setq av5 (elt avs 4))  (setq av6 (elt avs 5))
	    (setq av7 (elt avs 6))  (setq av8 (elt avs 7))  (setq av9 (elt avs 8))
	    (setq av10 (elt avs 9))
	    (setq tm (* 1000 1000))
	    (setq ac (action action1
			     (motion motion1
				     (send *ri* :angle-vector av1)
				     (send *ri* :wait-interpolation))
			     (motion motion2
				     (send *ri* :angle-vector av2)
				     (send *ri* :wait-interpolation))
			     (motion motion3
				     (send *ri* :angle-vector av3)
				     (send *ri* :wait-interpolation))
			     (motion motion4
				     (send *ri* :angle-vector av4)
				     (send *ri* :wait-interpolation))
			     (motion motion5
				     (send *ri* :angle-vector av5)
				     (send *ri* :wait-interpolation))
			     (motion motion6
				     (send *ri* :angle-vector av6)
				     (send *ri* :wait-interpolation))
			     (motion motion7
				     (send *ri* :angle-vector av7)
				     (send *ri* :wait-interpolation))
			     (motion motion8
				     (send *ri* :angle-vector av8)
				     (send *ri* :wait-interpolation))
			     (motion motion9
				     (send *ri* :angle-vector av9)
				     (send *ri* :wait-interpolation))
			     (motion motion10
				     (send *ri* :angle-vector av10)
				     (send *ri* :wait-interpolation))))
	    (setq task1 (task (list ac)))
	    (execute-task task1 :loop-check-error-p nil)
	    ))
      (progn
	(setq send-angle-seq-ac (action action1
					(motion motion1
						(send *ri* :angle-vector-sequence avs times)
						(send *ri* :wait-interpolation)
						)
					)
	      )
	(setq send-angle-seq-task1 (task (list send-angle-seq-ac)))
	(execute-task send-angle-seq-task1 :loop-check-error-p nil)))
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

      ;;(x::window-main-one)

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

      (cond
       ((equal arm :larm)
	(setq *look-at-target* (car target-coords))
	)
       ((equal arm :rarm)
	(setq *look-at-target* (cadr target-coords))
	))


      (setq ik-suc
	    (send *pr2* :inverse-kinematics target-coords
		  :debug-view nil
		  :look-at-target *look-at-target*
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
	;;(return)
	(send model :move-to (send start-model :copy-worldcoords) :world)
	(send model :angle-vector (send start-model :angle-vector))
	(return-from object-move)
	))
      )

    (cond
     (ik-check
      (print (length avs))
      (print interpolation-num)
      (equal (length avs) interpolation-num)
      )
     (t
      (when avs
	(print avs)
	(setq avs (reverse avs))
	(setq a avs)

	;;	(send *ri* :angle-vector-sequence avs 2000)
	;;	(send *ri* :wait-interpolation)

	(setq av1 (elt avs 0))
	(setq av2 (elt avs 1))
	(setq av3 (elt avs 2))
	(setq av4 (elt avs 3))
	(setq av5 (elt avs 4))
	(setq av6 (elt avs 5))
	(setq av7 (elt avs 6))
	(setq av8 (elt avs 7))
	(setq av9 (elt avs 8))
	(setq av10 (elt avs 9))
	(setq tm (* 1000 1000))
	(setq ac (action action1
			 (motion motion1
				 (send *ri* :angle-vector av1)
				 (send *ri* :wait-interpolation))
			 
			 ;;                           (unix:usleep tm))
			 (motion motion2
				 (send *ri* :angle-vector av2)
				 (send *ri* :wait-interpolation))
			 ;;                           (unix:usleep tm))
			 (motion motion3
				 (send *ri* :angle-vector av3)
				 (send *ri* :wait-interpolation))

			 ;;                           (unix:usleep tm))
			 (motion motion4
				 (send *ri* :angle-vector av4)
				 (send *ri* :wait-interpolation))
			 ;;                           (unix:usleep tm))
			 (motion motion5
				 (send *ri* :angle-vector av5)
				 (send *ri* :wait-interpolation))
			 ;;                           (unix:usleep tm))
			 (motion motion6
				 (send *ri* :angle-vector av6)
				 (send *ri* :wait-interpolation))
			 ;;                           (unix:usleep tm))
			 (motion motion7
				 (send *ri* :angle-vector av7)
				 (send *ri* :wait-interpolation))
			 ;;                           (unix:usleep tm))
			 (motion motion8
				 (send *ri* :angle-vector av8)
				 (send *ri* :wait-interpolation))
			 ;;                           (unix:usleep tm))
			 (motion motion9
				 (send *ri* :angle-vector av9)
				 (send *ri* :wait-interpolation))
			 ;;                           (unix:usleep tm))
			 (motion motion10
				 (send *ri* :angle-vector av10)
				 (send *ri* :wait-interpolation))))
	;;                           (unix:usleep tm))))
	(setq task1 (task (list ac)))
	(execute-task task1 :loop-check-error-p nil)


	(when (equal *recording* "robot")
	  (print "recording angle vector")
	  (push (list 'send-angle-vector-sequence avs 2000) *recorded-movement*)
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

(setq trans (send (send *pr2* :head :end-coords) :transformation (send *pr2* :links :openni_rgb_optical_frame)))
(send (send *pr2* :head :end-coords) :transform trans)

(objects (list *pr2*))



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

(defun make-model-callback
  (msg)
  (print-ros-msg msg)
  (unless (string-equal (send msg :model) "")
    (setq model-list (mapcar #'read-from-string (split-string (send msg :model) " ")))
    (cond
     ((functionp (car model-list))
      (setq model (apply (car model-list) (cdr model-list)))
      ;;(send model :name (send msg :name))
      )
     (t
      (return-from object-relation-callback nil))
     )

    ;;(setq model (funcall (read-from-strimg (send msg :model))))
    (setq frame-id "")
    (setq *target-object* (instance object-relation :init model frame-id "target"))
    (objects (list *pr2* model))
    )
  (setq model (send *target-object* :model))
  (setq start-model (copy-object (send *target-object* :model)))
  (unless model
    (return-from make-model-callback))
  
  (unless (string-equal (send msg :pose :header :frame_id) "")
    (send model :move-to (ros::tf-pose-stamped->coords (send msg :pose)) :world)
    )

  (cond
   ((string-equal (send msg :type) "grasp_larm")
    (setq selection-link (elt (send model :links) (send msg :grasp_link_index)))
    (send *target-object* :set-selection "aa" selection-link "grasp_point_larm" (send msg :grasp_link_index))
    
    (send *target-object* :selection "grasp_point_larm" (send msg :grasp_type) (send msg :grasp_index))
    (setq grasp-larm-part (send *target-object* :grasp-larm-part))
    (send grasp-larm-part :set-distance (send msg :grasp_distance))
    (grasp-object :larm)
    )
   ((string-equal (send msg :type) "grasp_rarm")
    (setq selection-link (elt (send model :links) (send msg :grasp_link_index)))
    (send *target-object* :set-selection "aa" selection-link "grasp_point_rarm" (send msg :grasp_link_index))
    
    (send *target-object* :selection "grasp_point_rarm" (send msg :grasp_type) (send msg :grasp_index))
    (setq grasp-rarm-part (send *target-object* :grasp-rarm-part))
    (send grasp-rarm-part :set-distance (send msg :grasp_distance))
    (grasp-object :rarm)
    )

   ((string-equal (send msg :type) "release_larm")
    (release-object :larm)
    )
   ((string-equal (send msg :type) "release_rarm")
    (release-object :rarm)
    )
   ((string-equal (send msg :type) "set_pose")
    (send model :move-to (ros::tf-pose-stamped->coords (send msg :pose)) :world)
    (set-pose :rarm)
    (set-pose :larm)
    )
   ((string-equal (send msg :type) "move")
    (setq goal-model (copy-object model))
    (apply-joint_state (send msg :joint_state) goal-model)
    (setq js-diff (v- (send goal-model :angle-vector) (send start-model :angle-vector)))
    (setq cds-diff (send (send start-model :worldcoords) :transformation (send goal-model :worldcoords)))
    
    (send model :angle-vector (send start-model :angle-vector))
    (send model :move-to (send start-model :copy-worldcoords) :world)

    (object-move cds-diff js-diff :record *recording*)
    )
   )
  )

(setq *make-model-topic* #'make-model-callback)

(ros::unsubscribe *marker-feedback-topic*)
(ros::unsubscribe *object-relation-topic*)
(ros::unsubscribe *model-feedback-topic*)

(ros::unsubscribe *object-move-topic*)




(ros::subscribe *publish-make-model-topic*
		jsk_interactive_marker::MakeModel #'make-model-callback)

;; (ros::subscribe *object-move-topic*
;; 		jsk_interactive_marker::MoveModel #'object-move-callback)

(ros::subscribe *record-topic*
		jsk_rviz_plugins::RecordCommand #'record-callback)

(ros::advertise *add-marker-topic*
		visualization_msgs::InteractiveMarker)

(ros::advertise *grasp-marker-topic*
		visualization_msgs::Marker)

(ros::advertise *clear-marker-topic*
		std_msgs::Empty)


(clear-all-markers)
(setq *ocs* nil)
(while (ros::ok)
  ;;(x::window-main-one)
  (ros::spin-once)
  )
(ros::spin)


;; #f(50.0476 57.9131 72.8416 68.8176 -120.775 18.746 -27.978 181.044 -14.0341 34.9127 -90.4354 -76.1615 -52.4834 -24.1792 190.927 0.0 0.0)


