;;; Copyright (c) 2015, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :nmmi-executive)

(defun init-proprioception ()
  (let* ((tf (make-instance 'cl-tf2:buffer-client))
         (joint-states-fluent (cpl-impl:make-fluent))
         (joint-states-sub (subscribe 
                            "joint_states_throttle" "sensor_msgs/JointState"
                            (lambda (msg) 
                              (setf (cpl:value joint-states-fluent) (from-msg msg)))))
         (arm-error-fluent (cpl-impl:make-fluent))
         (arm-error-sub (subscribe 
                            "arm_pos_error/data" "iai_qb_cube_msgs/CubeDataArray"
                            (lambda (msg) 
                              (setf (cpl:value arm-error-fluent) (from-msg msg))))))
    (values tf joint-states-sub joint-states-fluent arm-error-sub arm-error-fluent)))

(defun guarded-tf2-lookup (tf target-frame source-frame)
  (handler-case (cl-tf2:lookup-transform tf target-frame source-frame 0.0 0.1)
    (cl-tf2::tf2-server-error () (guarded-tf2-lookup tf target-frame source-frame))))

(defun move-finished-p (handle kb target)
  "Predicate to check whether we have reached `target' using `handle' and `kb'."
  (let ((goal (getf-rec kb :targets target)))
    (etypecase goal
      (cl-tf2::stamped-transform
       (let ((target-transform (getf-rec kb :targets target))
             (threshold (or (getf-rec kb :thresholds target)
                            (getf-rec kb :thresholds :default-cartesian))))
         (with-slots (child-frame-id header) target-transform
           (with-slots (frame-id) header
             (let ((current-transform (guarded-tf2-lookup (getf handle :tf) frame-id child-frame-id)))
               (> threshold
                  (v-dist
                   (translation (cl-tf2:transform target-transform))
                   (translation (cl-tf2:transform current-transform)))))))))
      (list
       (let ((threshold (or (getf-rec kb :thresholds target)
                            (getf-rec kb :thresholds :default-joint)))
             (current (cpl-impl:value (getf handle :joint-states-fluent))))
         (> threshold
            (reduce #'+
                    (mapcar (lambda (a b)
                              (abs (- a b)))
                            (getf-multiple current :arm_1_joint :arm_2_joint :arm_3_joint) 
                            (mapcar (alexandria:rcurry #'getf :equilibrium_point) goal)))))))))
       

(defun gripper-finished-p (handle kb target)
  (let ((target-state (getf-rec kb :gripper target :equilibrium_point))
        (threshold (or (getf-rec kb :thresholds :gripper target)
                       (getf-rec kb :thresholds :default-joint)))
        (current-state (getf (cpl-impl:value (getf handle :joint-states-fluent)) :arm_4_joint)))
    
    (> threshold 
       (abs (- target-state current-state)))))

(defun collision-p (handle kb)
  (let* ((threshold (getf-rec kb :thresholds :default-contact-scalar))
         (arm-error (cpl-impl:value (getf handle :arm-error-fluent))))
    (and (> 0.001 (getf arm-error :contact-scalar-gradient))
         (< threshold (getf arm-error :contact-scalar)))))