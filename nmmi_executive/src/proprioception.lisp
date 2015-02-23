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
                            (lambda (msg) (setf (cpl:value joint-states-fluent) msg)))))
    (values tf joint-states-sub joint-states-fluent)))

;;;
;;; COMPUTE THE DISTANCE BETWEEN EE AND GOALS TO DISCRETIZE SPACE:
;;; - TF holds frames for the goals, TF has EE
;;;

;; (defun guarded-tf2-lookup (tf target-frame source-frame)
;;   (handler-case (cl-tf2:lookup-transform tf target-frame source-frame 0.0 0.1)
;;     (cl-tf2::tf2-server-error () (guarded-tf2-lookup tf target-frame source-frame))))

;; (defun transform-distance (stamped-transform)
;;   (cl-transforms:v-norm (translation (cl-tf2:transform stamped-transform))))

;; (defun discrete-localization (handle)
;;   (labels ((lookup-places-rec (places tf)
;;              (when places
;;                (destructuring-bind (key frame &rest remainder) places
;;                  (concatenate 'list
;;                               `(,key ,(transform-distance (guarded-tf2-lookup tf "arm_fixed_finger" frame)))
;;                               (lookup-places-rec remainder tf))))))
;;     (lookup-places-rec 
;;      `(:left-target "left_holder_link" :right-target "right_holder_link")
;;      (getf handle :tf))))
