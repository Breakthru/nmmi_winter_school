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

(defun knowledge-base ()
  `(:targets
    (:left-pregrasp ,(make-stamped-transform 
                      "left_holder_link" "arm_fixed_finger" 0.0
                      (make-transform (make-3d-vector 0.0 -0.05 0.0)
                                      (make-identity-rotation)))
     :right-pregrasp ,(make-stamped-transform 
                       "right_holder_link" "arm_fixed_finger" 0.0
                       (make-transform (make-3d-vector 0.0 0.05 0.0)
                                       (make-identity-rotation)))
     :middle ,(make-stamped-transform 
               "base_link_zero" "arm_fixed_finger" 0.0
               (make-transform (make-3d-vector 0.244 0.0 0.0)
                               (make-quaternion 1.0 0.0 0.0 0.0))))
    :stiffness-presets
    (:default (:arm_1_joint 15 :arm_2_joint 15 :arm_3_joint 15)
     :right-pregrasp (:arm_1_joint 0 :arm_2_joint 0 :arm_3_joint 0))
    :gripper (:open (:joint_name "arm_4_joint" 
                     :equilibrium_point ,(/ PI -4.0) 
                     :stiffness_preset 20)
              :close (:joint_name "arm_4_joint" 
                     :equilibrium_point 0.0 
                     :stiffness_preset 20))
    :thresholds (:default-cartesian 0.02
                 :default-joint 0.05)))

(defun init-nmmi-executive ()
  (multiple-value-bind (arm-control stiff-control joint-control) (init-arm-control)
    (multiple-value-bind (tf joint-states-sub joint-states-fluent) (init-proprioception)
      ;; wait for everything to settle down
      (sleep 1)
      `(:arm-control ,arm-control 
        :stiff-control ,stiff-control 
        :joint-control ,joint-control
        :tf ,tf
        :joint-states-fluent ,joint-states-fluent
        :joint-states-sub ,joint-states-sub))))

(defun move (handle kb target)
  (let ((finished-fluent (cpl-impl:make-fluent :value nil)))
    (cpl:pursue
      (cpl-impl:wait-for finished-fluent)
      (cpl-impl:whenever ((cpl:pulsed (getf handle :joint-states-fluent)))
        (if (move-finished-p handle kb target)
            (setf (cpl-impl:value finished-fluent) t)
            (command-move handle kb target))))))

(defun gripper (handle kb target)
  (let ((finished-fluent (cpl-impl:make-fluent :value nil)))
    (cpl:pursue
      (cpl-impl:wait-for finished-fluent)
      (cpl-impl:whenever ((cpl:pulsed (getf handle :joint-states-fluent)))
        (if (gripper-finished-p handle kb target)
            (setf (cpl-impl:value finished-fluent) t)
            (command-gripper handle kb target))))))
    
(defun run-nmmi-executive (handle kb)
  (move handle kb :middle)
  (gripper handle kb :open)
  (move handle kb :left-pregrasp)
  (gripper handle kb :close)
  (move handle kb :middle)
  (move handle kb :right-pregrasp)
  (gripper handle kb :open))

(defun main ()
  (with-ros-node ("nmmi_executive" :spin t)
    (let ((handle (init-nmmi-executive))
          (kb (knowledge-base)))
      (cpl-impl:top-level
        (loop do (run-nmmi-executive handle kb))))))