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
    (:left-pregrasp ((:joint_name "arm_1_joint"
                      :equilibrium_point 0.257 
                      :stiffness_preset 20)
                     (:joint_name "arm_2_joint"
                      :equilibrium_point 1.47
                      :stiffness_preset 20)
                     (:joint_name "arm_3_joint"
                      :equilibrium_point -0.17
                      :stiffness_preset 20))
     :left-pregrasp2 ,(make-stamped-transform 
                   "left_holder_link" "arm_fixed_finger" 0.0
                   (make-transform (make-3d-vector -0.04 0.01 0.0)
                                   (make-identity-rotation)))
     :left-grasp ,(make-stamped-transform 
                   "left_holder_link" "arm_fixed_finger" 0.0
                   (make-transform (make-3d-vector 0.006 0.01 0.0)
                                   (make-identity-rotation)))
     :left-postgrasp ,(make-stamped-transform 
                       "left_holder_link" "arm_fixed_finger" 0.0
                       (make-transform (make-3d-vector -0.0 -0.07 0.0)
                                       (make-identity-rotation)))
     :right-pregrasp ((:joint_name "arm_1_joint"
                      :equilibrium_point -0.257 
                      :stiffness_preset 20)
                     (:joint_name "arm_2_joint"
                      :equilibrium_point -1.47
                      :stiffness_preset 20)
                     (:joint_name "arm_3_joint"
                      :equilibrium_point 0.47
                      :stiffness_preset 20))
     :right-pregrasp3 ,(make-stamped-transform 
                        "right_holder_link" "arm_fixed_finger" 0.0
                        (make-transform (make-3d-vector -0.01 0.04 0.0)
                                        (axis-angle->quaternion
                                         (make-3d-vector 0 0 1) 0.35)))
     :right-grasp ,(make-stamped-transform 
                       "right_holder_link" "arm_fixed_finger" 0.0
                       (make-transform (make-3d-vector 0.0 0.0 0.0)
                                       (axis-angle->quaternion
                                        (make-3d-vector 0 0 1) -0.1)))
     :right-postgrasp ,(make-stamped-transform 
                       "right_holder_link" "arm_fixed_finger" 0.0
                       (make-transform (make-3d-vector -0.02 0.0 0.0)
                                       (make-identity-rotation)))
     :middle ((:joint_name "arm_1_joint"
               :equilibrium_point 0.0
               :stiffness_preset 20)
              (:joint_name "arm_2_joint"
               :equilibrium_point 0.0
               :stiffness_preset 15)
              (:joint_name "arm_3_joint"
               :equilibrium_point 0.0
               :stiffness_preset 15)))
    :stiffness-presets
    (:default (:arm_1_joint 15 :arm_2_joint 15 :arm_3_joint 15)
     :left-postgrasp (:arm_1_joint 30 :arm_2_joint 30 :arm_3_joint 0)
     :right-pregrasp3 (:arm_1_joint 30 :arm_2_joint 30 :arm_3_joint 0)
     :right-grasp (:arm_1_joint 30 :arm_2_joint 30 :arm_3_joint 0))
    :gripper (:open (:joint_name "arm_4_joint" 
                     :equilibrium_point 0.0
                     :stiffness_preset 30)
              :close-tight (:joint_name "arm_4_joint" 
                            :equilibrium_point 1.1
                            :stiffness_preset 30)
              :close-loose (:joint_name "arm_4_joint" 
                            :equilibrium_point 1.1
                            :stiffness_preset 20))
    :thresholds (:default-cartesian 0.015
                 :default-joint 0.15
                 :default-contact-scalar 0.19
                 :gripper (:close-loose 0.35))))

(defun init-nmmi-executive ()
  (multiple-value-bind (arm-control stiff-control joint-control arm-interpolator) 
      (init-arm-control)
    (multiple-value-bind (tf joint-states-sub joint-states-fluent
                          arm-error-sub arm-error-fluent) (init-proprioception)
      ;; wait for everything to settle down
      (sleep 1)
      `(:arm-control ,arm-control
        :stiff-control ,stiff-control
        :joint-control ,joint-control
        :arm-interpolator ,arm-interpolator
        :tf ,tf
        :joint-states-fluent ,joint-states-fluent
        :joint-states-sub ,joint-states-sub
        :arm-error-sub ,arm-error-sub
        :arm-error-fluent ,arm-error-fluent))))

(defun move (handle kb target)
  (let ((finished-fluent (cpl-impl:make-fluent :value nil)))
    (cpl:pursue
      (cpl-impl:wait-for finished-fluent)
      (unless (move-finished-p handle kb target)
        (command-move handle kb target)
        (cpl-impl:whenever ((cpl:pulsed (getf handle :joint-states-fluent)))
          (when (move-finished-p handle kb target)
            (setf (cpl-impl:value finished-fluent) t)))))))

(defun move-for-collision (handle kb target)
  (let ((finished-fluent (cpl-impl:make-fluent :value nil)))
    (cpl:pursue
      (cpl-impl:wait-for finished-fluent)
      (unless (or (move-finished-p handle kb target)
                  (collision-p handle kb))
        (command-move handle kb target)
        (cpl-impl:whenever ((cpl:pulsed (getf handle :joint-states-fluent)))
          (when (or (move-finished-p handle kb target)
                    (collision-p handle kb))
            (setf (cpl-impl:value finished-fluent) t)))))))

(defun gripper (handle kb target)
  (let ((finished-fluent (cpl-impl:make-fluent :value nil)))
    (cpl:pursue
      (cpl-impl:wait-for finished-fluent)
      (unless (gripper-finished-p handle kb target)
        (command-gripper handle kb target)
        (cpl-impl:whenever ((cpl:pulsed (getf handle :joint-states-fluent)))
          (when (gripper-finished-p handle kb target)
            (setf (cpl-impl:value finished-fluent) t)))))))

(defun run-nmmi-executive (handle kb)
  (move handle kb :middle)
  (gripper handle kb :open)
  (move handle kb :left-pregrasp)
  (move handle kb :left-pregrasp2)
  (move-for-collision handle kb :left-grasp)
  (gripper handle kb :close-loose)
  (move handle kb :left-postgrasp)
  (sleep 1)
  (move handle kb :middle)
  (move handle kb :right-pregrasp)
  (move handle kb :right-pregrasp3)
  (move handle kb :right-grasp)
  (gripper handle kb :open)
  (move handle kb :right-postgrasp)
  (sleep 3)
  (move handle kb :middle))

(defun main ()
  (with-ros-node ("nmmi_executive" :spin t)
    (let ((handle (init-nmmi-executive))
          (kb (knowledge-base)))
      (cpl-impl:top-level
        (loop do (run-nmmi-executive handle kb))))))