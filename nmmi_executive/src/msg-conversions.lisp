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

(defgeneric to-msg (data))

(defmethod to-msg ((data cl-transforms:3d-vector))
  (with-slots (x y z) data
    (make-msg "geometry_msgs/Vector3" :x x :y y :z z)))
   
(defmethod to-msg ((data cl-transforms:quaternion))
  (with-slots (x y z w) data
    (make-msg "geometry_msgs/Quaternion" :x x :y y :z z :w w)))

(defmethod to-msg ((data cl-transforms:transform))
  (with-slots (translation rotation) data
    (make-msg "geometry_msgs/Transform" 
              :translation (to-msg translation):rotation (to-msg rotation))))

(defmethod to-msg ((data cl-tf2::header))
  (with-slots (frame-id stamp) data
    (make-msg "std_msgs/Header" :stamp stamp :frame_id frame-id)))
  
(defmethod to-msg ((data cl-tf2::stamped-transform))
  (with-slots (header child-frame-id (transform cl-tf2:transform)) data
    (make-msg 
     "geometry_msgs/TransformStamped" 
     :header (to-msg header) :child_frame_id child-frame-id :transform (to-msg transform))))

(defun to-stiffness-msg (stiffness-presets)
  (labels ((to-msg-rec (stiffness-presets)
           (when stiffness-presets
             (destructuring-bind (joint-name stiffness-preset &rest remainder) 
                 stiffness-presets
               (concatenate 'list
                            (list (make-msg "iai_qb_cube_msgs/CubeStiff"
                                            :joint_name (string-downcase (string joint-name))
                                            :stiffness_preset stiffness-preset))
                            (to-msg-rec remainder))))))
    (make-msg "iai_qb_cube_msgs/CubeStiffArray"
              :stiffness_presets (coerce (to-msg-rec stiffness-presets) 'vector))))

(defun to-joint-cmd-msg (cmds)
  (let ((msg-list 
          (mapcar (lambda (cmd)
                    (apply #'roslisp::make-message-fn "iai_qb_cube_msgs/CubeCmd" cmd))
                  cmds)))
  (make-msg "iai_qb_cube_msgs/CubeCmdArray" :commands (coerce msg-list 'vector))))

(defgeneric from-msg (msg))

(defmethod from-msg ((msg sensor_msgs-msg:jointstate))
  (with-fields (name position) msg
    (apply #'concatenate 'list
           (loop for n in (coerce name 'list)
                 for p in (coerce position 'list)
                 collect `(,(intern (string-upcase n) :keyword) ,p)))))

(defmethod from-msg ((msg iai_qb_cube_msgs-msg:cubedataarray))
  (with-fields (contact_scalar contact_scalar_gradient) msg
    `(:contact-scalar ,contact_scalar
      :contact-scalar-gradient ,contact_scalar_gradient)))