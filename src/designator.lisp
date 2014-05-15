;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :designator-integration-lisp)

(defun call-designator-service (topic desig-request)
  (let ((result (roslisp:call-service
                 topic
                 "designator_integration_msgs/DesignatorCommunication"
                 :request (roslisp:make-message
                           "designator_integration_msgs/DesignatorRequest"
                           :designator (designator->msg desig-request)))))
    (roslisp:with-fields (response) result
      (roslisp:with-fields (designators) response
      (map 'list (lambda (x)
                   (msg->designator x))
           designators)))))

(defun description->msg (desc &key (index 0) (parent 0))
  (case (type-of (car desc))
    (common-lisp:symbol
     ;; It is a single pair
     (let* ((key (string (car desc)))
            (value (car (cdr desc)))
            (type
              (cond
                ((or (symbolp value)
                     (stringp value)) 0)
                ((numberp value) 1)
                ((eql (type-of value) 'geometry_msgs-msg:posestamped) 4)
                ((eql (type-of value) 'geometry_msgs-msg:pose) 5)
                ((eql (type-of value) 'cl-tf:pose) 5)
                ((eql (type-of value) 'cl-tf:pose-stamped) 4)
                ((eql (type-of value) 'cram-designators:action-designator) 6)
                ((eql (type-of value) 'cram-designators:object-designator) 7)
                ((eql (type-of value) 'cram-designators:location-designator) 8)
                (t 3))) ;; Default: list
            (new-index (+ index 1)))
       (let ((type (cond ((or (eql type 6) (eql type 7) (eql type 8)) 3)
                         (t type))) ;; Treat designators as lists
             (value (cond ((or (eql type 6) (eql type 7) (eql type 8)) ;; Take designator's description as list
                           (append (description value)
                                   (list `(_designator_type
                                           ,(case type
                                              (6 'action)
                                              (7 'object)
                                              (8 'location))))
                                   (list `(_designator_memory_address
                                           ,(write-to-string
                                             (sb-kernel:get-lisp-obj-address value))))))
                          (t value))))
         (cond ((eql (type-of value) 'common-lisp:cons)
                ;; Value is list
                (multiple-value-bind
                      (msgs high-index)
                    (description->msg
                     value :index new-index
                           :parent new-index)
                  (values
                   (append
                    (list
                     (roslisp:make-message
                      "designator_integration_msgs/KeyValuePair"
                      :id new-index
                      :parent parent
                      :key key
                      :type type))
                    msgs)
                   (max new-index high-index))))
               (t
                ;; Value is symbol/number/string/pose(stamped)
                (values
                 (roslisp:make-message
                  "designator_integration_msgs/KeyValuePair"
                  :id new-index
                  :parent parent
                  :key key
                  :type type
                  :value_string (cond ((or (symbolp value)
                                           (stringp value))
                                       (string value))
                                      (t ""))
                  :value_float (cond ((numberp value)
                                      value)
                                     (t 0.0))
                  :value_posestamped
                  (cond ((eql (type-of value) 'geometry_msgs-msg:posestamped)
                         value)
                        ((eql (type-of value) 'cl-tf:pose-stamped)
                         (tf:pose-stamped->msg value))
                        (t (tf:pose-stamped->msg
                            (tf:pose->pose-stamped
                             "" 0.0
                             (tf:make-identity-pose)))))
                  :value_pose
                  (cond ((eql (type-of value) 'geometry_msgs-msg:pose)
                         value)
                        ((eql (type-of value) 'cl-tf:pose)
                         (tf:pose->msg value))
                        (t (tf:pose->msg (tf:make-identity-pose)))))
                 new-index))))))
    (common-lisp:cons
     ;; It is a list of pairs
     (let ((index index))
       (values
        (loop for pair in desc
              for msg = (multiple-value-bind
                              (msg new-index)
                            (description->msg
                             pair :index index :parent parent)
                          (setf index new-index)
                          msg)
              collect msg)
        index)))))

(defun designator->msg (desig)
  (let* ((type (ecase (class-name (class-of desig))
                 (desig:object-designator 0)
                 (desig:action-designator 1)
                 (desig:location-designator 2)))
         (description (desig:description desig))
         (desc-msgs (alexandria:flatten
                     (description->msg description)))
         (vec-msgs (map 'vector #'identity desc-msgs)))
    (roslisp:make-message
     "designator_integration_msgs/Designator"
     :type type
     :description vec-msgs)))

(defun msg->description (msg)
  (roslisp:with-fields
      (type key value_string value_float value_posestamped value_pose) msg
    (append
     `(,(intern (string-upcase key)
                'cram-designator-properties))
     (ecase type
       (0 (list value_string))
       (1 (list value_float))
       (3 `())
       (4 (list value_posestamped))
       (5 (list value_pose))))))

(defun msg->designator (msg)
  (roslisp:with-fields (type description) msg
    (let* ((final-tier nil)
           (desc-pairs (map 'list
                            (lambda (desc)
                              (cons
                               desc (msg->description desc)))
                            description)))
      (flet ((get-leaf-nodes ()
               (let ((indices
                       (loop for pair in desc-pairs
                             collect (roslisp:with-fields (parent) (car pair)
                                       parent))))
                 (loop for pair in desc-pairs
                       when (roslisp:with-fields (id) (car pair)
                              (not (position id indices)))
                         collect pair)))
              (get-pair (id)
                (loop for pair in desc-pairs
                      for id-current = (roslisp:with-fields ((id-current id)) (car pair)
                                         id-current)
                      when (eql id id-current)
                        do (return-from get-pair pair)))
              (pair-properties (pair)
                (third pair))
              (pair-keyname (pair)
                (second pair))
              (pair-parent-id (pair)
                (roslisp:with-fields (parent) (first pair)
                  parent))
              (pair-id (pair)
                (roslisp:with-fields (id) (first pair)
                  id)))
        (flet ((remove-pair (id)
                 (setf desc-pairs
                       (remove id desc-pairs
                               :test (lambda (id x)
                                       (roslisp:with-fields ((id-current id))
                                           (car x)
                                         (eql id id-current)))))))
          (flet ((add-child-pair (parent-id child-id)
                   (let ((parent-pair (get-pair parent-id))
                         (child-pair (get-pair child-id)))
                     (remove-pair parent-id)
                     (remove-pair child-id)
                     (let* ((parent-keyname (pair-keyname parent-pair))
                            (parent-props (pair-properties parent-pair))
                            (child-keyname (pair-keyname child-pair))
                            (child-props (pair-properties child-pair))
                            (merged-props (append
                                           parent-props
                                           (list (list child-keyname
                                                       child-props)))))
                       (push (cons (car parent-pair)
                                   (list parent-keyname merged-props))
                             desc-pairs)))))
            (flet ((move-node-to-final-tier (pair)
                     (let ((id (pair-id pair)))
                       (remove-pair id)
                       (push (rest pair) final-tier))))
              (loop while desc-pairs do
                (let ((leaf-nodes (get-leaf-nodes)))
                  (loop for node in leaf-nodes do
                    (let ((parent-id (pair-parent-id node)))
                      (case parent-id
                        (0 (move-node-to-final-tier node))
                        (t (add-child-pair (pair-parent-id node)
                                           (pair-id node))))))))))))
      (case type
        (0 (cram-designators:make-designator 'cram-designators:object final-tier))
        (1 (cram-designators:make-designator 'cram-designators:action final-tier))
        (2 (cram-designators:make-designator 'cram-designators:location final-tier))
        (3 (cram-designators:make-designator 'cram-designators:object final-tier)) ;; Unknown -> assume an object
        ))))
