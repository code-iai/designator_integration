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

(defun value->type-code (value)
  (cond
    ((or (symbolp value)
         (stringp value)) 0)
    ((numberp value) 1)
    ((eql (type-of value) 'geometry_msgs-msg:posestamped) 4)
    ((eql (type-of value) 'geometry_msgs-msg:pose) 5)
    ((eql (type-of value) 'cl-transforms:pose) 5)
    ((eql (type-of value) 'cl-transforms-stamped:pose-stamped) 4)
    ((eql (type-of value) 'cram-designators:action-designator) 6)
    ((eql (type-of value) 'cram-designators:object-designator) 7)
    ((eql (type-of value) 'cram-designators:location-designator) 8)
    ((eql (type-of value) 'cram-designators:human-designator) 9)
    ((eql (type-of value) 'geometry_msgs-msg:point) 10)
    ((eql (type-of value) 'geometry_msgs-msg:wrench) 11)
    (t 3))) ;; Default: list

(defun is-type-designator (type)
  (or (eql type 6) (eql type 7) (eql type 8) (eql type 9)))

(defun description->msg (desc &key (index 0) (parent 0))
  (case (type-of (car desc))
    ((common-lisp:symbol common-lisp:keyword)
     ;; It is a single pair
     (let* ((key (string (car desc)))
            (value (car (cdr desc)))
            (type (value->type-code value))
            (new-index (+ index 1)))
       (let ((value (cond ((is-type-designator type) ;; Use designator's description as list
                           (append (description value)
                                   (list `(_designator_type
                                           ,(case type
                                              (6 'action)
                                              (7 'object)
                                              (8 'location)
                                              (9 'human))))
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
               ((and (eql (type-of (type-of value)) 'common-lisp:cons)
                     (eql (first (type-of value)) 'simple-vector))
                ;; Value is nested list <-> vector
                (list->msg (map 'list #'identity value) parent new-index
                           :key key))
               (t
                ;; Value is symbol/number/string/pose(stamped)
                (values
                 (roslisp:make-message
                  "designator_integration_msgs/KeyValuePair"
                  :id new-index
                  :parent parent
                  :key key
                  :type type
                  :value_point (cond ((typep value 'geometry_msgs-msg:point) value)
                                     (t (make-msg "geometry_msgs/Point")))
                  :value_wrench (cond ((typep value 'geometry_msgs-msg:wrench) value)
                                     (t (make-msg "geometry_msgs/Wrench")))
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
                        ((eql (type-of value) 'cl-transforms-stamped:pose-stamped)
                         (cl-transforms-stamped:to-msg value))
                        (t (cl-transforms-stamped:to-msg
                            (cl-transforms-stamped:make-pose-stamped
                             "" 0.0
                             (cl-transforms:make-identity-vector)
                             (cl-transforms:make-identity-rotation)))))
                  :value_pose
                  (cond ((eql (type-of value) 'geometry_msgs-msg:pose)
                         value)
                        ((eql (type-of value) 'cl-transforms:pose)
                         (cl-transforms-stamped:to-msg value))
                        (t (cl-transforms-stamped:to-msg
                            (cl-transforms:make-identity-pose)))))
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

(defun nested-list->msg (nested-list)
  (roslisp:make-message
   "designator_integration_msgs/Designator"
   :type 3
   :description
   (map 'vector #'identity (list->msg nested-list 0 0))))

(defun list-item->msg (value parent-index current-index)
  (roslisp:make-message
   "designator_integration_msgs/KeyValuePair"
   :id current-index
   :parent parent-index
   :key ""
   :type (value->type-code value)
   :value_string (cond ((or (symbolp value)
                            (stringp value))
                        (string value))
                       (t ""))
   :value_float (cond ((numberp value)
                       value)
                      (t 0.0))
   :value_posestamped (cond ((eql (type-of value) 'geometry_msgs-msg:posestamped)
                             value)
                            ((eql (type-of value) 'cl-transforms-stamped:pose-stamped)
                             (cl-transforms-stamped:to-msg value))
                            (t (cl-transforms-stamped:to-msg
                                (cl-transforms-stamped:make-pose-stamped
                                 "" 0.0
                                 (cl-transforms:make-identity-vector)
                                 (cl-transforms:make-identity-rotation)))))
   :value_point (cond ((typep value 'geometry_msgs-msg:point) value)
                      (t (make-msg "geometry_msgs/Point")))
   :value_wrench (cond ((typep value 'geometry_msgs-msg:wrench) value)
                       (t (make-msg "geometry_msgs/Wrench")))
   :value_pose (cond ((eql (type-of value) 'geometry_msgs-msg:pose)
                      value)
                     ((eql (type-of value) 'cl-transforms:pose)
                      (cl-transforms-stamped:to-msg value))
                     (t (cl-transforms-stamped:to-msg
                         (cl-transforms:make-identity-pose))))))

(defun list->msg (lst parent-index highest-index &key (key ""))
  (let* ((list-element-id (incf highest-index))
         (list-element
           (make-message
            "designator_integration_msgs/KeyValuePair"
            :id list-element-id
            :parent parent-index
            :key key
            :type 3)))
    (values
     (alexandria:flatten
      (append
       `(,list-element)
       (loop for list-item in lst
             for ct = (cond ((listp list-item)
                             (multiple-value-bind
                                   (result last-index)
                                 (list->msg
                                  list-item
                                  list-element-id
                                  highest-index)
                               (setf highest-index last-index)
                               result))
                            ((is-type-designator (value->type-code list-item))
                             (let ((list-desc
                                     (append (description list-item)
                                             (list `(_designator_type
                                                     ,(case (value->type-code list-item)
                                                        (6 'action)
                                                        (7 'object)
                                                        (8 'location)
                                                        (9 'human))))
                                             (list `(_designator_memory_address
                                                     ,(write-to-string
                                                       (sb-kernel:get-lisp-obj-address list-item)))))))
                               (multiple-value-bind
                                     (result last-index)
                                   (list->msg
                                    list-desc
                                    list-element-id
                                    highest-index)
                                 (setf highest-index last-index)
                                 result)))
                            (t (list-item->msg
                                list-item
                                list-element-id
                                (incf highest-index))))
             collect ct)))
      highest-index)))

(defun designator->msg (desig)
  (let* ((type (ecase (class-name (class-of desig))
                 (desig:object-designator 0)
                 (desig:action-designator 1)
                 (desig:location-designator 2)
                 (desig:human-designator 3)))
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
     `(,(intern (string-upcase key) :keyword))
     (ecase type
       (0 (list value_string))
       (1 (list value_float))
       (3 `())
       (4 (list (cl-transforms-stamped:from-msg value_posestamped)))
       (5 (list (cl-transforms-stamped:from-msg value_pose)))))))

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
        (0 (cram-designators:make-designator :object final-tier))
        (1 (cram-designators:make-designator :action final-tier))
        (2 (cram-designators:make-designator :location final-tier))
        (3 (cram-designators:make-designator :human final-tier))
        (4 (cram-designators:make-designator :object final-tier)) ;; Unknown -> assume an object
        ))))
