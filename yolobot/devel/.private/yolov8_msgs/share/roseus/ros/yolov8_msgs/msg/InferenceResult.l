;; Auto-generated. Do not edit!


(when (boundp 'yolov8_msgs::InferenceResult)
  (if (not (find-package "YOLOV8_MSGS"))
    (make-package "YOLOV8_MSGS"))
  (shadow 'InferenceResult (find-package "YOLOV8_MSGS")))
(unless (find-package "YOLOV8_MSGS::INFERENCERESULT")
  (make-package "YOLOV8_MSGS::INFERENCERESULT"))

(in-package "ROS")
;;//! \htmlinclude InferenceResult.msg.html


(defclass yolov8_msgs::InferenceResult
  :super ros::object
  :slots (_class_name _top _left _bottom _right ))

(defmethod yolov8_msgs::InferenceResult
  (:init
   (&key
    ((:class_name __class_name) "")
    ((:top __top) 0)
    ((:left __left) 0)
    ((:bottom __bottom) 0)
    ((:right __right) 0)
    )
   (send-super :init)
   (setq _class_name (string __class_name))
   (setq _top (round __top))
   (setq _left (round __left))
   (setq _bottom (round __bottom))
   (setq _right (round __right))
   self)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:top
   (&optional __top)
   (if __top (setq _top __top)) _top)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:bottom
   (&optional __bottom)
   (if __bottom (setq _bottom __bottom)) _bottom)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; string _class_name
    4 (length _class_name)
    ;; int64 _top
    8
    ;; int64 _left
    8
    ;; int64 _bottom
    8
    ;; int64 _right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; int64 _top
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _top (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _top) (= (length (_top . bv)) 2)) ;; bignum
              (write-long (ash (elt (_top . bv) 0) 0) s)
              (write-long (ash (elt (_top . bv) 1) -1) s))
             ((and (class _top) (= (length (_top . bv)) 1)) ;; big1
              (write-long (elt (_top . bv) 0) s)
              (write-long (if (>= _top 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _top s)(write-long (if (>= _top 0) 0 #xffffffff) s)))
     ;; int64 _left
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _left (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _left) (= (length (_left . bv)) 2)) ;; bignum
              (write-long (ash (elt (_left . bv) 0) 0) s)
              (write-long (ash (elt (_left . bv) 1) -1) s))
             ((and (class _left) (= (length (_left . bv)) 1)) ;; big1
              (write-long (elt (_left . bv) 0) s)
              (write-long (if (>= _left 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _left s)(write-long (if (>= _left 0) 0 #xffffffff) s)))
     ;; int64 _bottom
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _bottom (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _bottom) (= (length (_bottom . bv)) 2)) ;; bignum
              (write-long (ash (elt (_bottom . bv) 0) 0) s)
              (write-long (ash (elt (_bottom . bv) 1) -1) s))
             ((and (class _bottom) (= (length (_bottom . bv)) 1)) ;; big1
              (write-long (elt (_bottom . bv) 0) s)
              (write-long (if (>= _bottom 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _bottom s)(write-long (if (>= _bottom 0) 0 #xffffffff) s)))
     ;; int64 _right
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _right (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _right) (= (length (_right . bv)) 2)) ;; bignum
              (write-long (ash (elt (_right . bv) 0) 0) s)
              (write-long (ash (elt (_right . bv) 1) -1) s))
             ((and (class _right) (= (length (_right . bv)) 1)) ;; big1
              (write-long (elt (_right . bv) 0) s)
              (write-long (if (>= _right 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _right s)(write-long (if (>= _right 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _top
#+(or :alpha :irix6 :x86_64)
      (setf _top (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _top (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _left
#+(or :alpha :irix6 :x86_64)
      (setf _left (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _left (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _bottom
#+(or :alpha :irix6 :x86_64)
      (setf _bottom (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _bottom (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _right
#+(or :alpha :irix6 :x86_64)
      (setf _right (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _right (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get yolov8_msgs::InferenceResult :md5sum-) "953d486c7f880d6374bef7ecbdfb1210")
(setf (get yolov8_msgs::InferenceResult :datatype-) "yolov8_msgs/InferenceResult")
(setf (get yolov8_msgs::InferenceResult :definition-)
      "string class_name
int64 top
int64 left
int64 bottom
int64 right

")



(provide :yolov8_msgs/InferenceResult "953d486c7f880d6374bef7ecbdfb1210")


