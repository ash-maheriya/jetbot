#!/usr/bin/env python2
#-########################################################################################
# Load a saved TF-TRT optimized model and run inference on given input image
#-########################################################################################

import cv2 as cv
import sys
import os
import tensorflow as tf
import tensorflow.contrib.tensorrt as trt
import numpy as np
import time


tf_sess = None
def setup():
    global tf_sess
    model = 'ssd_mnetv2_tennis'
    data = os.environ['HOME'] + '/work/exported_models/ssd_mnetv2_tennis.10-15.TENNIS-2019.10.14/'
    INPUTS = {'MODEL'           : model,
              'DATA_DIR'        : data}
    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True
    tf_sess = tf.Session(config=tf_config)
    with tf.gfile.GFile("{DATA_DIR}/{MODEL}_trt.pb".format(**INPUTS), 'rb') as f:
        trt_graph = tf.GraphDef()
        trt_graph.ParseFromString(f.read())
    
    tf.import_graph_def(trt_graph, name='')
    

    print("=== Setup complete!")
    
def inferenceBox(img):
    global tf_sess
    IMGSIZE   = (300, 300)    
    image  = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    image_resized = cv.resize(image, IMGSIZE, interpolation = cv.INTER_CUBIC)
    
    tf_input = tf_sess.graph.get_tensor_by_name('image_tensor:0')
    tf_boxes = tf_sess.graph.get_tensor_by_name('detection_boxes:0')
    tf_classes = tf_sess.graph.get_tensor_by_name('detection_classes:0')
    tf_scores = tf_sess.graph.get_tensor_by_name('detection_scores:0')
    tf_num_detections = tf_sess.graph.get_tensor_by_name('num_detections:0')

    _scores, _boxes, _classes, _num_detections = tf_sess.run([tf_scores, tf_boxes, tf_classes, tf_num_detections], 
                                                          feed_dict={tf_input: image_resized[None, ...]})
   
    box = _boxes[0][0]
    cv.rectangle(img, (int(box[1]*image.shape[1]), int(box[0]*image.shape[0])), (int(box[1]*image.shape[1]) + int(box[3]*image.shape[1]), int(box[0]*image.shape[0]) + int(box[2]*image.shape[1])), (0, 255, 0), 2)    

    print(box)
    cv.imshow("Image", img)
    cv.imshow("boxed", img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        cv.destroyAllWindows()
    return box    


#EOF
