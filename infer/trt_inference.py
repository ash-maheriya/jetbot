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


debug = False;
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
    t0 = time.time();
    global tf_sess
    IMGSIZE   = (300, 300)    
    image  = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    image_resized = cv.resize(image, IMGSIZE, interpolation = cv.INTER_CUBIC)
    
    time_till_image_alteration = time.time() - t0
    
    tf_input = tf_sess.graph.get_tensor_by_name('image_tensor:0')
    tf_boxes = tf_sess.graph.get_tensor_by_name('detection_boxes:0')
    tf_classes = tf_sess.graph.get_tensor_by_name('detection_classes:0')
    tf_scores = tf_sess.graph.get_tensor_by_name('detection_scores:0')
    tf_num_detections = tf_sess.graph.get_tensor_by_name('num_detections:0')

    _scores, _boxes, _classes, _num_detections = tf_sess.run([tf_scores, tf_boxes, tf_classes, tf_num_detections], 
                                                          feed_dict={tf_input: image_resized[None, ...]})

    num_detections = 4
    boxes = _boxes[0][0:num_detections]
    classes = _classes[0][0:num_detections]
    scores = _scores[0][0:num_detections]

    # Visualize one bounding box and print out the time each process takes
    if (debug):
        ymin, xmin, ymax, xmax = boxes[0]
        
        img_width = img.shape[1]
        img_height = img.shape[0]
        
        print("img_width: {w}, img_height: {h}".format(h=img_height, w=img_width))
        
        time_till_inference = time.time() - t0
        inference_length = time_till_inference - time_till_image_alteration
        cv.rectangle(img, (int(xmin*img_width), int(ymin*img_height)), (int(xmax*img_width), int(ymax*img_height)), (0, 255, 0), 2) 
            
        print("box: " + str(boxes[0]))
        cv.imshow("Image", img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
        time_till_display = time.time() - t0
        display_length = time_till_display - time_till_inference
        
        total_time = time.time() - t0;
        print("Image alteration length: " + str(time_till_image_alteration))
        print("Inference length: " + str(inference_length))
        print("Display length: " + str(display_length))
        print("Total time: " + str(total_time))
    
    return boxes, classes, scores, num_detections    


#EOF
