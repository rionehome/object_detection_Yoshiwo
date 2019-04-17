#!/usr/bin/env python
import os
import sys

import rospy
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.4

######### Set the label map file here ###########
LABEL_NAME = 'mscoco_label_map.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]),'data','labels', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 90

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)





def callback(message):
    global label_map, categories, category_index
    count = 1
    #print message.detections  #list
    #print len(message.detections)
    detected_obj = message.detections[0]
    #print detected_obj.results  #list
    #print len(detected_obj.results)
    obj_result = detected_obj.results[0]
    print category_index[obj_result.id]['name']       # object name
    print "---------------"

def main(): 
    global label_map, categories, category_index
    rospy.init_node('object_sub')
    sub = rospy.Subscriber("/objects", Detection2DArray, callback)
    
    #print type(label_map)
    #print categories
    #print "---------------"
    #print category_index        # dict: {number: {id:, name:}}  number == id
    #print "------------------------------"

    rospy.spin()
    
if __name__=='__main__':
    main()

