#! /usr/bin/env python
# -*- coding: utf-8 -*-

#使用方法
"""
必要ファイル
libdarknet.soをdarknet.pyと同じディレクトリに配置
.dataファイルと.namesファイルをプログラム内で指定（オリジナルデータを使う際はまた異なるファイルが必要なのでそのときは声をかけて）
.cfgファイルと.weight（学習の重みファイル）をプログラム内で指定
"""

#画像データ
"""
data/ディレクトリ
に認識対象の画像を格納
USBcameraなどで取得した画像をここに格納
"""
#認識結果
"""
result/ディレクトリに格納される
"""

#実行方法
"""
python darknet.py
認識対象画像はプログラム内で指定
認識結果等もプログラム内で指定 
"""

from ctypes import *
import math
import random
import sys
import cv2


def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1

def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr

class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]

class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]

class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]

    

#lib = CDLL("/home/yoshiwo/darknet/libdarknet.so", RTLD_GLOBAL)

#ここで[libdarknet.so]ファイルを指定しないと動かない"./libdarknet.so"
lib = CDLL("/home/yoshiwo/catkin_ws/src/yoshiwo_pivate_lesson/darknet_python/libdarknet.so", RTLD_GLOBAL)
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

set_gpu = lib.cuda_set_device
set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)

def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        res.append((meta.names[i], out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res

def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
    im = load_image(image, 0, 0)
    num = c_int(0)
    pnum = pointer(num)
    predict_image(net, im)
    dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, None, 0, pnum)
    num = pnum[0]
    if (nms): do_nms_obj(dets, num, meta.classes, nms);

    res = []
    for j in range(num):
        for i in range(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                left, right = int((b.x-b.w/2.0)),int((b.x+b.w/2.0))
                top,bottom = int(b.y-b.h/2.0),int(b.y+b.h/2.0)
                res.append((meta.names[i], dets[j].prob[i], (left,right,top,bottom)))
    
    #left, right = int((b.x-b.w/2.0)),int((b.x+b.w/2.0))
    #top,bottom = int(b.y-b.h/2.0),int(b.y+b.h/2.0)
    #print(left,right,top,bottom)
    #top, bottom = 214, 542
    
    res = sorted(res, key=lambda x: -x[1])
    free_image(im)
    free_detections(dets, num)
    return res
    
if __name__ == "__main__":
    #net = load_net("cfg/densenet201.cfg", "/home/pjreddie/trained/densenet201.weights", 0)
    #im = load_image("data/wolf.jpg", 0, 0)
    #meta = load_meta("cfg/imagenet1k.data")
    #r = classify(net, meta, im)
    #print r[:10]
    
    #.cfgファイルを指定
    #.weightsを指定
    net = load_net("/home/yoshiwo/catkin_ws/src/yoshiwo_pivate_lesson/darknet_python/yolov3-tiny.cfg", "/home/yoshiwo/catkin_ws/src/yoshiwo_pivate_lesson/darknet_python/yolov3-tiny.weights", 0)
    
    #.dataファイルを指定
    meta = load_meta("/home/yoshiwo/catkin_ws/src/yoshiwo_pivate_lesson/darknet_python/coco.data")
    
    #認識対象の画像を指定
    r = detect(net, meta, "/home/yoshiwo/catkin_ws/src/yoshiwo_pivate_lesson/darknet_python/data/dog.jpg")
    
    #認識結果を  r  に格納
    print(r)
    """
    rは多次元配列
    例：
    r = ("物体名",予測値,(left,right,top,bottom))
    r[0] = [('car', 0.6152912378311157, (465, 679, 71, 169))]
    r[0][0] = "car"
    r[2][0][0] = 465
    
    """
    print("---------")
    img = cv2.imread("/home/yoshiwo/catkin_ws/src/yoshiwo_pivate_lesson/darknet_python/data/dog.jpg")
    for i in range(len(r)):
        im2 = img[r[i][2][2]:r[i][2][3], r[i][2][0]:r[i][2][1]]
        #認識結果をresultディレクトリに指定
        cv2.imwrite("/home/yoshiwo/catkin_ws/src/yoshiwo_pivate_lesson/darknet_python/result/" + str(r[i][0])+".jpg", im2)
    
    

