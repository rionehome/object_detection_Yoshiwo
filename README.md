## Object Detection

1. install camera dependencies
```
$ sudo apt-get install ros-kinetic-usb_cam ros-kinetic-openni2-launch
```

2. Install tensorflow into python virtualenv
```
$ sudo apt-get install python-pip python-dev python-virtualenv
$ virtualenv --system-site-packages ~/tensorflow
$ source ~/tensorflow/bin/activate
$ easy_install -U pip
$ pip install --upgrade tensorflow
```

3. setup 
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/rionehome/object_detection.git
$ cd ./object_detection
$ git clone https://github.com/Kukanani/vision_msgs.git
$ git clone https://github.com/osrf/tensorflow_object_detector.git
```

4. run
```
$ source ~/tensorflow/bin/activate && source ~/catkin_ws/devel/setup.bash
$ roslaunch tensorflow_object_detector usb_cam_detector.launch
$ source ~/tensorflow/bin/activate && source ~/catkin_ws/devel/setup.bash
$ rosrun sub_obj_detector object_sub.py
```