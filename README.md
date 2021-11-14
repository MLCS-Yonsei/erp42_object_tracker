# erp42_object_tracker
AI_racing erp42_object_tracker

## Installation
```bash
$ cd catkin_ws/src
$ git clone https://github.com/sjnah/erp42_object_tracker.git
$ git submodule update --init --recurisve
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Run
```bash
$ roslaunch erp42_object_tracker_node runTracker.launch
```