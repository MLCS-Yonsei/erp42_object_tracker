# erp42_object_tracker
AI_racing competition (erp42) LiDAR tracking library, for tracking objects obtaining from segmentation-based detection and improve segmentation.

## HowToUse

### Installation
```bash
$ cd catkin_ws/src
$ git clone https://github.com/sjnah/erp42_object_tracker.git
$ git submodule update --init --recurisve
$ git submodule foreach git pull origin master
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release --only-pkg-with-deps costmap_converter
$ catkin_make -DCMAKE_BUILD_TYPE=Release --only-pkg-with-deps segmenters_lib
$ catkin_make -DCMAKE_BUILD_TYPE=Release --only-pkg-with-deps erp42_object_tracker_node
```

### Run
```bash
$ roslaunch erp42_object_tracker_node runTracker.launch
```

## Parameters
