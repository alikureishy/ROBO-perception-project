

# Robotics - Perception Pipeline

## Overview

This was a Udacity project, built along the lines of the Amazon Challenge, for detecting objects in a cluttered environment, and invoking one of two robot arms to pick up and drop the objects in its associated bin.

Note: The present version of the pipeline does not include the robot arm movement, but I hope to add it in at a later time. At present my goal was to complete this project and move on to the next one. 

## Components

This project has 3 parts:
- The simulation environment
- The training pipeline
- The perception pipeline

### Simulation Environment

### Training Pipeline

### Perception Pipeline

There are 6 stages to this perception pipeline.

#### RGB-D Camera View

#### Downsampling

#### Cleaning

#### Segmentation

#### Clustering

### Classification & Labeling


```
robond@udacity:> ./extract_features.py -i ~/data/library/ -y ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/config/pick_list_2.yaml -c 30 -o ~/data/features/p2_c32_h32_n30.features

Reading object types from: /home/robond/catkin_ws/src/RoboND-Perception-Project/pr2_robot/config/pick_list_2.yaml
YAML contained 5 models:
[{'group': 'green', 'name': 'biscuits'}, {'group': 'green', 'name': 'soap'}, {'group': 'red', 'name': 'book'}, {'group': 'red', 'name': 'soap2'}, {'group': 'red', 'name': 'glue'}]

Calculating sample availability:
	biscuits: 1000
	soap: 1000
	book: 1000
	soap2: 1000
	glue: 1000
Total available sample count: 5000
0: Model: biscuits. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/biscuits/pc_biscuits_812.pcd ...
	1: Processing /home/robond/data/library/biscuits/pc_biscuits_198.pcd ...
	2: Processing /home/robond/data/library/biscuits/pc_biscuits_861.pcd ...
  ...
  27: Processing /home/robond/data/library/biscuits/pc_biscuits_259.pcd ...
	28: Processing /home/robond/data/library/biscuits/pc_biscuits_467.pcd ...
	29: Processing /home/robond/data/library/biscuits/pc_biscuits_738.pcd ...
1: Model: soap. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/soap/pc_soap_396.pcd ...
	1: Processing /home/robond/data/library/soap/pc_soap_705.pcd ...
	2: Processing /home/robond/data/library/soap/pc_soap_749.pcd ...
  ...
  27: Processing /home/robond/data/library/soap/pc_soap_143.pcd ...
	28: Processing /home/robond/data/library/soap/pc_soap_317.pcd ...
	29: Processing /home/robond/data/library/soap/pc_soap_248.pcd ...
2: Model: book. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/book/pc_book_151.pcd ...
	1: Processing /home/robond/data/library/book/pc_book_499.pcd ...
	2: Processing /home/robond/data/library/book/pc_book_2.pcd ...
  ...
	27: Processing /home/robond/data/library/book/pc_book_86.pcd ...
	28: Processing /home/robond/data/library/book/pc_book_990.pcd ...
	29: Processing /home/robond/data/library/book/pc_book_1.pcd ...
3: Model: soap2. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/soap2/pc_soap2_14.pcd ...
	1: Processing /home/robond/data/library/soap2/pc_soap2_269.pcd ...
	2: Processing /home/robond/data/library/soap2/pc_soap2_222.pcd ...
  ...
	27: Processing /home/robond/data/library/soap2/pc_soap2_250.pcd ...
	28: Processing /home/robond/data/library/soap2/pc_soap2_761.pcd ...
	29: Processing /home/robond/data/library/soap2/pc_soap2_465.pcd ...
4: Model: glue. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/glue/pc_glue_492.pcd ...
	1: Processing /home/robond/data/library/glue/pc_glue_509.pcd ...
	2: Processing /home/robond/data/library/glue/pc_glue_974.pcd ...
  ...
	27: Processing /home/robond/data/library/glue/pc_glue_344.pcd ...
	28: Processing /home/robond/data/library/glue/pc_glue_17.pcd ...
	29: Processing /home/robond/data/library/glue/pc_glue_515.pcd ...
Storing features in file: /home/robond/data/features/p2_c32_h32_n30.features
Feature extraction complete!
```

```
object_list:
- arm_name: left
  object_name: sticky_notes
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.43974432349205017
      y: 0.21434171497821808
      z: 0.6919182538986206
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: 0.71
      z: 0.605
  test_scene_num: 3
- arm_name: left
  object_name: book
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.49164295196533203
      y: 0.08344084769487381
      z: 0.7354593276977539
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: 0.71
      z: 0.605
  test_scene_num: 3
- arm_name: right
  object_name: snacks
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.4238100051879883
      y: -0.32212504744529724
      z: 0.7633553147315979
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: -0.71
      z: 0.605
  test_scene_num: 3
- arm_name: right
  object_name: biscuits
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.5885060429573059
      y: -0.21873414516448975
      z: 0.7125846147537231
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: -0.71
      z: 0.605
  test_scene_num: 3
- arm_name: left
  object_name: eraser
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.6121700406074524
      y: 0.28314337134361267
      z: 0.6523650884628296
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: 0.71
      z: 0.605
  test_scene_num: 3
- arm_name: right
  object_name: soap2
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.4536884129047394
      y: -0.04403987526893616
      z: 0.683113157749176
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: -0.71
      z: 0.605
  test_scene_num: 3
- arm_name: right
  object_name: soap
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.6805647015571594
      y: 0.0030703614465892315
      z: 0.6835725903511047
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: -0.71
      z: 0.605
  test_scene_num: 3
- arm_name: left
  object_name: glue
  pick_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0.6127251982688904
      y: 0.13942934572696686
      z: 0.6910355687141418
  place_pose:
    orientation:
      w: 0
      x: 0
      y: 0
      z: 0
    position:
      x: 0
      y: 0.71
      z: 0.605
  test_scene_num: 3
```