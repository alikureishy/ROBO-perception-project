

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

```

```

### Perception Pipeline

```
robond@udacity:~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts$ ./pick_and_place.py --help
usage: pick_and_place.py [-h] -i INFILE -t TEST_SCENE -o OUTFILE

Perform advanced pick+place

optional arguments:
  -h, --help     show this help message and exit
  -i INFILE      Model file for the object recognition
  -t TEST_SCENE  Test scene number

```

As an example:
```
```

There are 6 stages to this perception pipeline, discussed below, with illustrations. I will be using illustrations mostly from the World # 3 (Test Case # 3) for this, since it involved 8 object types and was the hardest portion of this assignment.

#### RGB-D Camera View

Attached here is the original point cloud captured from the /prt/world/points topic.

#### Downsampling

This point cloud was massive -- 30+ MB -- which required downsampling because there was unnecessary overhead to the system and no real advantages over a downsampled version of the same.


#### Cleaning

The camera captured noisy data, as is obvious in the image in the 'downsampling' section.

#### Passthrough Filter

The next step is to extract only that part of the image that is relevant. For the purposes of this project, a passthrough filter had to be applied two times, as below:
- 'Z' axis: 
- 'H' axis: 

#### Segmentation

Now, with the sliced point cloud, the next step is to separate the table from the objects, for which we use RANSAC segmentatlon.

#### Clustering

### Classification & Labeling

## Debugging

```
robond@udacity:~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts$ ./capture_camera.py --help
usage: capture_camera.py [-h] -i INFILE -t TOPIC [-c COUNT]
                         [-l [LEVELS [LEVELS ...]]] -o OUTFOLDER [-p]

Perform advanced pick+place

optional arguments:
  -h, --help            show this help message and exit
  -i INFILE             Model file for the object recognition
  -t TOPIC              Name of topic to capture. ['/pr2/world/points', ....]
  -c COUNT              Number of times to snapshot
  -l [LEVELS [LEVELS ...]]
                        List of stages whose outputs will be saved to disk [0
                        = Original] [1 = Downsampled] [2 = Cleaned] [3 =
                        Sliced] [4 = Segmented] [5 = Object Cloud] [6 =
                        Detected Objects]
  -o OUTFOLDER          Folder where all the pipeline PCDs will be saved
  -p                    Whether to plot the feature histograms (default:
                        False)
```

As an example:

```
```

## Conclusions

Few things I learned from this exercise are:
- That the resolution of the point clouds used for training the classifier had to match (at least to some extent) the resolution of the deployment point clouds. So, the voxel downsampling that I was performing at the start of the perception pipeline (prior to cleaning, segmentation, feature extraction etc) had to also be applied to the training point clouds, prior to feature extraction. Keeping these consistent really improved the accuracy of the classifier.
- I had to switch to the _ExtraTreesClassifier_ for best results.
- In general, I only had to use between 10-50 samples for each object type, during training. There was no need to train with more data.
- I found that a combination of 3 types of features would be ideal for this classification:
-- RGB histogram: This is because color is an important component of identification (though not the only one)
-- HSV histogram: This is essential in the case where there are shadows and other lighting variation between the training and deployment captures.
-- Normals histogram: This captures the shape of the object, which is clearly crucial in determining any type of object.

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
