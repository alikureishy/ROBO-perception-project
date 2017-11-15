
![PR2 Perception Project](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p3_scene.png)

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

#### Sample Collection

```
robond@udacity:~/catkin_ws/src/sensor_stick/scripts$ ./capture_point_clouds.py --help
usage: capture_point_clouds.py [-h] -y YAML -c COUNT -o OUTFOLDER [-t TOPIC]

Capture point clouds for feature extraction

optional arguments:
  -h, --help    show this help message and exit
  -y YAML       YAML file with model names
  -c COUNT      Num of variations (default: 10)
  -o OUTFOLDER  Folder to save .pcd files for point cloud samples
  -t TOPIC      Topic to which to publish the sampled point clouds
```


#### Feature Extraction

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

#### Training

```
robond@udacity:~/catkin_ws/src/sensor_stick/scripts$ ./train_svm.py --help
usage: train_svm.py [-h] -i INFILE -o OUTFILE [-p]

Capture point clouds for feature extraction

optional arguments:
  -h, --help  show this help message and exit
  -i INFILE   Pickle file with point cloud features
  -o OUTFILE  File to store the model into
  -p          Whether to plot the confusion matrix (default: False)

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
  -o OUTFILE     YAML file to save the generated pick+place request sequence
```

As an example:
```
```

The different pipeline stages below are implemented here. The overall pipeline is implemented here.

There are 6 stages to this perception pipeline, discussed below, with illustrations. I will be using illustrations mostly from the World # 3 (Test Case # 3) for this, since it involved 8 object types and was the hardest portion of this assignment.

#### RGB-D Camera View

Attached here is the original point cloud captured from the /prt/world/points topic.

![PR2 Camera View](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p3_camera_view.png)

#### Downsampling

This point cloud was massive -- 30+ MB -- which required downsampling because there was unnecessary overhead to the system and no real advantages over a downsampled version of the same.

See next section for the output image.

#### Cleaning

The camera captured noisy data, as is obvious in the image in the 'downsampling' section.

Here's the output of the same table after downsampling and cleaning:
![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/cleaned.png)

#### Passthrough Filter

The next step is to extract only that part of the image that is relevant. For the purposes of this project, a passthrough filter had to be applied two times, as below:
- 'Z' axis: To eliminate any noise from above and below the table
- 'X' axis: To eliminate the interference from the sides (for example, the two colored 'objects' on the left and right sides)

This gave me just what I needed for the rest of my pipeline:
![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/sliced.png)

#### Segmentation

Now, with the sliced point cloud, the next step is to separate the table from the objects, for which we use RANSAC segmentatlon.

* Segmented Table

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/segmented_table_pc.png)

* Segmented Non-Table-Stuff (Targets)

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/segmented_objects_pc.png)

#### Clustering

This pipeline uses Euclidian Distance clustering, and produces the following object detections (as seen in the World # 3 test case):

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/object_list.png)

### Classification & Labeling

#### Classification

This involved reading the model from the provided model file, obtaining the classifier, the label encoder, and the scaler, all of which are needed for proper classification.

## Debugging

The importance of being able to debug the point clouds obtained during training, and then those obtained at each point of the pipeline, cannot be overstated. At first I decided to just publish each of these point clouds to different topics, so that I could view what each stage was outputing through RViz's PointCloud2 visualizer. However, I found this to be a hassle for two reasons:
- Because my computer was too slow to be able to be able to smoothly switch the PointCloud2 visualizer's topic between these topics, and
- Because I wanted to be able to inspect the point clouds at each stage in more detail than RViz would allow (also because of the processor limitations above).

So, I wrote a tool -- _capture_camera.py_ -- that would essentially output the following point clouds:
- Original point cloud (received straight from the /pr2/world/points topic)
- Downsampled PC
- Cleaned PC
- Sliced PCs (for each passthrough filter that I applied)
- Segmented Inliers and Outliers (Table + Remaining items)
- Clustered Objecs (color-coded, after clustering)
- Each clustered object (these are what the classifier was classifying)

The tool's help menu is below, as an illustration:

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

## Results

I was able to achieve 100% accuracy of detection, for all 3 worlds, as illustrated below.

### World 1

This world had only 3 objects:
```
object_list:
  - name: biscuits
    group: green
  - name: soap
    group: green
  - name: soap2
    group: red
```

Training parameters:
- 20 samples for each object type
- Features = [32 bins for RGB, 32 bins for HSV, 32 bins for Surface Normals]
- Classifier: ExtraTreesClassifier (Though Linear SVM with 'rbf' kernel sufficed here due to small number of categories)

#### Confusion Matrix

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p1_training_20.png)

#### Labeled Output

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p1_labeled.png)


### World 2

This world had 5 objects:
```
object_list:
  - name: biscuits
    group: green
  - name: soap
    group: green
  - name: book
    group: red
  - name: soap2
    group: red
  - name: glue
    group: red
```

Training parameters:
- 30 samples for each object type
- Features = [32 bins for RGB, 32 bins for HSV, 32 bins for Surface Normals]
- Classifier: ExtraTreesClassifier

#### Confusion Matrix

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p2_training.png)

#### Labeled Output

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p2_labeled.png)

### World 3

This world had 8 objects:
```
object_list:
  - name: sticky_notes
    group: red
  - name: book
    group: red
  - name: snacks
    group: green
  - name: biscuits
    group: green
  - name: eraser
    group: red
  - name: soap2
    group: green
  - name: soap
    group: green
  - name: glue
    group: red
```

Training parameters:
- 50 samples for each object type
- Features = [32 bins for RGB, 32 bins for HSV, 32 bins for Surface Normals]
- Classifier: ExtraTreesClassifier (Though Linear SVM with 'rbf' kernel sufficed here due to small number of categories)

#### Confusion Matrix

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p3_training.png)

#### Labeled Output

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p3_labeled.png)

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
