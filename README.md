
![Overview](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/p3_scene.png)

# Robotics - Perception Pipeline

## Table of Contents

- [Overview](#overview)
- [Components](#components)
	- [Simulation Environment](#simulation-environment)
	- [Training Pipeline](#training-pipeline)
		- [Sample Collection](#sample-collection)
		- [Feature Extraction](#feature-extraction)
		- [Training](#training)
	- [Perception Pipeline](#perception-pipeline)
		- [RGBD Camera View](#rgbd-camera-view)
		- [Downsampling](#downsampling)
		- [Cleaning](#cleaning)
		- [Passthrough Filter](#passthrough-filter)
		- [Segmentation](#segmentation)
		- [Clustering](#clustering)
		- [Classification](#classification)
		- [Labeling](#labeling)
- [Debugging](#debugging)
- [Results](#results)
	- [World 1](#world-1)
	- [World 2](#world-2)
	- [World 3](#world-3)
- [Conclusions](#conclusions)

## Overview

This was a Udacity project, built along the lines of the [Amazon Robotics Challenges](https://www.amazonrobotics.com/#/roboticschallenge/past-challenges), for detecting objects in a cluttered environment, and invoking one of two robot arms to pick up and drop the objects in its associated bin.

Note: The present version of the pipeline does not include the robot arm movement, but I hope to add it in at a later time. The immediate goal was to focus on the _perception_ portion of the project. 

## Components

This project has 3 parts:
- The simulation environment
- The training pipeline
- The perception pipeline

### Simulation Environment

This project was run on the ROS platform, and ran the simulation using Gazebo.

![Gazebo](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/simulation_gazebo.png)

### Training Pipeline

Traning involves collecting samples, extracting features, and then training a classifier on those features. For efficiency, it seemed best to separate these 3 pipeline stages into separate tools.

#### Sample Collection

This would issue gazebo requests to sample each of the objects in a certain number of random orientations and positions, in order to take a snapshot to use as the reference point cloud for that object type. I created a library of a 1000 such reference point clouds for each of the 15 objects covered in this project, each categorized under a folder named after that object as shown below:

```
robond@udacity:~/data/library$ ls -al
total 10324
drwxrwxr-x 17 robond robond    4096 Nov 11 21:15 .
drwxr-xr-x  7 robond robond    4096 Nov 14 13:36 ..
drwxrwxr-x  2 robond robond   36864 Nov 10 02:37 beer
drwxrwxr-x  2 robond robond   36864 Nov 10 13:12 biscuits
drwxrwxr-x  2 robond robond   36864 Nov 10 12:50 book
drwxrwxr-x  2 robond robond   36864 Nov 10 02:50 bowl
drwxrwxr-x  2 robond robond   36864 Nov 10 03:04 create
drwxrwxr-x  2 robond robond   36864 Nov 10 03:16 disk_part
drwxrwxr-x  2 robond robond   45056 Nov 10 13:21 eraser
drwxrwxr-x  2 robond robond   36864 Nov 10 13:44 glue
drwxrwxr-x  2 robond robond   36864 Nov 10 03:26 hammer
drwxrwxr-x  2 robond robond   57344 Nov 10 03:34 plastic_cup
drwxrwxr-x  2 robond robond   36864 Nov 10 13:00 snacks
drwxrwxr-x  2 robond robond   36864 Nov 10 13:37 soap
drwxrwxr-x  2 robond robond   36864 Nov 10 13:29 soap2
drwxrwxr-x  2 robond robond   36864 Nov 11 09:28 soda_can
drwxrwxr-x  2 robond robond   49152 Nov 10 12:39 sticky_notes
```

The [tool](https://github.com/safdark/ROBO-perception-project/blob/master/sensor_stick/scripts/capture_point_clouds.py) to capture these samples was:
```
$> ./capture_point_clouds.py --help
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

The next stage was feature extraction, which involved the following pipeline for each point cloud:
- Downsampling
- Extract (and concatenate together):
	- Histogram of 32 bins for each of the R, G and B channels
		- This is because color is an important component of identification (though not the only one)
	- Histogram of 32 bins for each of the H, S and V channels
		- This is essential in the case where there are shadows and other lighting variation between the training and deployment captures.
	- Histogram of 32 bins for the surface normals for the X, Y and Z axes
		- This captures the shape of the object, which is clearly crucial in determining any type of object.

The final feature array for each point cloud, therefore, would contain 32 * 9 = 288 floats, as depicted in this image here:
![Histogram](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/histogram_example.png)

Feature extraction is triggered using this [tool](https://github.com/safdark/ROBO-perception-project/blob/master/sensor_stick/scripts/extract_features.py):
```
$> ./extract_features.py --help
usage: extract_features.py [-h] -i INFOLDER -y YAML -c COUNT -o OUTFILE [-p]

Capture point clouds for feature extraction

optional arguments:
  -h, --help   show this help message and exit
  -i INFOLDER  Root folder containing model-based sub-folders of point cloud
               samples
  -y YAML      YAML file with model names
  -c COUNT     Num of variations of each model to process (default: 10)
  -o OUTFILE   Pickle file to save extracted features
  -p           Whether to plot the feature histograms (default: False)

```

As an example:
```
$> ./extract_features.py -i ~/data/library/ -y ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/config/pick_list_2.yaml -c 30 -o ~/data/features/p2_c32_h32_n30.features

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
  ...
	29: Processing /home/robond/data/library/biscuits/pc_biscuits_738.pcd ...
1: Model: soap. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/soap/pc_soap_396.pcd ...
  ...
	29: Processing /home/robond/data/library/soap/pc_soap_248.pcd ...
2: Model: book. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/book/pc_book_151.pcd ...
  ...
	29: Processing /home/robond/data/library/book/pc_book_1.pcd ...
3: Model: soap2. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/soap2/pc_soap2_14.pcd ...
  ...
	29: Processing /home/robond/data/library/soap2/pc_soap2_465.pcd ...
4: Model: glue. Avaiable samples 1000. Selecting 30...
	0: Processing /home/robond/data/library/glue/pc_glue_492.pcd ...
  ...
	29: Processing /home/robond/data/library/glue/pc_glue_515.pcd ...
Storing features in file: /home/robond/data/features/p2_c32_h32_n30.features
Feature extraction complete!
```

#### Training

Finally, the extracted features from above could be independently trained on, using various types of classifiers. This was the advantage of separating the training pipeline into these stages. Once the features were generated for all the samples for a given world, I just had to tweak the classifier hyperparams to see which one performed best, without having to alter any of the previous stage outputs.

Here is the [tool](https://github.com/safdark/ROBO-perception-project/blob/master/sensor_stick/scripts/train_svm.py) for training the SVM from the features extracted above.

```
$> ./train_svm.py --help
usage: train_svm.py [-h] -i INFILE -o OUTFILE [-p]

Capture point clouds for feature extraction

optional arguments:
  -h, --help  show this help message and exit
  -i INFILE   Pickle file with point cloud features
  -o OUTFILE  File to store the model into
  -p          Whether to plot the confusion matrix (default: False)

```

As an example:
```
$> ./train_svm.py -i ~/data/features/p3_d0_003_c32_h32_n32_c50.features -o ~/data/models/p3_etc_d0_003_c32_h32_n32_c50.model -p
Loading training set...
Features in Training Set: 400
Invalid Features in Training set: 0
Splitting train/test data..
Scaling feature columns...
Performing cross validation on classifier
Scores: [ 1.          0.975       0.975       0.975       0.97468354]
Accuracy: 0.98 (+/- 0.02)
accuracy score: 0.977443609023
Training the classifier...
Accuracy with held-out test-set...
Prediction: [4] / Actual: 4
Feature ranking:			<-- Ranking (desc) of the top 20 most useful bins in the histograms
	1. feature 102 (0.028090)
	2. feature 155 (0.025194)
	3. feature 100 (0.023304)
	4. feature 101 (0.022612)
	5. feature 32 (0.022203)
	6. feature 127 (0.022073)
	7. feature 150 (0.019617)
	8. feature 116 (0.018971)
	9. feature 159 (0.017219)
	10. feature 158 (0.016200)
	11. feature 115 (0.015090)
	12. feature 109 (0.014815)
	13. feature 156 (0.014782)
	14. feature 157 (0.014092)
	15. feature 137 (0.013967)
	16. feature 64 (0.013903)
	17. feature 138 (0.013400)
	18. feature 98 (0.013236)
	19. feature 99 (0.012634)
	20. feature 33 (0.011953)
Saving classifier to disk...
```

Note: See the Results section for the confusion matrices obtained for the 3 different worlds.

### Perception Pipeline

The 'pick_and_place.py' utility performs the perception pipeline, consisting of the following stages:
- [RGBD Camera View](#rgbd-camera-view)
- [Downsampling](#downsampling)
- [Cleaning](#cleaning)
- [Passthrough Filter](#passthrough-filter)
- [Segmentation](#segmentation)
- [Clustering](#clustering)
- [Classification](#classification)
- [Labeling](#labeling)

```
$> ./pick_and_place.py --help
usage: pick_and_place.py [-h] -i INFILE -t TEST_SCENE -o OUTFILE

Perform advanced pick+place

optional arguments:
  -h, --help     show this help message and exit
  -i INFILE      Model file for the object recognition
  -t TEST_SCENE  Test scene number
  -o OUTFILE     YAML file to save the generated pick+place request sequence
```

The different pipeline stages below are implemented [here](https://github.com/safdark/ROBO-perception-project/blob/master/sensor_stick/src/sensor_stick/pipeline.py).

The script to do the pick+place operation using the pipeline methods discussed below, is:

```
    image, _ = downsampled, latency = downsample(image, leaf_ratio=0.003)
    image, _ = cleaned, latency = clean(image, mean_k=50, std_dev_mul_thresh=1.0)
    image, _ = sliced1, latency = slice(image, field_name='z', limits=[0.6,1.5])
    image, _ = sliced2, latency = slice(image, field_name='y', limits=[-0.4,0.4])
    inliers, latency = segmentize(image, distance_thresh=0.025)
    table_cloud, non_table_cloud, latency = separate_segments(image, inliers)
    objects_cloud, clusters, _, latency = clusterize_objects(non_table_cloud, cluster_tolerance=0.04, min_size=200, max_size=5000, debug=False)
    ...
    ...
    detections, markers, object_clouds, latency = classify_objects(clusters, non_table_cloud, classifier, encoder, scaler)
```

There are 8 stages to this perception pipeline, discussed below, with illustrations. I will be using illustrations mostly from the World # 3 (Test Case # 3) for this, since it involved 8 object types and was the hardest portion of this assignment.

#### RGBD Camera View

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

* Segmented Table (Inlier points)

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/segmented_table_pc.png)

* Segmented Non-Table-Stuff (Outlier points - with all the target objects)

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/segmented_objects_pc.png)

#### Clustering

Euclidian Distance clustering was used here to produces the following object detections (as seen in the World # 3 test case):

![PR2 Colored Objects](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/colored_objects.png)

The coloring indicates the unique clusters that had been discovered -- since each cluster has been assigned a unique color here for this generated point cloud. This figure here shows 8 clusters having clearly been identified in world # 3.

#### Classification

This involved reading the model from the provided model file, obtaining the classifier, the label encoder, and the scaler, all of which are needed for proper classification. The points for each clustered object above were then used to extract the point cloud of the actual object, using the shared feature-extraction method(s) available [here](https://github.com/safdark/ROBO-perception-project/blob/master/sensor_stick/src/sensor_stick/features.py), from the segmented point cloud, to obtain the individual objects' point clouds, as shown here:

![PR2 Object List](https://github.com/safdark/ROBO-perception-project/blob/master/docs/images/object_list.png)

Then, the same features would be extracted for these clustered objects as was done for the training samples, to generate a 288-bin histogram, which was then used in the classification, to obtain the corresponding label.

Each detected object's centroid was also determined, using the cluster points associated with it, across the x, y and z axes.

#### Labeling

With the labels and centroids known, the pickable-object list (ROS config param: '/object_list') was looked up to collect the group associated with each label. This group was then used to lookup the corresponding bucket/arm combination (ROS config param: '/dropbox') for where the bucket into which it had to be placed, and a corresponding list of handling instructions were generated as yaml, as pictured (for example) below:

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
```

Another list of detected objects (containing the label) was published to the '/detected_objects' ROS topic, so that it could be displayed on RViz using the 'Marker' Visualizer there.


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
$> ./capture_camera.py --help
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

Here's a sample output from this utility:
```
$> ./capture_camera.py -i ~/data/models/p1_etc_d0_003_c32_h32_n32_c20.model -t /pr2/world/points -c 1 -l 0 1 2 3 4 5 6 -o ./p1
Namespace(count=3, infile='/home/robond/data/models/p1_etc_d0_003_c32_h32_n32_c20.model', levels=[0, 1, 2, 3, 4, 5, 6], outfolder='./p1', plot=False, topic='/pr2/world/points')
1: Received capture from topic: /pr2/world/points
	Deserialization: 2.94999313354 seconds
	Downsampling: 0.259364843369 seconds
	Cleaning: 4.6358230114 seconds
	Passthrough: 0.00532102584839 seconds
	Passthrough: 0.000747919082642 seconds
	Ransac: 0.0131590366364 seconds
	Extraction: 0.00392293930054 seconds
	Clusterizing: 0.170705795288 seconds
	Classification: 2.14816594124 seconds
	Found 3 objects: ['biscuits', 'soap', 'soap2']
```

The OUTFOLDER path will then contain a folder hierarchy containing debug point-clouds as below:
```
<OUTFOLDER>
	\<1>				--> <Frame#>
		\0_<object>.pcd		--> Indexed list of recognized object point clouds (e.g, 0_biscuits.pcd)
		\1_<object>.pcd
		...
		\original.pcd		--> Original point cloud (received from the topic)
		\downsampled		--> Downsampled point cloud
		\cleaned.pcd		--> Cleaned point cloud
		\slice1.pcd		--> Point cloud after the vertical slicing ('Z' axis)
		\slice2.pcd		--> Point cloud after the horizontal slicing ('X' axis)
		\table.pcd		--> Segmented point cloud of the table
		\non-table.pcd		--> Segmented point cloud of everything else (except the table)
		\objects.pcd		--> Color-coded view of the clustered objects from non-table.pcd
	\<2>
		...
	\<3>
		...
```

See [here](https://github.com/safdark/ROBO-perception-project/tree/master/RoboND-Perception-Project/pr2_robot/scripts/launches/runs) for the flattened point cloud (debug) objects for each of the 3 worlds, under this hierarchy:
```
	\runs
		\p1			--> World # 1 (with the list of debug objects listed above under '<1>')
		\p2			--> World # 2			"
		\p3			--> World # 3			"
```

## Results

I was able to achieve 100% accuracy of detection, for all 3 worlds, as illustrated below.

### World 1

Accuracy: 100% (3/3 objects labeled correctly)

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

#### Generated YAML Instructions & Other Debug Info

See generated pick+place 'commands' file [here](https://github.com/safdark/ROBO-perception-project/blob/master/RoboND-Perception-Project/pr2_robot/scripts/launches/runs/p1/p1_commands.yaml).

Debug point clouds [here](https://github.com/safdark/ROBO-perception-project/tree/master/RoboND-Perception-Project/pr2_robot/scripts/launches/runs/p1).

### World 2

Accuracy: 100% (5/5 objects labeled correctly)

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

#### Generated YAML Instructions & Other Debug Info

See generated pick+place 'commands' file [here](https://github.com/safdark/ROBO-perception-project/blob/master/RoboND-Perception-Project/pr2_robot/scripts/launches/runs/p2/p2_commands.yaml).

Debug point clouds [here](https://github.com/safdark/ROBO-perception-project/tree/master/RoboND-Perception-Project/pr2_robot/scripts/launches/runs/p2).

### World 3

Accuracy: 100% (8/8 objects labeled correctly)

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

#### Generated YAML Instructions & Other Debug Info

See generated pick+place 'commands' file [here](https://github.com/safdark/ROBO-perception-project/blob/master/RoboND-Perception-Project/pr2_robot/scripts/launches/runs/p3/p3_commands.yaml).

Debug point clouds [here](https://github.com/safdark/ROBO-perception-project/tree/master/RoboND-Perception-Project/pr2_robot/scripts/launches/runs/p3).

## Conclusions

Few things I learned from this exercise are:
- That the resolution of the point clouds used for training the classifier had to match (at least to some extent) the resolution of the deployment point clouds. So, the voxel downsampling that I was performing at the start of the perception pipeline (prior to cleaning, segmentation, feature extraction etc) had to also be applied to the training point clouds, prior to feature extraction. Keeping these consistent really improved the accuracy of the classifier.
- I had to switch to the _ExtraTreesClassifier_ for best results.
- In general, I only had to use between 10-50 samples for each object type, during training. There was no need to train with more data.
