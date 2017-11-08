# Robotics - Perception Project

## Command Line Utilities

* Generating features:
roslaunch sensor_stick training.launch
rosrun sensor_stick capture_features.py

* Training SVM:
rosrun sensor_stick train_svm.py

* Object recognition (sensor_stick):
roslaunch sensor_stick robot_spawn.launch
./object_recognition.py

* Object recognition (PR2) - DEMO:
./pr2_safe_spawner.sh

* Object recognition (PR2):
roslaunch pr2_robot pick_place_project.launch
rosrun pr2_robot project_template.py

## Training Runs

* 20 iterations / hhists(32,0,256)+nhists(32,0,256) / rbf:
Scores: [ 0.89285714  0.82142857  0.75        0.85714286  0.75      ]
Accuracy: 0.81 (+/- 0.11)
accuracy score: 0.814285714286

* 20 iterations / chists(32,0,256)+nhists(32,-1,1) / rbf
Scores: [ 0.75        0.57142857  0.92857143  0.85714286  0.78571429]
Accuracy: 0.78 (+/- 0.24)
accuracy score: 0.778571428571

* 20 iterations / chists(32,0,256)+hhists(32,0,256)+nhists(32,-1,1) / rbf:
Scores: [ 0.96428571  0.75        0.71428571  0.78571429  0.78571429]
Accuracy: 0.80 (+/- 0.17)
accuracy score: 0.8

* 20 iterations / hhists(32,0,256)+nhists(32,-1,1) / rbf:
Scores: [ 0.85714286  0.67857143  0.85714286  0.92857143  0.85714286]
Accuracy: 0.84 (+/- 0.17)
accuracy score: 0.835714285714

* 30 iterations / hhists(32,0,256)+nhists(32,-1,1) / rbf:
Scores: [ 0.85714286  0.88095238  0.88095238  0.83333333  0.85714286]
Accuracy: 0.86 (+/- 0.04)
accuracy score: 0.861904761905

* 30 iterations / chists(32,0,256)+hhists(32,0,256)+nhists(32,-1,1) / rbf:
Scores: [ 0.92857143  0.85714286  0.9047619   0.85714286  0.88095238]
Accuracy: 0.89 (+/- 0.06)
accuracy score: 0.885714285714

