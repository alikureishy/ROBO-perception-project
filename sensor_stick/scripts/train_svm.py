#!/usr/bin/env python
import pickle
import itertools
import numpy as np
import argparse
import matplotlib.pyplot as plt
from sklearn import svm
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn.model_selection import train_test_split
from sklearn import cross_validation
from sklearn import metrics


def plot_confusion_matrix(cm, classes,
                          normalize=False,
                          title='Confusion matrix',
                          cmap=plt.cm.Blues):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)

    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        plt.text(j, i, '{0:.2f}'.format(cm[i, j]),
                 horizontalalignment="center",
                 color="white" if cm[i, j] > thresh else "black")

    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')

if __name__ == "__main__":
    np.random.seed(100)
    
    parser = argparse.ArgumentParser(description='Capture point clouds for feature extraction')
    parser.add_argument('-i', dest="infile", required=True, type=str, help='Pickle file with point cloud features')
    parser.add_argument('-o', dest='outfile', default = "model.pickle", help='File to store the model into')
    parser.add_argument('-p', dest='plot', action="store_true", default = False, help='Whether to plot the confusion matrix (default: False)')

    args = parser.parse_args()
    # Load training data from disk
    print("Loading training set...")
    training_set = pickle.load(open(args.infile, 'rb'))

    # Format the features and labels for use with scikit learn
    feature_list = []
    label_list = []

    for item in training_set:
        if np.isnan(item[0]).sum() < 1:
            feature_list.append(item[0])
            label_list.append(item[1])

    print('Features in Training Set: {}'.format(len(training_set)))
    print('Invalid Features in Training set: {}'.format(len(training_set)-len(feature_list)))

    X = np.array(feature_list)
    y = np.array(label_list)
    # Convert label strings to numerical encoding
    encoder = LabelEncoder()
    y = encoder.fit_transform(y)

    print ("Splitting train/test data..")
    # Create a 'held-out' test set:
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=1, random_state=0)

    print("Scaling feature columns...")
    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X_train)
    # Apply the scaler to X
    X_train = X_scaler.transform(X_train)

    # Create classifier
    clf = svm.SVC(kernel='rbf')

    # Set up 5-fold cross-validation
    kf = cross_validation.KFold(len(X_train),
                                n_folds=5,
                                shuffle=True,
                                random_state=1)

    # Perform cross-validation
    print("Performing cross validation on classifier")
    scores = cross_validation.cross_val_score(cv=kf,
                                             estimator=clf,
                                             X=X_train, 
                                             y=y_train,
                                             scoring='accuracy'
                                            )
    print('Scores: ' + str(scores))
    print('Accuracy: %0.2f (+/- %0.2f)' % (scores.mean(), 2*scores.std()))

    # Gather predictions
    predictions = cross_validation.cross_val_predict(cv=kf,
                                              estimator=clf,
                                              X=X_train, 
                                              y=y_train
                                             )

    accuracy_score = metrics.accuracy_score(y_train, predictions)
    print('accuracy score: '+str(accuracy_score))

    confusion_matrix = metrics.confusion_matrix(y_train, predictions)
    class_names = encoder.classes_.tolist()

    #Train the classifier
    print("Training the classifier...")
    clf.fit(X=X_train, y=y_train)
    model = {'classifier': clf, 'classes': encoder.classes_, 'scaler': X_scaler}

    # Predict using the held-out test set:
    print("Accuracy with held-out test-set...")
    y_pred = clf.predict(X_scaler.transform(X_test[0].reshape(1,-1)))
    print ("Prediction: {} / Actual: {}".format(y_pred, y_test[0]))

    # Save classifier to disk
    print("Saving classifier to disk...")
    pickle.dump(model, open(args.outfile, 'wb'))

    # Plot non-normalized confusion matrix
    if args.plot:
        plt.figure()
        plot_confusion_matrix(confusion_matrix, classes=encoder.classes_,
                              title='Confusion matrix, without normalization')

        # Plot normalized confusion matrix
        plt.figure()
        plot_confusion_matrix(confusion_matrix, classes=encoder.classes_, normalize=True,
                              title='Normalized confusion matrix')

        plt.show()
