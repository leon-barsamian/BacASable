print(__doc__)


from sklearn import neighbors, metrics
import time
from vgerDatasets import datasets as vgerdata


n_neighbors = 15


X, y = vgerdata.load_vger_data('../samples/1450688404569_30864.txt')

h = .02  # step size in the mesh


# we create an instance of Neighbours Classifier and fit the data.
start_time = time.time()
clf = neighbors.KNeighborsClassifier(n_neighbors, weights='uniform')
clf.fit(X, y)
print("--- Fit tooks %s seconds ---" % (time.time() - start_time))
start_time = time.time()
print(metrics.classification_report(y, clf.predict(X)))
print("--- classification report tooks %s seconds ---" % (time.time() - start_time))

start_time = time.time()
nb_anomalies = 0
for index, coordinates in enumerate(X):
    if -0 > coordinates[1] > -10:
        predic_result = clf.predict(coordinates.reshape(1, -1))
        if abs(predic_result[0] - y[index]) > 99 :
            nb_anomalies+=1
            print(coordinates, " Prediction : ", predic_result[0], " Observed : ", y[index])


print("%s X coordinate(s) for possible waypoint(s)" % nb_anomalies)
print()
print("--- possible waypoint(s) detection tooks %s seconds ---" % (time.time() - start_time))


