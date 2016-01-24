# Machine learning in path-finding

Those few lines takes in input the x,y,z coordinates retrieved from Google Tango and use scikit-learn to find potential
waypoint.

The goal is to use the prediction to find possible ways using z coordinate information.
The Z coordinates default behaviour is to be linear. A gap in Z coordiantes between neigbors points
indicate a possible waypoint that the robot must check.

## Steps
1. read input data
2. instanciate KNeighborsClassifier and "train" it
3. check that the observed value match the predicted one. If not there is a potential waypoint.

## Misc
This code use scikit-learn 0.17 and run on python 3.4.3


