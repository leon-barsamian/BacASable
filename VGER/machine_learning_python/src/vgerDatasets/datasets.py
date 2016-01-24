import numpy as np
import csv


def load_vger_data(csv_data_file_path):
    """
    :param csv_data_file_path: an export of google tango point cloud data given by  "public void onXyzIjAvailable(final TangoXyzIjData xyzIj)"
    :return:

    data format : x,y,z coordinates, on point per line, value in meter


    In order to have better estimation and improve process speed we filter the point from the point cloud received.
    We choose to keep all point where y <= robot max height. in this case point 0,0 is the top of the robot

    """

    height_max = 0
    height_min = -10

    n_samples = 0

    with open(csv_data_file_path) as csv_file:
        data_file = csv.reader(csv_file)

        for nbLine, values in enumerate(data_file):
            # filtering interesting points
            if height_max > float(values[1]) > height_min:
                n_samples += 1

    # second read of file should be avoided.
    with open(csv_data_file_path) as csv_file:
        data_file = csv.reader(csv_file)
        n_features = 2
        data = np.empty((n_samples, n_features))
        target = np.empty((n_samples,), dtype=np.int)

        # xyz tango data are in meters so for conveniance we translate into centimeter.
        factor = 100
        count = 0
        for i, ir in enumerate(data_file):
            if height_max > float(ir[1]) > height_min:
                data[count] = np.multiply(np.around(np.asarray(ir[:-1], dtype=np.float), decimals=2), factor)
                target[count] = np.multiply(np.around(np.array(ir[-1], dtype=np.float), decimals=2), factor)
                count += 1

    return data, target
