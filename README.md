# Lidar_2D_Pointcloud_Filter
Filter algorithm that works with two dimensional pointclouds in polar coordinates (LIDAR-readout filter)

# Description
This algorithm takes lidar-readout data in polar coordinates (closed circle) and returns a filtered dataset in cartesian coordiantes.
Features include:
- aligning lines to the coordinate axes
- smoothes uneven scans
- discards error measurements that are too far off the nearest points, to improve the scan quality by filtering out measurements caused by reflections, etc.

Caution: it is the user's responsibility to rotate the input data in such a way that most lines are roughly parallel to the coordinate axes (only needed when filter threshold is greather than zero), doesn't matter when smoothing is the only needed operation

# Example
The following data is a readout of a living room using a very cheap 2D-Lidar module:
![Input data](images/input_data.jpg?raw=true "Input data")

After filtering, the plot looks the following:
![Output data](images/input_data.jpg?raw=true "Output data")


