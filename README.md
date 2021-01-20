# Lidar_2D_Pointcloud_Filter

THIS REPOSITORY IS CURRENTLY UNDER DEVELOPMENT AND SOME INFORMATION MAY CHANGE

Filter algorithm that works with two dimensional pointclouds in polar coordinates (LIDAR-readout filter)
![Preview image](splashscreen.png?raw=true "Preview")

# Description
This algorithm takes lidar-readout data in polar coordinates (closed circle) and returns a filtered dataset in polar coordiantes.
Features include:
- aligning lines to the coordinate axes
- smoothes uneven scans
- discards error measurements that are too far off the nearest points, to improve the scan quality by filtering out measurements caused by reflections, etc.

Caution: it is the user's responsibility to rotate the input data in such a way that most lines are roughly parallel to the coordinate axes (only needed when filter threshold is greather than zero), doesn't matter when smoothing is the only needed operation

Comparison RAW / FILTERED (Low threshold value here):
![Comparison](Comparison.png?raw=true "Comparison")

# Example
The following data is a readout of a living room using a very cheap 2D-Lidar module:
![Input data](input_data.png?raw=true "Input data")

After filtering, the plot looks the following:
![Output data](output_data.png?raw=true "Output data")

# Example project
I build an example device and .net Interface to demonstrate how the algorithm behaves under real-world-conditions:
![Device](PC_APPLICATION.png?raw=true "Example device")

A ST VL53L1X Time-Of-Flight Sensor is the heart of this lidar implementation:
![Sensor](Lidar_Front.JPG?raw=true "VL53L1X TOF sensor")

Project (uses UART to communicate with the host PC):
![Sensor](Lidar_Top.JPG?raw=true "Hardware driver using a STM32 Discovery board")



