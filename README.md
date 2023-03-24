# Lidar Obstacle Detection Project

This Project is done in case of Sensorfusion Engineer Nanodegree

https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313?utm_source=gsem_brand&utm_medium=ads_r&utm_campaign=19167921312_c_individuals&utm_term=143524476479&utm_keyword=sensor%20fusion%20udacity_e&gclid=Cj0KCQjwlPWgBhDHARIsAH2xdNckrndSxxvGmHAcx5fyXMbdraijrduF8iQLwajeDc6TxK0jxEVST3saAiVlEALw_wcB

## Description



## Real PCD vs Simulated

At the beggining of the chapter related to this project, we mainly used Simulated data represented as a Cloud Point. Each Point is defined by a set of 3 coordinates (x, y, z), we call it PointXYZ.
In real PCD data, a fourth dimension is added, I, the intensity, representing how strong is the received laser beam. For real data, the points becoming PointXYZI.
To avoid any duplicated code, make it more cleaner and more maintable, we using C++ templating for the function and using PointT class to represent any variable that needs to store a point.

The real PCD data containes more points than the simulated ones and represent a real 3D space with all objects that a car may encounter on a road.

![img1]("images/input.png")

In addition, due to uncontrolled environment, the real PCD may present incorrect points placement due to reflection or laser absorbtion. Those unexpected points may lead, in the next stages, to create False Positive or True Negatives detections.
<BR><BR><BR>


## Point Cloud Filtering

Processing the entire cloud points is challenging and computing intensive mainly due to extremly high resolution of the lidar. For ADAS applications, the main goal is to be as close as possible to real time and by that keeping the runtime as low as possible and to keep unough accuracy for the use case.

To reduce the processing runtime of our application and reduce the overload on the computing system we will apply several filter on out point cloud to make it more usable.

The filters that we will apply:
>- Reduce point cloud spatial dimensions. The lidar can measures points up to hundreds of meters. But for Collision avoidance systems there is no advantages to have all those points. 
>- Remove points reflected by the ego car. Depending on how the Lidar is installed, some lidar beams might detect the car roof it self.
>- Move to voxelgrid representation. Lidar can have a very high resolution, especially due to radial point distribution, closer you are to the laser and higher is the resolution. To reduce the number of points and make our point cloud distribution more linear, we introduce VoxelGrid representation. By doing that we will resume a 3D volume by a unique point.

After applying all those filters, the number of points drop from 118703 down to 1187 (Number of points for the displayed example)
<strong>The point Cloud was reduce by 99.9%.</strong>

![img2]("images/filter.png")
<BR><BR><BR>

## Plane Segmentation

Previously, we preprocessed our point Clouds. As Lidar beams also going with negative inclination, it will also catches the road. Now, we need to differentiate objects from the road. 
To do that, we will segment our pointCloud using Ransac fitting method.
Basically, Ransac will maximize the number of point belonging to a requested shape (line, plane).
In our case, we configuring RANSAC to try to fit the maximum points as possible to a plane and consider that plane is the road.

![img3]("images/segment_plane.png")

The downside of this method, it's that the results will depend of the RANSAC parameters. One of them, will also impact the runtime of our algorithms and it's the number of iterations. So more iterations we will request, more precise it will be, but more time it will take.

<BR><BR><BR>

## Clustering and Bbox Rendering

At this point, we are able to differentiate road from other objects. Neverthless, it's not sufficient for collision avoidance. Points that not belongs to the road are part of another object, but you might have several objects with different shapes and trajectories. To split our objects points into different instances we are using Clustering method. Precisely,  we will use Euclidean Clustering.
The Euclidean Clustering uses KD Tree to enhance neighbours points search and Euclidean distance to evaluate the distance between them.

![img1]("images/clustering.png")

To further clean our cloud points, we adding a minimum number of points to consider it as an instance and also a maximum number of points.

The quality of clustering is highly impacted by the parametrization of the Clustering function, the min, the max number of points and especially the distance threshold between points.

For our configuration, we are using:
>- Distance Threshold 0.5
>- 5 
>- 400

Using those parameters, allows us to discriminate and detect a pole from traffic_sign/traffic_light.

The main issues with that, it's for the pdc/data_1 it's working perfectly but in other use cases it might perform poorly.

Ideally, would be to perform a calibration process an a large set of pcd data or somehow dynamically update those values.

## PCL vs Personal Implementation

To avoid commenting and uncommenting ou, we define a preprocessor directive through define macro. This macro is USE_PCL, if set to 0(false) it will use our own implementation and if it's set to 1 (true) it will use the PCL libraries.

Eventhough USE_PCL is set 0(false), we will still use some of the functions from this libraries. All main functions, like PlaneSegmentation using Ransac and Clustering using Euclidean distance will <strong>NOT</strong> use this libraries for main computation

The following measurements are given for indication and may differ depending of the image used.
The runtimes of the filtering are not taken into consideration as it's same method but to be noticed the runtime of this filtering is changing.


| Stage  | Runtime PCL | Runtime Own Method |
| ------------- | ------------- | ------------- |
| Filtering  | 8.5ms  | 14ms |
| Plane Segmentation  | 0ms |  70ms |
| Clustering | 2ms |  18ms |

On any fuunction, the PCL library exceed our implementation. This can be due to different implementation technics used by PCL library and the use of boost library.


<img src="images/video.gif" width="800" height="400" />
