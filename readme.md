# Introduction

To perform object grasping based on the DL approach, it is often necessary to use DL to detect, classify, and locate 3D objects in preparation for grasping. To reduce the hardware costs of image sensors, research is often done on data collected from RGB-D sensors, which are collected from sensors capable of collecting RGB and depth (D) data, formerly the MS Kinect version (V)1, MS Kinect V2, and now the Intel RealSense sensor.

To be able to grasp objects on the table by the robot arm, or to support the process of grasping objects by the blind in the environment, the objects on the table need to be detected, recognized, and accurately positioned according to the coordinate system on the table, as shown in the figure below.

<p align="center">
<div style="display: flex; gap: 10px;">
  <img src="images\image2.png" width="100%">
</div>
    In particular, to perform the grip process accurately and safely, determining the grip position is very important.

In this paper, we publish a benchmark TQU-GraspingObject dataset for experimentation, evaluating, and comparing methods for detecting, recognizing, and locating 3D objects on the tabletop. The data were set up and collected based on blind mannequins, in a shared environment room using IR-D435 with five types of common everyday objects placed on the tabletop. TQU-GraspingObject dataset includes RGB-D image data, object box annotation, label annotation, coordinate system transformation matrix from the center of the camera to the tabletop, 3D object position annotation, 3D point cloud data, hand pose for grasping. Figure below presents the entire process of data collection, preparation and publishing the benchmark TQU-GraspingObject dataset.
<p align="center">
<div style="display: flex; gap: 10px;">
  <img src="images\image1.png" width="70%">
</div>



<img src="images\anhh.jpg" width="100%">
</p>
---

# Building point cloud data
In which the data we publish includes RGB and depth image data, based on this pair of data,  point cloud data is generated based on the camera’s internal parameter set according to the formula as below.

<img src="images\image.png" width="15%">

where given a pixel coordinate (u, v) with depth Z(u, v), and camera intrinsic parameters (fx, fy, cx, cy), each point P on the point cloud has coordinates (x, y, z), in our study we have fx = 613.7351684570312, fy = 614.2782592773438, cx = 320.08074951171875, cy = 243.789794921875. The color value of point P is the same as the color value of the pixel whose coordinates are being calculated.
# Point Cloud Dataset

A point cloud is a large collection of data points in three-dimensional space, each with coordinates (x, y, z) and may contain additional information such as color and brightness.

Below is an illustration of a color image (RGB), a depth image (Depth), and a point cloud (Point Cloud) representing data in three-dimensional (3D) space.

<div style="display: flex; gap: 10px;">
  <img src="images\anh_point_cloud.png" width="20%">
  <img src="images\anh_rgb.png" width="20%">
  <img src="images\anh_deth.png" width="20%">
</div>

---




# TQU-GraspingObject Dataset 

## Data Acquisition

  <img src="images\xoay.jpg" width="100%">
  
The equipment we used for data collection includes an IR-D435 camera and a Laptop to collect
and save data with configuration: MSI Modern 15 B12MO CPU - i5 1235U 16GB RAM GPU Intel
Iris Xe Graphics. To collect data on a Laptop installed with the software
we used to process and collect data is Inter RealSense Viewer. In the sharing room at Tan Trao
University (TQU), there is a table 0.8m high, the table plane size is 0.8 × 1.0m. A blind mannequin
wears the device, and walks around the table with the distance between
the person and the table is about 0.7m, the camera direction is always towards the table plane.
The objects placed on the tabletop include: a cup, a yogurt, a banana, a milk carton, an apple.

The dataset contains **63,593 images** captured by the Intel RealSense D435 camera with high resolution.  
Each image contains from **1 to 5 objects** including:

- Apple  
- Milk carton  
- Yogurt  
- Cup  
- Banana  

The data is **manually annotated**.

  <img src="images\thamso.jpg" width="100%">


## Data Normalization

To evaluate the 3D-HPE and 3D HAR, we split the training and validation data like the human data segmented evaluation data. 
3D human pose renderings of Mediapipe, LiftingA in different spatial coordinates. Before performing their 3D human pose estimation results, we normalize the estimated data to the same coordinate system as 3D human pose annotation data according to the following process. We combine the findings of the rotation and the translation matrix into a process, in which the rotation and translation matrices are represented in the 3-D space as Eq. (1).

## Equation (1)

```
⎡ x' ⎤   ⎡ R11  R12  R13  T1 ⎤   ⎡ x ⎤
⎢ y' ⎥ = ⎢ R21  R22  R23  T2 ⎥ × ⎢ y ⎥
⎢ z' ⎥   ⎢ R31  R32  R33  T3 ⎥   ⎢ z ⎥
⎣ 1  ⎦   ⎣  0    0    0   1  ⎦   ⎣ 1 ⎦
```

Where **P(x, y, z)** is the estimated point of 3-D human pose estimation result;  
**P'(x', y', z')** is the estimated point of the 3-D human pose estimation result after transforming to the same coordinate system with the 3-D ground truth data.  
Therefore, we have a formulation as in Eq. (2).

## Equation (2)

```
x' = R11·x + R12·y + R13·z + T1
y' = R21·x + R22·y + R23·z + T2
z' = R31·x + R32·y + R33·z + T3
```

From the coordinates of the key points in the 3-D human pose of the dataset, we define the coordinates of a 3-D pose including **n** points as in Eq. (3).

## Equation (3)

```
[ 1   z1   y1   x1 ]
[ 1   z2   y2   x2 ]
[ .    .    .    . ]
[ .    .    .    . ]
[ .    .    .    . ]
[ 1   zn   yn   xn ]
```

In particular, the rotation matrix and translation according to the x, y, z axes are presented in the order θ1, θ2, θ3 as in Eq. (4).

## Equation (4)

```
θ1 = [ T1   R13   R12   R11 ]
θ2 = [ T2   R23   R22   R21 ]
θ3 = [ T3   R33   R32   R31 ]
```

The results of rotation and translation are shown in the vectors X', Y', Z' as in Eq. (5).

## Equation (5)

```
X' = [ x1'  x2'  ...  xn' ]
Y' = [ y1'  y2'  ...  yn' ]
Z' = [ z1'  z2'  ...  zn' ]
```

Where:

- \( x_i, y_i, z_i \) are the coordinate values on the 3-D pose **ground truth data**,  
- \( x_j, y_j, z_j \) are the coordinates of the **estimated** 3-D human pose data,  
  which must be rotated and translated to the same coordinate system with the 3-D human pose ground truth data.

From this, we have a system of linear equations presented in Eq. (6).

## Equation (6)

```
X' = M·θ1
Y' = M·θ2
Z' = M·θ3
```

In which the estimation θᵢ is done using the Least Squares (LS) method as in Eq. (7).

## Equation (7)

```
θ1 = (MᵀM)⁻¹ Mᵀ X'
θ2 = (MᵀM)⁻¹ Mᵀ Y'
θ3 = (MᵀM)⁻¹ Mᵀ Z'
```

The source code to normalize the data is shown in the link:  
**Evaluation_measurement.zip**  
[download](https://drive.google.com/drive/folders/1qKxYRZIF3RI0LaA8K9wM3M684pEetHkx?usp=sharing)


Finally, we have the transformation matrix in the form:

```
( θ1 ; θ2 ; θ3 )
```

## Tabletop Object Detection and Recognition Dataset (2025)

**Best model result achieved: AP = 99.49%** → emphasizing the importance of temporal diversity and high-quality data.

<div style="display: flex; gap: 10px;">
  <img src="images\anh_quay.jpg" width="35%">
  <img src="images\anh_cac_doi_tuong.png" width="35%">
</div>


  <img src="images\anh_danh_gia.png" width="75%">

## Media
<img src="images\cac_fold.png" width="75%">

![description](images\detec_mobilenetv3.gif)
---

# TQU-GraspingObject Annotation

## RGB-D images
To download RGB-D images

## YOLO Labels ColorImage

This folder contains a compressed file storing YOLO labels of each object from 10 scenes. They can be used together with the color images in the RGB-D folder.

**Link:** [Download](https://drive.google.com/drive/folders/1pZFtDkGpglHXQcGK75IzJvDUjcwkypSZ?usp=drive_link)

## 4 points Marked on the RGB Images

This folder contains a compressed file storing 4 points on the color images of 10 scenes. Each frame in the compressed file marks the positions of 4 points.

**Link:** [Download](https://drive.google.com/drive/folders/1Mi_U7n4SVUnoKNDI8q0Y7DlG-1GQxsVy?usp=drive_link)

## 4 Points Marked on the PointCloud 

This folder contains a compressed file storing 4 points on the Point Cloud of 10 scenes. Each frame in the compressed file marks the positions of 4 points.

**Link:** [Download](https://drive.google.com/drive/folders/1kqfFrGwOAVViD1xyLRJGiBYua3dsK9Qa?usp=drive_link)

## 4 Points Transformed PointCloud

This folder contains a compressed file storing 4 points after being transformed into the coordinate system of 10 scenes. Each frame in the compressed file contains 4 points in `.PCD` format.

**Link:** [Download](https://drive.google.com/drive/folders/1BSTjL-rE28VD-zYZcfJWngaxFP1-T5VN?usp=drive_link)

## Transformation Matrices

This folder contains a compressed file storing the 4x4 matrices of 10 scenes, including the matrix of each frame and one file that combines all matrices.

**Link:** [Download](https://drive.google.com/drive/folders/1WPEmfQz3d83OGAX1Z6I_nVJFAqdWH6Kf?usp=drive_link)

## PointCloud Coordinate Transform

This folder contains a compressed file storing the coordinate system data used for transformation of 10 scenes, including the coordinate origin.

**Link:** [Download](https://drive.google.com/drive/folders/13zYJdTLIBTVBLPZUWnzgRVX44mtP3Ick?usp=drive_link)

## PointCloud Objects

This folder contains a compressed file storing the Point Cloud of each object in 10 scenes. Each frame is separated into the Point Cloud of each object based on YOLO bounding boxes and their corresponding labels.

**Link:** [Download](https://drive.google.com/drive/folders/1Mr-Q-Q9vO53xbDgogzxxBaJ5ouckgjr5?usp=drive_link)

## Transformed objects PointCloud

This folder contains a compressed file storing the Point Cloud after transforming to the coordinate system of 10 scenes (`.PCD` format).

**Link:** [Download](https://drive.google.com/drive/folders/1VJyNZZENzHFD06wyNKz--FYU1LZ33wrG?usp=drive_link)

