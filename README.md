# Python Automatic Reconstruction 

A lightweight, fast, and simple automatic reconstruction system for low-poly meshes and low powered computers.

## Features
* Python 3.9 based reconstruction system
* Can be configured to use ORB or SIFT
* CPU based reconstruction
* Uses Alpha Surface for mesh reconstruction
* Uses OpenCV for image processing
* Can sync with Arduino rigs for automated reconstruction


## Installation

```pip3 install opencv-python numpy matplotlib open3d-python scipy```

*Note: Open3D might not be available on Linux, therefore Alpha reconstruction may not work*

## Usage

Optional: Follow instructions to assemble Arduino [rail](Arduino/rig.md) or [turntable](Arduino/turntable.md) and pair it with FAR.

Add PAR.py to your project folder and import it into your main script. PIP package is coming soon.

```
from PAR import *
```

## calibration() 
```
calibration(checkerboard_x, checkerboard_y, image_path, output_matrix_path, output_distortion_path)
```
Performs Camera Calibration on checkerboard images. Crucial for reconstruction.

* ```checkerboard_x``` : Number of checkerboard squares on x axis
* ```checkerboard_y``` : Number of checkerboard squares on y axis
* ```image_path``` : Path to image folder containing checkerboard images (should include wildcard at the end)
* ```output_matrix_path``` : Path to output camera matrix file
* ```output_distortion_path``` : Path to output camera distortion coefficients file

## undistort() 
Undistort images using calibration matrix and distortion coefficients. It is not needed for reconstruction.
```
undistort(image_path, output_path, calibration_matrix_path, distortion_coefficients_path)
```
* ```image_path``` : Path to image folder containing images to be undistorted (should include wildcard at the end)
* ```output_path``` : Path to output folder for undistorted images
* ```calibration_matrix_path``` : Path to camera calibration matrix file
* ```distortion_coefficients_path``` : Path to camera distortion coefficients file

## triangulate() 
Finds triangulated points from two images. Performs ORB or SIFT feature detection and matching and passes that through the essential matrix in order to find the triangulated points.
```
triangulate(image_path_1, image_path_2, calibration_matrix_path, distortion_coefficients_path, alogrithm, output_path)
```
* ```image_path_1``` : Path to image 1
* ```image_path_2``` : Path to image 2
* ```calibration_matrix_path``` : Path to camera calibration matrix file
* ```distortion_coefficients_path``` : Path to camera distortion coefficients file
* ```alogrithm``` : Algorithm to use for feature detection and matching. Options are ORB or SIFT
* ```output_path``` : Path to output file for triangulated points

## multitriangulate() 
Finds triangulated points from sequential set of images. Performs ORB or SIFT feature detection and matching and passes that through the essential matrix in order to find the triangulated points. Returns a numpy list of triangulated points or point clouds.
```
multitriangulate(images_path, calibration_matrix_path, distortion_coefficients_path, keypoint_algorithm, recon_algorithm, ply)
```
* ```images_path``` : Path to image folder containing images to be triangulated (should include wildcard at the end)
* ```calibration_matrix_path``` : Path to camera calibration matrix file
* ```distortion_coefficients_path``` : Path to camera distortion coefficients file
* ```keypoint_algorithm``` : Algorithm to use for feature detection and matching. Options are ORB, BRISK, or SIFT
* ```recon_algorithm``` : Algorithm to use for surface reconstruction. Options are either ALPHA or POISSON
* ```ply``` : Option to output point cloud as PLY file instead of numpy. False by default.

[//]: # (## reconstruct&#40;&#41;)

[//]: # (Reconstructs mesh from triangulated points)

[//]: # (```)

[//]: # (reconstruct&#40;triangulated_points_path, output_path, alpha&#41;)

[//]: # (```)

[//]: # (* ```triangulated_points_path``` : Path to triangulated points file)

[//]: # (* ```output_path``` : Path to output file for reconstructed mesh)

[//]: # (* ```alpha``` : Values between 0 and 1.0, determines how much coverage mesh should get. Higher values cover more but less detail is preserved and vice versa.)

# Acknowledgement
This work was partially funded by the NSF CAREER award “Certifiable Perception for Autonomous Cyber-Physical Systems”.

