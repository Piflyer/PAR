# Lightweight Low-Poly Photogrammetry with Object Classification Validation
Using OpenCV and PointNet for object classification validation

## Abstract
Images in their purest form can only provide information in a two-dimensional space, which lacks the information that can be found in the three-dimensional space of the real world. Information such as the cross-sectional shape of each part of an object and the volumetric properties of the object is lost, and these factors are often a method used to help classify complex objects, such as fossils and manufacturing defects. To leverage the versatility that images provide and transform them into a medium with more detail, photogrammetry is a popular method to do the task. With programs like COLMAP and Meshroom, the user simply puts in the images of an object and the program tries its best to produce a 3D version of the object using keypoint detection. When paired up with 3D object classification, images can be transformed into 3D objects that contain a lot more data and can be used for robust classification. While both programs produce 3D objects with color, both programs may require specialized hardware like a dedicated GPU to run on them. Even without the need for dedicated GPUs, these programs still take a very long time to perform reconstruction, These factors alone put a limit on the practicality of photogrammetry in places where image recognition is not adequate. We can create a lightweight photogrammetry package that can run a wide range of form factors, from laptops to Raspberry Pis, and that can perform up to 9.8 times faster than COLMAP while producing up to 21 times more vertices. When leveraging the object classification network to validate the quality of the mesh, we can get an accuracy rate of up to 27% compared to COLMAP's 0%. This highlights that a high-quality and lightweight photogrammetry package is possible in the age of edge computing.


## Features
* Python 3.9+ based reconstruction system
* Can be configured to use ORB or SIFT
* CPU based reconstruction
* Uses Alpha Surface for mesh reconstruction
* Uses OpenCV for image processing
* Use PointNet for object classification validation tests
* Can sync with Arduino rigs for automated reconstruction


## Organization
* [Arduino](Arduino) - Contains Arduino code for turntable and rail
* [PAR](PAR) - Contains Python Automated Reconstruction (PAR) code for photogrammetry
* [PointNet](PointNet) - Contains PointNet code for object classification

## Installation

```pip3 install opencv-python numpy matplotlib open3d-python scipy tensorflow```

*Note: Open3D might not be available on Linux, therefore surface reconstruction may not work. You will also need to compile TensorFlow for your environment.*

## Usage

Optional: Follow instructions to assemble Arduino [rail](Arduino/rig.md) or [turntable](Arduino/turntable.md) and pair it with PAR.

```
import PAR
```

## calibration() 
```
PAR.calibration(checkerboard_x, checkerboard_y, image_path, output_matrix_path, output_distortion_path)
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
PAR.undistort(image_path, output_path, calibration_matrix_path, distortion_coefficients_path)
```
* ```image_path``` : Path to image folder containing images to be undistorted (should include wildcard at the end)
* ```output_path``` : Path to output folder for undistorted images
* ```calibration_matrix_path``` : Path to camera calibration matrix file
* ```distortion_coefficients_path``` : Path to camera distortion coefficients file

## triangulate() 
Finds triangulated points from two images. Performs ORB or SIFT feature detection and matching and passes that through the essential matrix in order to find the triangulated points.
```
PAR.triangulate(image_path_1, image_path_2, calibration_matrix_path, distortion_coefficients_path, alogrithm, output_path)
```
* ```image_path_1``` : Path to image 1
* ```image_path_2``` : Path to image 2
* ```calibration_matrix_path``` : Path to camera calibration matrix file
* ```distortion_coefficients_path``` : Path to camera distortion coefficients file
* ```alogrithm``` : Algorithm to use for feature detection and matching. Options are ORB or SIFT
* ```output_path``` : Path to output file for triangulated points

## multitriangulate() 
Finds triangulated points from sequential set of images. Performs BRISK, ORB, or SIFT feature detection and matching and passes that through the essential matrix in order to find the triangulated points. Returns a numpy list of triangulated points or point clouds.
```
PAR.multitriangulate(images_path, calibration_matrix_path, distortion_coefficients_path, keypoint_algorithm, recon_algorithm, ply)
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

