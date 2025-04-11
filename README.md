# Disparity-and-depth-map-from-Stereo-Vision

This repository implements an end-to-end pipeline for 3D reconstruction using stereo images. The code processes a pair of images (a stereo pair) and computes the 3D structure by estimating the fundamental and essential matrices, recovering camera pose, performing stereo rectification, and computing both a disparity map and a depth map. Each of the steps below is accompanied by the underlying rationale without using explicit mathematical formulas.

---

## 1. Preprocessing

**Purpose:**  
- Convert the color image to grayscale to reduce unnecessary information for feature detection.
- Apply a Gaussian blur to smooth the image and reduce noise.

**Explanation:**  
- **Grayscale Conversion:**  
  The color image is converted to grayscale by computing a weighted sum of the red, green, and blue channels. Here, the contributions of the color channels are balanced (approximately 30% for red, 59% for green, and 11% for blue) to maintain the perception of brightness.
  
- **Gaussian Blur:**  
  A Gaussian blur is applied to the grayscale image using a kernel whose weights are determined by the Gaussian (normal) distribution. This process smooths the image and helps to reduce high-frequency noise before further processing.

---

## 2. Feature Detection

**Purpose:**  
- Detect keypoints in each image using the SIFT algorithm and compute their corresponding descriptors.

**Process Description:**  
- Create a SIFT (Scale Invariant Feature Transform) object and apply its detection and descriptor computation method on the preprocessed image.
- Use a Brute-Force Matcher to find candidate matches between keypoints in the two images.
- Apply Lowe’s ratio test to filter out ambiguous matches by comparing the relative distances of the best matches. Only matches that pass this test (i.e., where the best match is significantly closer than the next best match) are retained.

---

## 3. Fundamental Matrix Estimation

**Purpose:**  
- Estimate the fundamental matrix that relates corresponding points between the left and right images such that the epipolar constraint is satisfied.

**Process Description:**  
- For each pair of matched points, construct a row in a matrix using the coordinates from both images.
- With at least eight pairs of points, set up a linear system whose solution gives the entries of the fundamental matrix.
- Solve the system using Singular Value Decomposition (SVD); the solution is chosen as the singular vector associated with the smallest singular value.
- Enforce that the resulting fundamental matrix is of rank 2 by zeroing out its smallest singular value and then (optionally) normalizing it.

---

## 4. RANSAC for Outlier Rejection

**Purpose:**  
- Remove outlier matches to improve the robustness of the fundamental matrix estimation.

**Process Description:**  
- Run many iterations (for example, 1500 iterations) where in each iteration:
  - A random set of 8 point pairs is selected.
  - A candidate fundamental matrix is computed using the 8-point algorithm.
  - The epipolar error is evaluated for all matches.
  - Matches that produce an error below a set threshold are marked as inliers.
- The candidate fundamental matrix that results in the maximum number of inliers is chosen as the best estimate.

---

## 5. Essential Matrix Computation

**Purpose:**  
- Convert the fundamental matrix into an essential matrix by incorporating the intrinsic parameters of the cameras.

**Process Description:**  
- Use the intrinsic calibration matrices from both cameras to compute the essential matrix.
- Refine the essential matrix by ensuring that it has the required structure (that is, two equal non-zero singular values and one zero singular value), which is characteristic of an essential matrix derived from calibrated cameras.

---

## 6. Decomposition of the Essential Matrix

**Purpose:**  
- Decompose the essential matrix to recover the relative rotation and translation (i.e., the camera pose) between the two views.

**Process Description:**  
- Apply SVD on the essential matrix to separate it into rotation and translation components.
- Use an auxiliary matrix to compute two possible rotation matrices.
- Derive the translation direction from one of the singular vectors (considering both possible sign variations).
- Out of the four candidate solutions (combinations of rotations and translations), select the one with physically plausible properties (for example, ensuring the rotation matrix has a positive determinant).

---

## 7. Triangulation – Computing 3D Coordinates

**Purpose:**  
- Recover the 3D positions of corresponding points from their 2D projections in both images.

**Process Description:**  
- Construct projection matrices for both the left (first) and right (second) cameras. The first camera typically uses an identity rotation and zero translation.
- For each pair of corresponding 2D points, set up a system of linear equations based on the projection matrices.
- Solve this system (typically via SVD) to obtain the 3D point in homogeneous coordinates and then normalize it.
- Repeat this process for all valid point correspondences to obtain the complete set of 3D points.

---

## 8. Disambiguation of Camera Pose

**Purpose:**  
- Choose the correct camera pose (rotation and translation) among the candidates derived from the essential matrix decomposition.

**Process Description:**  
- For each candidate pose, count the number of triangulated 3D points that have a positive depth (i.e., lie in front of both cameras).
- The candidate that results in the highest number of valid 3D points is selected as the correct camera pose.

---

## 9. Epipolar Lines Computation and Drawing

**Purpose:**  
- Visually validate and illustrate the geometric relationship between the images by drawing epipolar lines.

**Process Description:**  
- For any given point in one image, compute the corresponding epipolar line in the other image using the fundamental matrix.
- Use available computer vision functions (such as those in OpenCV) to calculate and draw these lines, ensuring that they correctly represent the constraints imposed by the stereo geometry.

---

## 10. Stereo Rectification

**Purpose:**  
- Transform the stereo images so that their epipolar lines become horizontal and parallel, which simplifies the matching process.

**Process Description:**  
- Compute new projection matrices and rectification homographies using the intrinsic parameters, rotation, and translation between the cameras.
- Remap the original images to generate rectified images in which corresponding points appear along the same horizontal lines.

---

## 11. Disparity Map Computation

**Purpose:**  
- Compute the disparity map that represents the horizontal offset (shift) between corresponding pixels in the rectified stereo images.

**Process Description:**  
- Convert the rectified images to grayscale.
- For each pixel (using a small window, e.g., 7x7), search over a specified range in the corresponding image to find the best matching block based on a similarity measure (such as the sum of absolute differences, SAD).
- Record the difference in the x-coordinate between the matched blocks as the disparity at that pixel.

---

## 12. Depth Map Computation

**Purpose:**  
- Derive a depth map from the disparity map using the known focal length and baseline (distance between cameras).

**Process Description:**  
- Since depth is inversely proportional to disparity, compute the depth at each pixel by dividing the product of the baseline and the focal length by the disparity value (adding a small constant to avoid division by zero).
- Normalize the resulting depth values for visualization or further processing.

---

## Summary of the Pipeline

1. **Preprocessing:** Convert color images to grayscale and apply Gaussian blur to reduce noise.
2. **Feature Detection:** Use SIFT to extract robust features and match them with a Brute-Force Matcher combined with Lowe’s ratio test.
3. **Fundamental Matrix Estimation:** Estimate the relationship between stereo images using the 8-point algorithm and refine it with RANSAC.
4. **Essential Matrix Computation:** Convert the fundamental matrix into an essential matrix by using camera calibration parameters.
5. **Decomposition of the Essential Matrix:** Decompose the essential matrix to obtain potential camera rotations and translations.
6. **Triangulation:** Recover 3D coordinates from 2D image correspondences.
7. **Pose Disambiguation:** Identify the correct camera pose by selecting the candidate with the most points in front of both cameras.
8. **Epipolar Lines:** Compute and draw epipolar lines to validate the correspondence between stereo pairs.
9. **Stereo Rectification:** Rectify images so that epipolar lines become aligned, simplifying the matching process.
10. **Disparity Calculation:** Compute the disparity map by block matching between rectified images.
11. **Depth Mapping:** Convert the disparity information into a depth map using the camera parameters.

Each step in this pipeline is grounded in the principles of geometric vision and image processing, from feature detection and camera calibration to 3D reconstruction.

---
## Dataset and Requirements
- **Python:** Version 2.0 or above  
- **Dependencies:**  
  - OpenCV  
  - NumPy
