# Disparity-and-depth-map-from-Stereo-Vision

This repository implements a pipeline for 3D reconstruction using stereo images. The code processes a pair of images (a stereo pair) and computes the 3D structure by estimating the Fundamental and Essential matrices, recovering camera pose, performing stereo rectification, and computing both a disparity map and a depth map. Each of the steps below is accompanied by the underlying mathematical rationale.

---

## 1. Preprocessing

**Purpose:**  
- Convert the color image to grayscale to reduce unnecessary information for feature detection.  
- Apply a Gaussian blur to smooth the image and reduce noise.

**Mathematics:**  
- **Grayscale Conversion:**  
  $$ I_{\text{gray}}(x,y) = 0.299 \, R(x,y) + 0.587 \, G(x,y) + 0.114 \, B(x,y) $$
- **Gaussian Blur:**  
  The Gaussian kernel is defined as:
  $$ G(x,y) = \frac{1}{2\pi\sigma^2} \exp\left(-\frac{x^2+y^2}{2\sigma^2}\right) $$
  This kernel is convolved with the grayscale image.

---

## 2. Feature Detection

**Purpose:**  
- Utilize the SIFT algorithm to detect keypoints in each image and compute their descriptors.

**Process:**  
- Create a SIFT object and call the `detectAndCompute()` function on the preprocessed image.
- Use the Brute-Force Matcher (BFMatcher) to match descriptors between the two images.
- **Lowe’s Ratio Test:**  
  Retain matches that satisfy:
  $$ m.\text{distance} < 0.7 \times n.\text{distance} $$
  This helps to eliminate unstable matches.

---

## 3. Fundamental Matrix Estimation

**Purpose:**  
- Compute the Fundamental matrix \( F \) such that for each pair of corresponding points \( x \) and \( x' \), the following epipolar constraint holds:
  $$ x'^T F \, x = 0 $$

**8-Point Algorithm:**  
1. For each corresponding pair of points \((x, y)\) from the left image and \((x', y')\) from the right image, construct a row of the matrix \( A \):
   $$ A_i = [x'x, \; x'y, \; x', \; y'x, \; y'y, \; y', \; x, \; y, \; 1] $$
2. Stack at least 8 pairs to form a system of equations:
   $$ A\,f = 0 $$
   where \( f \) is a 9-element vector formed from the entries of \( F \).
3. Solve for \( f \) using Singular Value Decomposition (SVD) of \( A \):
   $$ A = U \,\Sigma \, V^T $$
   The solution \( f \) is taken as the right singular vector corresponding to the smallest singular value (typically the last row of \( V \)).
4. **Enforcing the Rank-2 Constraint:**  
   Decompose \( F \) via SVD:
   $$ F = U_F \, \Sigma_F \, V_F^T $$
   Then set the smallest singular value in \( \Sigma_F \) to zero:
   $$ \Sigma_F = \operatorname{diag}(s_1, \; s_2, \; 0) $$
   and reconstruct \( F \):
   $$ F = U_F \, \Sigma_F \, V_F^T $$
5. Normalize \( F \) if needed.

---

## 4. RANSAC for Outlier Rejection

**Purpose:**  
- Remove outliers from the matched feature points and retain only those that are consistent with the computed \( F \).

**Process:**  
- Run multiple iterations (e.g., 1500 iterations):
  - Randomly select 8 corresponding point pairs.
  - Compute \( F \) using the 8-point algorithm.
  - For every pair of points, evaluate the epipolar error:
    $$ \text{error} = \left| x'^T F \, x \right| $$
  - If the error is below a predefined threshold (e.g., 0.02), mark the point as an inlier.
- The best \( F \) is the one that produces the highest number of inliers.

---

## 5. Essential Matrix Computation

**Purpose:**  
- Convert the Fundamental matrix \( F \) to the Essential matrix \( E \) using the camera intrinsic matrices \( K_1 \) and \( K_2 \):
  $$ E = K_2^T \, F \, K_1 $$

**Adjustment:**  
- Compute the SVD of \( E \):
  $$ E = U \, \operatorname{diag}(s_1, \; s_2, \; s_3) \, V^T $$
- Enforce the constraint that \( E \) must have two equal singular values and one zero singular value:
  $$ \operatorname{diag}(1, \; 1, \; 0) $$

---

## 6. Decomposition of the Essential Matrix

**Purpose:**  
- Decompose \( E \) to derive the possible rotation matrices \( R \) and translation vectors \( t \) (or camera center \( C \)) between the two views.

**Process:**  
1. Compute the SVD of \( E \):
   $$ E = U \,\Sigma \, V^T $$
2. Define the auxiliary matrix:
   $$ W = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix} $$
3. Compute four possible combinations:
   - \( R_1 = U \, W \, V^T \)
   - \( R_2 = U \, W^T \, V^T \)
   - The translation (or camera center) \( t \) is derived from the third column of \( U \) (with both positive and negative sign variations).
4. Adjust the sign such that \( \det(R) > 0 \).

---

## 7. Triangulation – Computing 3D Coordinates

**Purpose:**  
- Recover the 3D coordinates for the matching points from their 2D correspondences via the projection matrices.

**Process:**  
1. **Projection Matrices:**
   - For the first camera (assuming \( R = I \) and \( C = 0 \)):
     $$ P_1 = K_1 \,[ I \; | \; 0] $$
   - For the second camera:
     $$ P_2 = K_2 \,[ R \; | \; -R\,C] $$
2. **Constructing the Linear System:**  
   For each pair of corresponding image points \( x \) (in the left image) and \( x' \) (in the right image), set up the equation:
   $$ A\,X = 0 $$
   where \( A \) is built from the rows of \( P_1 \) and \( P_2 \).
3. **Solving:**  
   Use SVD to solve for the 3D point \( X \) in homogeneous coordinates and normalize it (i.e., set the last coordinate to 1).
4. Repeat for all valid correspondences for each possible camera pose candidate.

---

## 8. Disambiguation of Camera Pose

**Purpose:**  
- Choose the correct \( (R, t) \) pair by checking which candidate yields the most 3D points with positive depth (i.e., lying in front of both cameras).

**Criteria:**  
- For each candidate pose \( (R, t) \), count the number of reconstructed 3D points \( X \) satisfying:
  $$ (r_3)^T (X - C) > 0 \quad \text{and} \quad X_z > 0 $$
  where \( r_3 \) is the third row of \( R \) and \( C \) is the camera center.
- The candidate with the highest count of such points is selected.

---

## 9. Epipolar Lines Computation and Drawing

**Purpose:**  
- Compute the epipolar lines to visually illustrate the geometric relationship between corresponding points in the stereo images.

**Mathematics:**  
- For a point \( x \) in one image, the corresponding epipolar line in the other image is given by:
  $$ l' = F \, x $$
- The function `cv2.computeCorrespondEpilines()` computes these lines, which are then converted into drawable coordinates and overlaid on the images.

---

## 10. Stereo Rectification

**Purpose:**  
- Transform the images such that the epipolar lines become parallel. This simplifies the search for corresponding points.

**Process:**  
- Use the function `cv2.stereoRectify` with:
  - Camera intrinsic matrices \( K_1 \) and \( K_2 \)
  - Zero distortion coefficients (assuming no lens distortion)
  - The computed rotation \( R \) and translation \( t \)
- The output includes new projection matrices \( P_1 \) and \( P_2 \) and the rectification homographies \( H_1 \) and \( H_2 \):
  $$ H_i = P_i(:, :3) \cdot K_i^{-1}, \quad i = 1,2 $$
- The images are then remapped using `cv2.initUndistortRectifyMap` to obtain rectified images.

---

## 11. Disparity Map Computation

**Purpose:**  
- Compute the disparity (horizontal shift) between corresponding pixels in the rectified stereo images.

**Process:**  
- Convert the rectified images to grayscale.
- For each pixel in the left image, extract a block (e.g., a \(7 \times 7\) window).
- Within a predefined horizontal search range in the right image, compute the **Sum of Absolute Differences (SAD):**
  $$ \text{SAD} = \sum_{i,j} \left| I_{\text{left}}(x+i, \, y+j) - I_{\text{right}}(x'+i, \, y+j) \right| $$
- The disparity at position \((x,y)\) is then defined as:
  $$ \text{disparity}(x,y) = |x - x'| $$
  where \( x' \) is the best matching horizontal coordinate in the right image.

---

## 12. Depth Map Computation

**Purpose:**  
- Convert the disparity map into a depth map using the focal length and the baseline (distance between the two cameras).

**Formula:**  
- Given the focal length \( f \) and baseline \( B \), the depth is computed as:
  $$ \text{depth}(x,y) = \frac{B \cdot f}{\text{disparity}(x,y) + \epsilon} $$
  where \( \epsilon \) is a small constant to prevent division by zero.
- The resulting depth map is normalized and can be visualized using a colormap.

---

## Summary of the Pipeline

1. **Preprocessing:** Convert to grayscale and apply Gaussian smoothing.
2. **Feature Detection:** Use SIFT to extract keypoints and descriptors; match features using BFMatcher with Lowe’s ratio test.
3. **Fundamental Matrix Estimation:** Apply the 8-point algorithm and RANSAC to robustly estimate \( F \).
4. **Essential Matrix Computation:** Convert \( F \) to \( E \) using intrinsic parameters.
5. **Decomposition of \( E \):** Obtain the possible rotation matrices \( R \) and translation vectors \( t \).
6. **Triangulation:** Compute 3D coordinates from 2D correspondences using the projection matrices.
7. **Pose Disambiguation:** Select the correct \( (R, t) \) based on positive depth criteria.
8. **Epipolar Lines:** Compute and draw epipolar lines for visualization.
9. **Stereo Rectification:** Compute homographies to transform the original images to rectified images.
10. **Disparity Calculation:** Use block matching and the SAD metric to compute the disparity map.
11. **Depth Mapping:** Convert disparity to depth using the formula \( \text{depth} = \frac{B \cdot f}{\text{disparity}} \).

Each step is deeply rooted in geometric vision and image processing theory, covering topics from epipolar geometry and camera calibration to stereo-based 3D reconstruction.

---

Feel free to consult additional resources on **Epipolar Geometry**, the **8-Point Algorithm**, and **Stereo Vision** for a deeper understanding.

