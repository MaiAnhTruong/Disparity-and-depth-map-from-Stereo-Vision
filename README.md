# Disparity-and-depth-map-from-Stereo-Vision

This repository implements a pipeline for 3D reconstruction using stereo images. The code processes a pair of images (a stereo pair) and computes the 3D structure by estimating the Fundamental and Essential matrices, recovering camera pose, performing stereo rectification, and computing both a disparity map and a depth map. Each of the steps below is accompanied by the underlying mathematical rationale.

---

## 1. Preprocessing

**Purpose:**  
- Convert the color image to grayscale to reduce unnecessary information for feature detection.  
- Apply a Gaussian blur to smooth the image and reduce noise.

**Mathematics:**  
- **Grayscale Conversion:**  
  \[
  I_{\text{gray}}(x,y) = 0.299\,R(x,y) + 0.587\,G(x,y) + 0.114\,B(x,y)
  \]
- **Gaussian Blur:**  
  Use the Gaussian kernel defined as:
  \[
  G(x,y) = \frac{1}{2\pi\sigma^2} \exp\left(-\frac{x^2+y^2}{2\sigma^2}\right)
  \]
  and perform a convolution between the grayscale image and the Gaussian kernel.

---

## 2. Feature Detection

**Purpose:**  
- Utilize the SIFT algorithm to detect keypoints in each image and compute their descriptors.

**Process:**  
- Create a SIFT object and call `detectAndCompute()` on the preprocessed image.
- Use the Brute-Force Matcher (BFMatcher) to match descriptors between the two images.
- **Lowe’s Ratio Test:**  
  Retain matches that satisfy:
  \[
  \text{if } m.\text{distance} < 0.7 \times n.\text{distance, then keep } m
  \]
  This test helps eliminate unstable matches.

---

## 3. Fundamental Matrix Estimation

**Purpose:**  
- Compute the Fundamental matrix \( F \) that satisfies the epipolar constraint for each pair of corresponding points:
  \[
  x'^T F x = 0
  \]

**8-Point Algorithm:**  
1. For each corresponding pair of points \((x, y)\) from the left image and \((x', y')\) from the right image, construct a row of the matrix \( A \):
   \[
   A_i = [x'x, \quad x'y, \quad x', \quad y'x, \quad y'y, \quad y', \quad x, \quad y, \quad 1]
   \]
2. Stack at least 8 pairs to form the system of equations:
   \[
   A f = 0
   \]
   where \( f \) is the 9-element vector formed from the entries of \( F \).
3. Solve for \( f \) using Singular Value Decomposition (SVD) of \( A \):
   \[
   A = U \Sigma V^T
   \]
   The solution \( f \) is the right singular vector corresponding to the smallest singular value (typically the last row of \( V \)).
4. **Enforcing Rank-2 Constraint:**  
   Perform SVD on \( F \):
   \[
   F = U_F \Sigma_F V_F^T
   \]
   Set the smallest singular value in \( \Sigma_F \) to zero:
   \[
   \Sigma_F = \operatorname{diag}(s_1,\, s_2,\, 0)
   \]
   Reconstruct \( F \) as:
   \[
   F = U_F\, \Sigma_F\, V_F^T
   \]
5. Normalize \( F \) if necessary so that a specific element (or the norm) is scaled appropriately.

---

## 4. RANSAC for Outlier Rejection

**Purpose:**  
- Eliminate outliers from the matched feature points and retain only those that are consistent with the computed \( F \).

**Process:**  
- Run multiple iterations (e.g., 1500 iterations):
  - Randomly select 8 corresponding point pairs.
  - Compute \( F \) using the 8-point algorithm.
  - For all point pairs, evaluate the epipolar error:
    \[
    \text{error} = \left| x'^T F x \right|
    \]
  - If the error for a point is less than a predefined threshold (e.g., 0.02), consider it as an inlier.
- Retain the \( F \) that results in the highest number of inliers.

---

## 5. Essential Matrix Computation

**Purpose:**  
- Using the camera intrinsic matrices \( K_1 \) and \( K_2 \), convert the Fundamental matrix \( F \) to the Essential matrix \( E \) via:
  \[
  E = K_2^T \, F \, K_1
  \]

**Adjustment:**  
- Compute the SVD of \( E \):
  \[
  E = U\, \operatorname{diag}(s_1,\, s_2,\, s_3)\, V^T
  \]
- Enforce the constraint that \( E \) has two equal singular values and the third equal to zero:
  \[
  \operatorname{diag}(1,\,1,\,0)
  \]

---

## 6. Decomposition of the Essential Matrix

**Purpose:**  
- Decompose \( E \) to obtain the possible rotation \( R \) and translation \( t \) (or camera center \( C \)) between the two camera views.

**Process:**  
1. Compute the SVD of \( E \):
   \[
   E = U \Sigma V^T
   \]
2. Define the auxiliary matrix:
   \[
   W = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}
   \]
3. Derive four possible combinations:
   - \( R_1 = U\,W\,V^T \)
   - \( R_2 = U\,W^T\,V^T \)
   - The translation vector \( t \) (or camera center \( C \)) is taken from the third column of \( U \) (with sign variations).
4. Ensure that the determinant \( \det(R) > 0 \) by adjusting the sign if necessary.

---

## 7. Triangulation – Computing 3D Coordinates

**Purpose:**  
- Recover the 3D coordinates of matching points from their 2D correspondences using projection matrices.

**Process:**  
1. **Projection Matrices:**
   - For the first camera (with \( R = I \) and \( C = 0 \)):
     \[
     P_1 = K_1 \, [I \; | \; 0]
     \]
   - For the second camera:
     \[
     P_2 = K_2 \, [R \; | \; -R\,C]
     \]
2. **Constructing the Linear System:**  
   For each pair of corresponding image points \( x \) (from the left image) and \( x' \) (from the right image), set up a system of equations from the relationship:
   \[
   A\,X = 0
   \]
   where \( A \) is constructed from the rows of \( P_1 \) and \( P_2 \) such that:
   \[
   x_i\, (P_{k3}^T) - P_{ki}^T \quad \text{for } i = 1,2 \text{ and } k = 1,2.
   \]
3. **Solving:**  
   Solve the homogeneous system using SVD to find the 3D point \( X \) (in homogeneous coordinates) and normalize it so that the last element becomes 1.
4. Repeat for all valid point correspondences under each possible camera pose candidate.

---

## 8. Disambiguation of Camera Pose

**Purpose:**  
- Select the correct pair \( (R, t) \) by checking which candidate results in the most 3D points having positive depth (i.e., lying in front of both cameras).

**Criteria:**  
- For each candidate pose \( (R, t) \), count the number of reconstructed 3D points \( X \) that satisfy:
  \[
  (r_3)^T (X - C) > 0 \quad \text{and} \quad X_z > 0
  \]
  where \( r_3 \) is the third row of \( R \) and \( C \) is the camera center.
- The candidate with the maximum count of such points is chosen.

---

## 9. Epipolar Lines Computation and Drawing

**Purpose:**  
- Compute epipolar lines to visually represent the geometric relationship between the corresponding points in the stereo images.

**Mathematics:**  
- Given the Fundamental matrix \( F \) and a point \( x \) in one image, the epipolar line in the other image is given by:
  \[
  l' = F \, x
  \]
- The function `cv2.computeCorrespondEpilines()` is used to compute these lines.
- The epipolar lines are then converted to coordinate points and drawn on the images.

---

## 10. Stereo Rectification

**Purpose:**  
- Transform the images so that the epipolar lines become parallel. This simplifies the search for corresponding points in a stereo vision context.

**Process:**  
- Use `cv2.stereoRectify` with the following inputs:
  - Camera intrinsic matrices \( K_1 \) and \( K_2 \)
  - Zero distortion coefficients (i.e., assuming no lens distortion)
  - The recovered rotation \( R \) and translation \( t \)
- The function outputs new projection matrices \( P_1 \) and \( P_2 \), along with the transformation matrices (homographies) \( H_1 \) and \( H_2 \):
  \[
  H_i = P_i(:, :3) \cdot K_i^{-1}, \quad i = 1,2
  \]
- Using `cv2.initUndistortRectifyMap`, each original image is remapped to a rectified version.

---

## 11. Disparity Map Computation

**Purpose:**  
- Calculate the disparity (horizontal shift) between corresponding pixels in the rectified stereo images.

**Process:**  
- Convert the rectified images to grayscale.
- For each pixel in the left image, extract a block (for example, a \(7 \times 7\) window).
- Within a predefined horizontal search range on the right image, compute the **Sum of Absolute Differences (SAD):**
  \[
  \text{SAD} = \sum_{i,j} \left| I_{\text{left}}(x+i, y+j) - I_{\text{right}}(x'+i, y+j) \right|
  \]
- The disparity at position \((x, y)\) is then computed as:
  \[
  \text{disparity}(x,y) = |x - x'|
  \]
  where \( x' \) is the horizontal coordinate in the right image that minimizes the SAD.

---

## 12. Depth Map Computation

**Purpose:**  
- Convert the disparity map into a depth map using the known focal length and the baseline (distance between the two cameras).

**Formula:**  
- Let \( f \) be the focal length and \( B \) be the baseline, then the depth is given by:
  \[
  \text{depth}(x,y) = \frac{B \cdot f}{\text{disparity}(x,y) + \epsilon}
  \]
  where \( \epsilon \) is a small constant added to prevent division by zero.
- The depth map is then normalized and, optionally, a colormap is applied for visualization.

---

## Summary of the Pipeline

1. **Preprocessing:** Convert to grayscale and apply Gaussian smoothing.
2. **Feature Detection:** Use SIFT to extract keypoints and descriptors; match features with BFMatcher using Lowe’s ratio test.
3. **Fundamental Matrix Estimation:** Apply the 8-point algorithm combined with RANSAC for robust estimation.
4. **Essential Matrix Computation:** Compute \( E \) using the intrinsic parameters.
5. **Decomposition of \( E \):** Derive the possible rotation matrices \( R \) and translation vectors \( t \).
6. **Triangulation:** Compute 3D point coordinates from 2D correspondences using projection matrices.
7. **Pose Disambiguation:** Choose the correct \( (R, t) \) by ensuring the reconstructed 3D points lie in front of the cameras.
8. **Epipolar Lines:** Compute and draw the epipolar lines for visualization.
9. **Stereo Rectification:** Use homographies to map the original images to rectified images.
10. **Disparity Calculation:** Use block matching with the SAD metric to compute disparity.
11. **Depth Mapping:** Calculate depth from the disparity using the relation \( \text{depth} = \frac{B \cdot f}{\text{disparity}} \).

Each of these steps is deeply rooted in geometric vision and image processing theory, covering from epipolar geometry and camera calibration to stereo vision-based 3D reconstruction.

---

Feel free to consult additional resources on **Epipolar Geometry**, the **8-Point Algorithm**, and **Stereo Vision** for more in-depth understanding.
