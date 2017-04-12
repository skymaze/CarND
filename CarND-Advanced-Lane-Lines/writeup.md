##Writeup Template
###You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./writeup/ca_undist.png "Undistorted"
[image2]: ./writeup/undist.png "Road Transformed"
[image3]: ./writeup/binary_combo.png "Binary Example"
[image4]: ./writeup/warped_straight_lines.png "Warp Example"
[image5]: ./writeup/color_fit_lines.png "Fit Visual"
[image6]: ./writeup/4.jpg "Output"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.

You're reading it!
###Camera Calibration

####1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the IPython notebook located in "./Lane_Finding_Pipeline.ipynb"

I start by converting images to grayscale with cv2.cvtColor(img,cv2.COLOR_BGR2GRAY),
then, I use `cv2.findChessboardCorners()` finding chessboard corners. Finally, I use `cv2.calibrateCamera()` function with object points which will be appended with a copy of  the (x, y, z) coordinates of the chessboard corners every time I successfully detect all chessboard corners in a test image, image points which will be appended with the (x, y) pixel position of each of the corners, and the shape of the grayscale image.

I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

###Pipeline (single images)

####1. Provide an example of a distortion-corrected image.

The code for this step is included in a function called `undist_img()`, which is contained in the third code cell of the IPython notebook.

I use `cv2.undistort()` with the mtx and dist computed above. Here is an example:

![alt text][image2]

####2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

The code for this step is contained in the third code cell of the IPython notebook.

To generate a binary image, I used a combination with a selection for pixels where both the x and y gradients meet the threshold criteria, or the gradient magnitude and direction are both within their threshold values. Here's an example of my output for this step.

![alt text][image3]

####3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is included in a function called `warp_transform()`, which is contained in the third code cell of the IPython notebook.

The `warp_transform()` function takes as inputs an image (`img`), as well as corners which will provide source (`src`) and destination (`dst`) points.  I chose the hardcode corners in the following manner:

```
offset = 100

corners=np.float32([(190,720),(580,460),(700,460),(1090,720)])

src = np.float32([corners[0], corners[1], corners[2], corners[3]])

dst = np.float32([[2*offset, img_size[1]],
                 [2*offset, offset],
                 [img_size[0]-2*offset, offset],
                 [img_size[0]-2*offset, img_size[1]]])

```
This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 190, 720      | 200, 720      | 
| 580, 460      | 200, 100      |
| 700, 460      | 1080, 100     |
| 1090, 720     | 1080, 720     |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

####4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The code for this step is included in a function called `find_lines()`, which is contained in the third code cell of the IPython notebook.

I first toke a histogram along all the columns in the lower half of the image to find the base of the lane lines. Then I chose to use 12 sliding windows with margin +- 50px to find the lines. 

![alt text][image5]

####5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The code for this step is included in a function called `measuring_curvature()`, which is contained in the third code cell of the IPython notebook.

I derived a conversion from pixel space to world space in the perspective transformed image above, compare  with U.S. regulations, I get 

```
ym_per_pix = 30/720, xm_per_pix = 3.7/880
```

Then I assumed the camera is mounted at the center of the car and I compared the road center with the iamge center and cumputed the position of the vehicle with respect to center.

####6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

The code for this step is included in a function called `draw_lane()`, which is contained in the third code cell of the IPython notebook.

Here is an example of my result on a test image:

![alt text][image6]

---

###Pipeline (video)

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Diffent from single image processing. I cumputed average line data with last 6 iterations and the radius of curvature with last 24 iterations to make output stable.

Here's a [link to my video result](./project_video.mp4)

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

It is hard to keep the lane steady when the road line is difficult to identify, Then, I lower the margin which control the width of the windows when finding lines, and it works well.