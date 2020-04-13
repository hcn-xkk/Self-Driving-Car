# **Finding Lane Lines on the Road** 

The goal of this project is to create a pipeline that finds lane lines on the road using road images.

This is a project from the Self-driving car engineer nanodegree with Udacity. 


[//]: # (Image References)

[inputimage]: ./test_images/solidWhiteRight.jpg
[grayimage]: ./examples/md_gray_image.jpg
[edge_image]: ./examples/md_edge_image.jpg
[masked_edges]: ./examples/md_masked_edges.jpg
[lines_edges]: ./examples/md_lines_edges.jpg
[lines_images]: ./examples/md_lines_images.jpg
[video_output]: ./test_videos_output/solidWhiteRight.mp4

---

### Process images

## 1. Overview of the lane detection pipeline. 

Input: an image in RGB color.
Output: the same image with detected lanes marked in red.

I used the following techniques for lane detection:

`cv2.cvtColor` convert RGB image to grayscale <br>
`cv2.GaussianBlur` to prepare the image for the "gradient" step<br>
`cv2.Canny` detect edges<br>
`cv2.fillPoly` create a mask for region of interest <br>
`cv2.HoughLinesP` to detect lines using Hough transform

## 2. Detailed introduction of the lane detection pipeline. 

### Input image
Here is an example input image:
![alt text][inputimage]


### Convert to grayscale
`cv2.cvtColor` convert RGB image to grayscale using the code 
```
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
```


![alt text][grayimage]


### Calculate edges
To get the edge image, I first filtered the image using `cv2.GaussianBlur`, filter size 5. <br>
Then, `cv2.Canny` is used to detect edges, with tuned low_threshold and high_threshold. 
```
kernel_size=5
img_blur = gaussian_blur(gray, kernel_size)

low_threshold = 40
high_threshold = 100
edges = cv2.Canny(img_blur, low_threshold, high_threshold)
```
Output image looks like:
![alt text][edge_image]
We observe that after doing the gradient, the lanes are detected as there are large changes in the color value, which means a large gradient. 


### Detect lanes
We first create a mask to constrain the region of interest.
```
ignore_mask_color = 255
imshape = image.shape
mask = np.zeros_like(edges) 

vertices = np.array([[(0,imshape[0]),(imshape[1]*0.45, 0.6*imshape[0]), (imshape[1]*0.55, 0.6*imshape[0]), (imshape[1],imshape[0])]], dtype=np.int32)
cv2.fillPoly(mask, vertices, ignore_mask_color)
masked_edges = cv2.bitwise_and(edges, mask)
```
![alt_text][masked_edges]


`cv2.HoughLinesP` points on the same line are converted to a single point after Hough transform. With this technique, finding the intersection points in the Hough space will give us points in the x-y space that locates on the same line. <br>
Each element in `cv2.HoughLinesP` output is the (x1,y1, x2, y2) of each line. <br>
```
rho = 1 # distance resolution in pixels of the Hough grid
theta = 1*np.pi/180 # angular resolution in radians of the Hough grid
threshold = 30     # minimum number of votes (intersections in Hough grid cell)
min_line_length = 10 #minimum number of pixels making up a line
max_line_gap = 10    # maximum gap in pixels between connectable line segments
line_image = np.copy(image)*0 # creating a blank to draw lines on

# Run Hough on edge detected image
# Output "lines" is an array containing endpoints of detected line segments
lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),
                            min_line_length, max_line_gap)

```
Plotting the detected lines on the edge image:
![alt_text][lines_edges]


### Extrapolate to show the whole lanes
We want to create a single line for a lane detected, instead of only showing line segments.<br>
To identify left / right lane, use the slope of the lines and group the ones based on sign.  <br>
Then, doing a weighted average within groups to get line functions, with higher weights on longer lines. Then, extrapolate inside the mask to draw the whole lane. 
```
def average_slope_intercept(lines):
    '''
    Get the line function by averaging
    '''
    left_lines    = [] # (slope, intercept)
    left_weights  = [] # (length,)
    right_lines   = [] # (slope, intercept)
    right_weights = [] # (length,)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2==x1:
                continue # ignore a vertical line
            slope = (y2-y1)/(x2-x1)
            intercept = y1 - slope*x1
            length = np.sqrt((y2-y1)**2+(x2-x1)**2)
            if slope < 0: # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    
    # add more weight to longer lines    
    left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
    right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
    
    return left_lane, right_lane
    
def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None
    
    slope, intercept = line
    
    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    
    return ((x1, y1), (x2, y2))
```



Plot the lanes on the original image:
![alt_text][lines_images]


### Process videos

After constructing the pipeline for processing images, we can just apply it to videos. We need the package `VideoFileClip` from `moviepy.editor`.
```
clip1 = VideoFileClip("test_videos/solidWhiteRight.mp4") # this imports the video
white_clip = clip1.fl_image(process_image) # process images in the video using function process_image
```
Here is an example video output:
[solidWhiteRight.mp4](./test_videos_output/solidWhiteRight.mp4).


---
## Conclusion

This project builds a pipeline that identifies the left and right lanes from images and returns an annotated version. The lane detector works fine with most straight, clearly visible, solid or dashed yellow or white lanes.

One future improvement for this lane detector is to make it applicable to curved lanes. The current version does not work since it is using slopes of lines to identify whether it is a left lane or right lane, and averaging the slopes to get a single straight line for both sides. Due to this reason, the current pipeline does not work on the challenge video. 

Another future improvement is to make the mask adaptive. Currently the mask position is tuned for the region of interest, which works fine for none-hilly roads. For hilly roads, we need to first identify the horizontal line between the road and the sky, and then create a mask based on that. 