
## Task Description
The goal of this assignment is to create a simple program to detect the **most important** vanishing points of an image.

1. This is a C++ assignment.
2. You have to make your own implementation of the vanishing point algorithm, but libraries are allowed for edge and line extraction, drawing, etc.
3. Example input files 5D4L1L1D_L.jpg and 5D4KVN2Y_r.jpg are included.
4. Not all the lines will lead to a vanishing point.
5. Describe how you deal with some of the uncertainties in your estimation, for instance the fact that not all lines intersect the vanishing point in exactly the same place.
6. The expected output of the program is a new image with the vanishing points (when inside the boundaries of the image) and lines colored according to the vanishing point they belong to.


## Algorithm introduction

The algorithm find candidate points, calculate its sum of orthogonal distance to lines, then find clusters of point. 

1. Preprocessing: original image --> grey image --> blurred image
2. Canny Edge Detection to find edges
3. Probabilistic Hough Transform to find line segments.
4. Merge Lines by evaluating their difference in angle and distance (thinning)
5. Calculate intersections of the lines
6. Calculate the intersections points' residual (accumulated ortho-distance to all lines), to see how much the lines votes for this point.
7. Sort points with residual (higher potential candidates will be sorted to in front)
8. Find Big Clusters of Candidate
	- Set first candidate as key point of a cluster
	- Include candidate points in the vicinity of key point into the cluster
	- If enough intersection points are grouped to this cluster, choose the key point as the first vanishing point.
	- Find the next ungrouped candidate as a key point of the new cluster, repeat above steps
9. If no candidate points available, stop and output. 

## Parameters


| Canny Edge | Value | Description|
| --- | --- |---|
| canny_lowthreshold  | 100  ||
| canny_highthreshold  | canny_lowthreshold * 3  | Canny’s recommendation 2-3*lower|


|Hough Transform| Value | Description|
| --- | --- |---|
|hough_threshold| 150|[number of intersect] minimum number to agree to a line |
|min_line_len | 100  |[px] minimum detected length of line, bigger to avoid noise|
|max_line_gap | 10   |[px] max gap to differentiate lines, higher to get more continuous lines, slower|


| Merge Line| Value | Description|
| --- | --- |---|
|vertical_deg |85| [deg] filter out line angle bigger than this|
|merge_line_ang | 0.005| [deg] intersect smaller than this angle |
|merge_line_dist | 3| [px] orthogonal dist smaller than this distance |

|Vanishing Point| Value | Description|
| --- | --- |---|
|vicinity | 10| [px] points lies within this vicinity of a proposed point are same cluster|
| min_ratio | 100|[ratio of points] define vanishing points (cluster includes more than certain ratio of points), higher the less points|

## Usage

```console
cmake . 
make
./vanishing_pt <image directory>
```

result will be stored in result folder

## Results
<img src="https://github.com/jeanassignment/vanishing_point/blob/master/result/result_5D4L1L1D_L.jpg" width=100%/>

<img src="https://github.com/jeanassignment/vanishing_point/blob/master/result/result_5D4KVN2Y_R.jpg" width=100%/>

