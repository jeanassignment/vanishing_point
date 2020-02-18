
## Algorithm Step Alterations
Steps being altered are below:

4. Improve line merging method, merge_line_angle parameter become less sensitive
6. Instead of <del>Calculate the intersections points' residual (accumulated ortho-distance to all lines)</del> , Let lines vote for the points, if lines lie in the vicinity of the points
7. Sort point with votes

In addition:

* Alter structure of Line and Point Class, to keep track of vanishing points and lines agrees to it.
   * This make sure each line belong to its closest vanishing point


## Parameters

|Hough Transform| Value | Description|
| --- | --- |---|
|hough_threshold| 150|[number of intersect] minimum number to agree to a line |
|min_line_len | <del>100</del> 50  |[px] minimum detected length of line, bigger to avoid noise|
|max_line_gap | 10   |[px] max gap to differentiate lines, higher to get more continuous lines, slower|


| Merge Line| Value | Description|
| --- | --- |---|
|vertical_deg |85| [deg] filter out line angle bigger than this|
|merge_line_ang | <del>0.005</del> 0.5| [deg] intersect smaller than this angle |
|merge_line_dist |<del>3</del> 2| [px] orthogonal dist smaller than this distance |

|Vanishing Point| Value | Description|
| --- | --- |---|
|vicinity | <del>10</del> 5 | [px] points lies within this vicinity of a proposed point are same cluster|
| min_ratio | 100|[ratio of points] define vanishing points (cluster includes more than certain ratio of points), higher the less points|


## Results
<img src="https://github.com/jeanassignment/vanishing_point/blob/debug/result/result_5D4L1L1D_L.jpg" width=100%/>

<img src="https://github.com/jeanassignment/vanishing_point/blob/debug/result/result_5D4KVN2Y_R.jpg" width=100%/>

