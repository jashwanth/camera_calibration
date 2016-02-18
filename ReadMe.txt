---------------------------------------------
Images and corresponding indices
---------------------------------------------
- "list.txt" lists out the image names in order
- the image indices range from 0-8

--------------------------------------------
World Coordinates for the object
--------------------------------------------
- "WorldPoints.txt" lists out the 3D coordinates of 2055 world points corresponding to the images

--------------------------------------------
Image Points and corresponding world points
--------------------------------------------
- "ImageDetails.txt" has the points for each image
- format: 
	imgNo1 nPoints1
	a b x y z
	.
	.
	.
	imgNo2 nPoints2
	a b x y z
	.
	.
	.
	.

- (a,b) is the coordinate of the point in image <imgNo#>
- (x,y,z) is the 3D world coordinate for the same point.


--------------------------------------------
How to use this for the assignment
--------------------------------------------
- Run your camera calibration methods on two or more images.
- You could choose any number of point correspondences, unless explicitly mentioned in the assignment task.

