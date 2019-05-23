Project 07: Kalman based IMU SLAM

The dataset is composed by a single g2o file.
You should use the IMU as prediction and the point sensor as update.

With respect to the g2o file we have seen during the lectures, this dataset (g2o file) contains:

	PARAMS_SE3OFFSET 0 0 0 0.3 0.5 -0.5 0.5 -0.5 
	that describes the position of the sensor #0 (PointSensor) on the robot. [sensor_id x y z qx qy qz qw]
	
	PARAMS_SE3OFFSET 1 0 0 0 0 0 0 1  
	that describes the position of the sensor #1 (IMU) on the robot. [sensor_id x y z qx qy qz qw]

	VERTEX_TRACKXYZ 7 -5.07426 -12.3804 0.492881 
	that represents a landmark with Id=7, 3d position. [vertex_id x y z]

	VERTEX_SE3:QUAT 1000 0 0 0 0 0 0 1 
	that represent a pose with Id=1000, 3d pose. [vertex_id x y z qx qy qz qw]

	EDGE_SE3_TRACKXYZ 2001 779 0 0.422149 0.412126 1.41482 1000 0 0 1000 0 10
	that represents an observation made from pose 2001 of landmark 779, sensor id 0, x y z and covariance (upper triangular block, row major). [vertex_id vertex_id sensor_id x y z covariance]

	EDGE_SE3_PRIOR 1001 1 -0.004629 -0.0042542 0.14567 -0.0381847 0.0455061 -0.710633 0.701051 1000 0 0 0 0 0 1000 0 0 0 0 10 0 0 0 1000 0 0 1000 0 1000
	that represents an absolute prior about the position with id 1001, sensor id 1, pose, covariance (upper triangular block, row major). [vertex_id sensor_id x y z qx qy qz qw covariance]

Expected Output:
Trajectory of the robot and Map (landmarks)

