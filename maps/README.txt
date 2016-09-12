This file describes the "map.txt" files

map.txt:
It is a matrix of numbers that represents the robots visibility, '-1' means not visible,
'0' means visible and any other positive number means visible and also a radio in mm.
NOTE: First column is the ID number.

The matrix size is  Nx(N+1) matrix, with N number of robots,
the i colon represents the robot of the row i-1.

	           can see
	       Robot0 Robot12 Robot8
Robot0	  10      0      -1
Robot12   -1      2      -1
Robot8     0     -1       0

It is read like:
Robot0 can see Robot0 position and objects in a radio of 10mm from Robot0
Robot0 can see Robot12 position
Robot0 cannot see Robot8 position

Robot12 cannot see Robot0 position
Robot12 can see Robot12 position and objects in a radio of 2mm from Robot12
Robot12 cannot see Robot8 position

Robot8 can see Robot0 position
Robot8 cannot see Robot12 position
Robot8 can see Robot8 position

example:
0,10,0,-1
12,-1,2,-1
8,0,-1,0
