# 16662-RA-Perception

1. Capture Pointcloud (2 seconds after start, or on command)
2. Extract 5x5 region - X (-0.2 to 0.2) and Y (0 to -0.4)
3. Register known pcd to captured pcd - to find object pose
   (temp - bowl center is at (0, -0.17))
4. Equation of a shape - square with s=0.22 offset to object origin
5. Project Square to find depth

