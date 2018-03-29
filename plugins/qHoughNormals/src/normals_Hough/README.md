# normals_Hough

Normal Estimation in Unstructured Point Clouds with Hough transform

# Paper

Please acknowledge our the reference paper :

"Deep Learning for Robust Normal Estimation in Unstructured Point Clouds " by Alexandre Boulch and Renaud Marlet, Symposium of Geometry Processing 2016, Computer Graphics Forum

# Code

The code for normal estimation is C++ hearder only. Three version are proposed, previous version are located in cgal/ en pcl/ relies on CGAL and PCL libriaries.
The current version Normals.h relies on Eigen and nanoflann (assumed to be in the source folder when compiling).

# Usage

HoughCNN_Exec [options] -i input_file.xyz -o output_file.xyz

# Author webpage

[Alexandre Boulch](https://sites.google.com/site/boulchalexandre)

# Previous version

Previous version of the code are located in folders cgal/ and pcl/
