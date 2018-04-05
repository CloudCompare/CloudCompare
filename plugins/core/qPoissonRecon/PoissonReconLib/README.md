<CENTER><H2>Screened Poisson Surface Reconstruction <BR>(and Smoothed Signed Distance Reconstruction)<BR>Version 9.01</H2></CENTER>
<CENTER>
<A HREF="#LINKS">links</A>
<A HREF="#EXECUTABLES">executables</A>
<A HREF="#USAGE">usage</A>
<A HREF="#CHANGES">changes</A>
<A HREF="#SUPPORT">support</A>
</CENTER>
<HR>
<A NAME="LINKS"><B>LINKS</B></A><br>
<UL>
<B>Papers:</B>
<A href="http://www.cs.jhu.edu/~misha/MyPapers/SGP06.pdf">[Kazhdan, Bolitho, and Hoppe, 2006]</A>,
<A href="http://mesh.brown.edu/ssd/paper.html">[Calakli and Taubin, 2011]</A>,
<A href="http://www.cs.jhu.edu/~misha/MyPapers/ToG13.pdf">[Kazhdan and Hoppe, 2013]</A>
<br>
<B>Executables: </B>
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version9.01/PoissonRecon.x64.zip">Win64</A><BR>
<B>Source Code:</B>
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version9.01/PoissonRecon.zip">ZIP</A> <A HREF="https://github.com/mkazhdan/PoissonRecon">GitHub</A><BR>
<B>Older Versions:</B>
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version9.0/">V9.0</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version8.0/">V8.0</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/">V7.0</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.13a/">V6.13a</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.13/">V6.13</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.12/">V6.12</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.11/">V6.11</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.1/">V6.1</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6/">V6</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.71/">V5.71</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.6/">V5.6</A>,
<A href="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.5a/">V5.5a</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.1/">V5.1</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5/">V5</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version4.51/">V4.51</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version4.5/">V4.5</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version4/">V4</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version3/">V3</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version2/">V2</A>,
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version1/">V1</A>
</UL>
<HR>
<A NAME="EXECUTABLES"><B>EXECUTABLES</B></A><BR>
<UL>
<DL>
<FONT SIZE="+1" ><B><A HREF="" ONCLICK="toggleAll( $('poisson_recon') ) ; return false;" >PoissonRecon</A></B></FONT>
<DIV ID="poisson_recon">
<DT><b>--in</b> &#60;<i>input points</i>&#62;
<DD> This string is the name of the file from which the point set will be read.<br>
If the file extension is <i>.ply</i>, the file should be in
<A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format, giving the list of oriented
vertices with the x-, y-, and z-coordinates of the positions encoded by the properties <i>x</i>, <i>y</i>, and
<i>z</i> and the x-, y-, and z-coordinates of the normals encoded by the properties <i>nx</i>, <i>ny</i>, and
<i>nz</i> .<br>
If the file extension is <i>.bnpts</i>, the file should be a binary file, consisting of blocks of 6 32-bit
floats: x-, y-, and z-coordinates of the point's position, followed by the x-, y-, and z-coordinates
of the point's normal. (No information about the number of oriented point samples should be specified.)<br>
Otherwise, the file should be an ascii file with groups of 6,
white space delimited, numbers: x-, y-, and z-coordinates of the point's position, followed
by the x-, y- and z-coordinates of the point's normal. (No information about the number of oriented point samples should be specified.)<br> 

<DT>[<b>--out</b> &#60;<i>output triangle mesh</i>&#62;]
<DD> This string is the name of the file to which the triangle mesh will be written. 
The file is written in <A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format.

<DT>[<b>--voxel</b> &#60;<i>output voxel grid</i>&#62;]
<DD> This string is the name of the file to which the sampled implicit function will be written.
The filw is wrtten out in binary, with the first 4 bytes corresponding to the (integer) sampling resolution, 2^<i>d</i>,
and the next 4 x 2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i> bytes corresponding to the (single precision) floating point values
of the implicit function.

<DT>[<b>--degree</b> &#60;<i>B-spline degree</i>&#62;]
<DD> This integer specifies the degree of the B-spline that is to be used to define the finite elements system.
Larger degrees support higher order approximations, but come at the cost of denser system matrices (incurring a cost in both space and time).<BR>
The default value for this parameter is 2.

<DT>[<b>--bType</b> &#60;<i>boundary type</i>&#62;]
<DD> This integer specifies the boundary type for the finite elements. Valid values are:
<UL>
<LI> <B>1</B>: Free boundary constraints
<LI> <B>2</B>: Dirichlet boundary constraints
<LI> <B>3</B>: Neumann boundary constraints
</UL>
The default value for this parameter is 3 (Neumann).

<DT>[<b>--depth</b> &#60;<i>reconstruction depth</i>&#62;]
<DD> This integer is the maximum depth of the tree that will be used for surface reconstruction.
Running at depth <i>d</i> corresponds to solving on a voxel grid whose resolution is no larger than
2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i>. Note that since the reconstructor adapts the octree to the
sampling density, the specified reconstruction depth is only an upper bound.<br>
The default value for this parameter is 8.

<DT>[<b>--scale</b> &#60;<i>scale factor</i>&#62;]
<DD> This floating point value specifies the ratio between the diameter of the cube used for reconstruction
and the diameter of the samples' bounding cube.<br>
The default value is 1.1.

<DT>[<b>--samplesPerNode</b> &#60;<i>minimum number of samples</i>&#62;]
<DD> This floating point value specifies the minimum number of sample points that should fall within an
octree node as the octree construction is adapted to sampling density. For noise-free samples, small values
in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may
be needed to provide a smoother, noise-reduced, reconstruction.<br>
The default value is 1.0.

<DT>[<b>--pointWeight</b> &#60;<i>interpolation weight</i>&#62;]
<DD> This floating point value specifies the importance that interpolation of the point samples
is given in the formulation of the screened Poisson equation.<br>
The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0.<br>
The default value for this parameter is 4.

<DT>[<b>--confidence</b>]
<DD> Enabling this flag tells the reconstructor to use the size of the normals as confidence information. When the flag
is not enabled, all normals are normalized to have unit-length prior to reconstruction.

<DT>[<b>--nWeights</b>]
<DD> Enabling this flag tells the reconstructor to use the size of the normals to modulate the interpolation weights. When the flag
is not enabled, all points are given the same weight.

<DT>[<b>--iters</b> &#60;<i>GS iters</i>&#62;]
<DD> This integer value specifies the number of Gauss-Seidel relaxations to be performed at each level of the hiearchy.<br>
The default value for this parameter is 8.

<DT>[<b>--cgDepth</b> &#60;<i>conjugate gradients solver depth</i>&#62;]
<DD> This integer is the depth up to which a conjugate-gradients solver will be used to solve the linear system. Beyond this depth Gauss-Seidel relaxation will be used.<br>
The default value for this parameter is 0.

<DT>[<b>--fullDepth</b> &#60;<i>adaptive octree depth</i>&#62;]
<DD> This integer specifies the depth beyond depth the octree will be adapted.
At coarser depths, the octree will be complete, containing all 2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i> nodes.<br>
The default value for this parameter is 5.

<DT>[<b>--voxelDepth</b> &#60;<i>voxel sampling depth</i>&#62;]
<DD> This integer is the depth of the regular grid over which the implicit function is to be sampled.
Running at depth <i>d</i> corresponds to sampling on a voxel grid whose resolution is 2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i>.<br>
The default value for this parameter is the value of the <B>--depth</B> parameter.

<DT>[<b>--primalVoxel</b>]
<DD> Enabling this flag when outputing to a voxel file has the reconstructor sample the implicit function at the corners of the grid, rather than the centers of the cells.

<DT>[<b>--color</b> &#60;<i>pull factor</i>&#62;]
<DD> If specified, the reconstruction code assumes that the input is equipped with colors and will extrapolate
the color values to the vertices of the reconstructed mesh. The floating point value specifies the relative importance
of finer color estimates over lower ones. (In practice, we have found that a pull factor of 16 works well.)

<DT>[<b>--density</b>]
<DD> Enabling this flag tells the reconstructor to output the estimated depth values of the iso-surface vertices.

<DT>[<b>--linearFit</b>]
<DD> Enabling this flag has the reconstructor use linear interpolation to estimate the positions of iso-vertices.

<DT>[<b>--polygonMesh</b>]
<DD> Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating the results of Marching Cubes).

<DT>[<b>--threads</b> &#60;<i>number of processing threads</i>&#62;]
<DD> This integer specifies the number of threads across which the reconstruction
algorithm should be parallelized.<br>
The default value for this parameter is equal to the numer of (virtual) processors on the executing  machine.

<DT>[<b>--verbose</b>]
<DD> Enabling this flag provides a more verbose description of the running times and memory usages of
individual components of the surface reconstructor.
</DIV>
</DL>
</UL>


<UL>
<DL>
<FONT SIZE="+1"><B><A HREF="" ONCLICK="toggleAll( $('ssd_recon') ) ; return false;">SSDRecon</A></B></FONT>
<DIV ID="ssd_recon">
<DT><b>--in</b> &#60;<i>input points</i>&#62;
<DD> This string is the name of the file from which the point set will be read.<br>
If the file extension is <i>.ply</i>, the file should be in
<A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format, giving the list of oriented
vertices with the x-, y-, and z-coordinates of the positions encoded by the properties <i>x</i>, <i>y</i>, and
<i>z</i> and the x-, y-, and z-coordinates of the normals encoded by the properties <i>nx</i>, <i>ny</i>, and
<i>nz</i> .<br>
If the file extension is <i>.bnpts</i>, the file should be a binary file, consisting of blocks of 6 32-bit
floats: x-, y-, and z-coordinates of the point's position, followed by the x-, y-, and z-coordinates
of the point's normal. (No information about the number of oriented point samples should be specified.)<br>
Otherwise, the file should be an ascii file with groups of 6,
white space delimited, numbers: x-, y-, and z-coordinates of the point's position, followed
by the x-, y- and z-coordinates of the point's normal. (No information about the number of oriented point samples should be specified.)<br> 

<DT>[<b>--out</b> &#60;<i>output triangle mesh</i>&#62;]
<DD> This string is the name of the file to which the triangle mesh will be written. 
The file is written in <A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format.

<DT>[<b>--voxel</b> &#60;<i>output voxel grid</i>&#62;]
<DD> This string is the name of the file to which the sampled implicit function will be written.
The filw is wrtten out in binary, with the first 4 bytes corresponding to the (integer) sampling resolution, 2^<i>d</i>,
and the next 4 x 2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i> bytes corresponding to the (single precision) floating point values
of the implicit function.

<DT>[<b>--degree</b> &#60;<i>B-spline degree</i>&#62;]
<DD> This integer specifies the degree of the B-spline that is to be used to define the finite elements system.
Larger degrees support higher order approximations, but come at the cost of denser system matrices (incurring a cost in both space and time).<BR>
The default value for this parameter is 2.

<DT>[<b>--depth</b> &#60;<i>reconstruction depth</i>&#62;]
<DD> This integer is the maximum depth of the tree that will be used for surface reconstruction.
Running at depth <i>d</i> corresponds to solving on a voxel grid whose resolution is no larger than
2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i>. Note that since the reconstructor adapts the octree to the
sampling density, the specified reconstruction depth is only an upper bound.<br>
The default value for this parameter is 8.

<DT>[<b>--scale</b> &#60;<i>scale factor</i>&#62;]
<DD> This floating point value specifies the ratio between the diameter of the cube used for reconstruction
and the diameter of the samples' bounding cube.<br>
The default value is 1.1.

<DT>[<b>--samplesPerNode</b> &#60;<i>minimum number of samples</i>&#62;]
<DD> This floating point value specifies the minimum number of sample points that should fall within an
octree node as the octree construction is adapted to sampling density. For noise-free samples, small values
in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may
be needed to provide a smoother, noise-reduced, reconstruction.<br>
The default value is 1.0.

<DT>[<b>--valueWeight</b> &#60;<i>zero-crossing interpolation weight</i>&#62;]
<DD> This floating point value specifies the importance that interpolation of the point samples
is given in the formulation of the screened Smoothed Signed Distance Reconstruction.<br>
The default value for this parameter is 4.

<DT>[<b>--gradientWeight</b> &#60;<i>normal interpolation weight</i>&#62;]
<DD> This floating point value specifies the importance that interpolation of the points' normals
is given in the formulation of the screened Smoothed Signed Distance Reconstruction.<br>
The default value for this parameter is 0.001.

<DT>[<b>--biLapWeight</b> &#60;<i>bi-Laplacian weight weight</i>&#62;]
<DD> This floating point value specifies the importance that the bi-Laplacian regularization
is given in the formulation of the screened Smoothed Signed Distance Reconstruction.<br>
The default value for this parameter is 0.00001.

<DT>[<b>--confidence</b>]
<DD> Enabling this flag tells the reconstructor to use the size of the normals as confidence information. When the flag
is not enabled, all normals are normalized to have unit-length prior to reconstruction.

<DT>[<b>--nWeights</b>]
<DD> Enabling this flag tells the reconstructor to use the size of the normals to modulate the interpolation weights. When the flag
is not enabled, all points are given the same weight.

<DT>[<b>--iters</b> &#60;<i>GS iters</i>&#62;]
<DD> This integer value specifies the number of Gauss-Seidel relaxations to be performed at each level of the hiearchy.<br>
The default value for this parameter is 8.

<DT>[<b>--cgDepth</b> &#60;<i>conjugate gradients solver depth</i>&#62;]
<DD> This integer is the depth up to which a conjugate-gradients solver will be used to solve the linear system. Beyond this depth Gauss-Seidel relaxation will be used.<br>
The default value for this parameter is 0.

<DT>[<b>--fullDepth</b> &#60;<i>adaptive octree depth</i>&#62;]
<DD> This integer specifies the depth beyond depth the octree will be adapted.
At coarser depths, the octree will be complete, containing all 2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i> nodes.<br>
The default value for this parameter is 5.

<DT>[<b>--voxelDepth</b> &#60;<i>voxel sampling depth</i>&#62;]
<DD> This integer is the depth of the regular grid over which the implicit function is to be sampled.
Running at depth <i>d</i> corresponds to sampling on a voxel grid whose resolution is 2^<i>d</i> x 2^<i>d</i> x 2^<i>d</i>.<br>
The default value for this parameter is the value of the <B>--depth</B> parameter.

<DT>[<b>--primalVoxel</b>]
<DD> Enabling this flag when outputing to a voxel file has the reconstructor sample the implicit function at the corners of the grid, rather than the centers of the cells.

<DT>[<b>--color</b> &#60;<i>pull factor</i>&#62;]
<DD> If specified, the reconstruction code assumes that the input is equipped with colors and will extrapolate
the color values to the vertices of the reconstructed mesh. The floating point value specifies the relative importance
of finer color estimates over lower ones. (In practice, we have found that a pull factor of 16 works well.)

<DT>[<b>--density</b>]
<DD> Enabling this flag tells the reconstructor to output the estimated depth values of the iso-surface vertices.

<DT>[<b>--nonLinearFit</b>]
<DD> Enabling this flag has the reconstructor use quadratic interpolation to estimate the positions of iso-vertices.

<DT>[<b>--polygonMesh</b>]
<DD> Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating the results of Marching Cubes).

<DT>[<b>--threads</b> &#60;<i>number of processing threads</i>&#62;]
<DD> This integer specifies the number of threads across which the reconstruction
algorithm should be parallelized.<br>
The default value for this parameter is equal to the numer of (virtual) processors on the executing  machine.

<DT>[<b>--verbose</b>]
<DD> Enabling this flag provides a more verbose description of the running times and memory usages of
individual components of the surface reconstructor.
</DIV>
</DL>
</UL>


<UL>
<DL>
<FONT SIZE="+1"><B><A HREF="" ONCLICK="toggleAll( $('surface_trimmer') ) ; return false;">SurfaceTrimmer</A></B></FONT>
<DIV ID="surface_trimmer">
<DT><b>--in</b> &#60;<i>input triangle mesh</i>&#62;
<DD> This string is the name of the file from which the triangle mesh will be read. 
The file is read in <A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format and it is assumed that the vertices have a <I>value</I> field which stores the signal's value. (When run with <B>--density</B> flag, the reconstructor will output this field with the mesh vertices.)

<DT><b>--trim</b> &#60;<i>trimming value</i>&#62;
<DD> This floating point values specifies the value for mesh trimming. The subset of the mesh with signal value less than the trim value is discarded.

<DT>[<b>--out</b> &#60;<i>output triangle mesh</i>&#62;]
<DD> This string is the name of the file to which the triangle mesh will be written. 
The file is written in <A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format.

<DT>[<b>--smooth</b> &#60;<i>smoothing iterations</i>&#62;]
<DD> This integer values the number of umbrella smoothing operations to perform on the signal before trimming.<BR>
The default value is 5.

<DT>[<b>--aRatio</b> &#60;<i>island area ratio</i>&#62;]
<DD> This floating point value specifies the area ratio that defines a disconnected component as an "island". Connected components whose area, relative to the total area of the mesh, are smaller than this value will be merged into the output surface to close small holes, and will be discarded from the output surface to remove small disconnected components.<BR>
The default value 0.001.

<DT>[<b>--polygonMesh</b>]
<DD> Enabling this flag tells the trimmer to output a polygon mesh (rather than triangulating the trimming results).
</DIV>
</DL>
</UL>

<HR>
<A NAME="USAGE"><B>USAGE</B></A><br>
For testing purposes, three point sets are provided:
<OL>

<LI> <A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.points.ply"><B>Eagle</B></A>:
A set of 796,825 oriented point samples with color (represented in PLY format) was obtained in the EPFL <A HREF="http://lgg.epfl.ch/statues.php">Scanning 3D Statues from Photos</A> course.<br>
<UL>
<LI>The original Poisson Reconstruction algorithm can be invoked by calling:
<BLOCKQUOTE><CODE>% PoissonRecon --in eagle.points.ply --out eagle.unscreened.ply --depth 10 --pointWeight 0</CODE></BLOCKQUOTE>
using the <b>--pointWeight 0</b> argument to disable the screening.<br>

<LI>By default, screening is enabled so the call:
<BLOCKQUOTE><CODE>% PoissonRecon --in eagle.points.ply --out eagle.screened.ply --depth 10</CODE></BLOCKQUOTE>
produces a reconstruction that more faithfully fits the input point positions.<BR>

<LI> A reconstruction of the eagle that extrapolates the color values from the input samples can be obtained by calling:
<BLOCKQUOTE><CODE>% PoissonRecon --in eagle.points.ply --out eagle.screened.color.ply --depth 10 --color 16</CODE></BLOCKQUOTE>
using the <b>--color 16</b> to indicate both that color should be used, and the extent to which finer color estimates should be preferenced over coarser estimates.

<LI> Finally, a reconstruction the eagle that does not close up the holes can be obtained by first calling:
<BLOCKQUOTE><CODE>% PoissonRecon --in eagle.points.ply --out eagle.screened.color.ply --depth 10 --color 16 --density</CODE></BLOCKQUOTE>
using the <B>--density</B> flag to indicate that density estimates should be output with the vertices of the mesh, and then calling:
<BLOCKQUOTE><CODE>% SurfaceTrimmer --in eagle.screened.color.ply --out eagle.screened.color.trimmed.ply --trim 7</CODE></BLOCKQUOTE>
to remove all subsets of the surface where the sampling density corresponds to a depth smaller than 7.
</UL>
<TABLE BORDER=1>
<TR>
<TD WIDTH="25%"><TABLE><TR><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.unscreened.1.jpg" WIDTH="100%"><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.unscreened.2.jpg" WIDTH="100%"></TR></TABLE>
<TD WIDTH="25%"><TABLE><TR><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.1.jpg" WIDTH="100%"><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.2.jpg" WIDTH="100%"></TR></TABLE>
<TD WIDTH="25%"><TABLE><TR><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.1.color.jpg" WIDTH="100%"><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.2.color.jpg" WIDTH="100%"></TR></TABLE>
<TD WIDTH="25%"><TABLE><TR><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.1.color.trimmed.jpg" WIDTH="100%"><TD WIDTH="50%"><IMG SRC="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/eagle.2.color.trimmed.jpg" WIDTH="100%"></TR></TABLE>
</TR>
<TR>
<TH>Unscreened
<TH>Screened
<TH>Screened + Color
<TH>Screened + Color + Trimmed
</TR>
</TABLE>

<LI> <A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/bunny.points.ply"><B>Bunny</B></A>:
A set of 362,271 oriented point samples (represented in PLY format) was obtained by merging the data from the original Stanford Bunny
<A HREF="ftp://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz">range scans</A>. The orientation of the sample points was estimated
using the connectivity information within individual range scans.<br>
The surface of the model can be reconstructed by calling the surface reconstructor as follows:
<BLOCKQUOTE><CODE>% PoissonRecon --in bunny.points.ply --out bunny.ply --depth 10</CODE></BLOCKQUOTE>

<LI> <A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/horse.npts"><B>Horse</B></A>:
A set of 100,000 oriented point samples (represented in ASCII format) was obtained by sampling a virtual horse model with a sampling density proportional to curvature, giving a set of non-uniformly distributed points.<br>
The surface of the model can be reconstructed by calling the surface reconstructor as follows:
<BLOCKQUOTE><CODE>% PoissonRecon --in horse.npts --out horse.ply --depth 10</CODE></BLOCKQUOTE>

</OL>

To convert the binary <A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format to
<A HREF="http://research.microsoft.com/~hoppe/">Hugues Hoppe's</A> ASCII
mesh format, a <A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/ply2mesh">Perl script</A> is provided.<br>
As an examples, the reconstructed bunny can be converted into the ASCII mesh format as follows:
<BLOCKQUOTE><CODE>% ply2mesh.pl bunny.ply &gt; bunny.m</CODE></BLOCKQUOTE>

<HR>
<A NAME="CHANGES"><B>CHANGES</B></A><br>
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version3/">Version 3</A>:
<OL>
<LI> The implementation of the <b>--samplesPerNode</b> parameter has been modified so that a value of "1" more closely corresponds to a distribution with one sample per leaf node.
<LI> The code has been modified to support compilation under MSVC 2010 and the associated solution and project files are now provided. (Due to a bug in the Visual Studios compiler, this required modifying the implementation of some of the bit-shifting operators.)
</OL>
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version4/">Version 4</A>:
<OL>
<LI> The code supports screened reconstruction, with interpolation weight specified through the <b>--pointWeight</b> parameter.
<LI> The code has been implemented to support parallel processing, with the number of threads used for parallelization specified by the <b>--threads</b> parameter.
<LI> The input point set can now also be in <A HREF="http://www.cc.gatech.edu/projects/large_models/ply.html">PLY</A> format, and the file-type is determined by the extension, so that the <b>--binary</b> flag is now obsolete.
<LI> At depths coarser than the one specified by the value <b>--minDepth</b> the octree is no longer adaptive but rather complete, simplifying the prolongation operator.
</OL>
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version4.5/">Version 4.5</A>:
<OL>
<LI> The algorithmic complexity of the solver was reduced from log-linear to linear.
</OL>
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version4.5/">Version 4.51</A>:
<OL>
<LI> Smart pointers were added to ensure that memory accesses were in bounds.
</OL>
<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5/">Version 5</A>:
<OL>
<LI> The <B>--density</B> flag was added to the reconstructor to output the estimated depth of the iso-vertices.
<LI> The <I>SurfaceTrimmer</I> executable was added to support trimming off the subset of the reconstructed surface that are far away from the input samples, thereby allowing for the generation of non-water-tight surface.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.1/">Version 5.1</A>:
<OL>
<LI> Minor bug-fix to address incorrect neighborhood estimation in the octree finalization.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.5a/">Version 5.5a</A>:
<OL>
<LI> Modified to support depths greater than 14. (Should work up to 18 or 19 now.)
<LI> Improved speed and memory performance by removing the construction of integral and value tables.
<LI> Fixed a bug in Version 5.5 that used memory and took more time without doing anything useful.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.6/">Version 5.6</A>:
<OL>
<LI> Added the <b>--normalWeight</b> flag to support setting a point's interpolation weight in proportion to the magnitude of its normal.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.7/">Version 5.7</A>:
<OL>
<LI> Modified the setting of the constraints, replacing the map/reduce implementation with OpenMP atomics to reduce memory usage.
<LI> Fixed bugs that caused numerical overflow when processing large point clouds on multi-core machines.
<LI> Improved efficiency of the iso-surface extraction phse.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.71/">Version 5.71</A>:
<OL>
<LI> Added the function <I>GetSolutionValue</I> to support the evaluation of the implicit function at a specific point.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6/">Version 6</A>:
<OL>
<LI> Modified the solver to use Gauss-Seidel relaxation instead of conjugate-gradients at finer resolution.
<LI> Re-ordered the implementation of the solver so that only a windowed subset of the matrix is in memory at any time, thereby reducing the memory usage during the solver phase.
<LI> Separated the storage of the data associated with the octree nodes from the topology.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.1/">Version 6.1</A>:
<OL>
<LI> Re-ordered the implementation of the iso-surface extraction so that only a windowed subset of the octree is in memory at any time, thereby reducing the memory usage during the extracted phase.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.11/">Version 6.11</A>:
<OL>
<LI> Fixed a bug that created a crash in the evaluation phase when <b>--pointWeight</b> is set zero.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.12/">Version 6.12</A>:
<OL>
<LI> Removed the OpenMP <I>firstprivate</I> directive as it seemed to cause trouble under Linux compilations.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.13/">Version 6.13</A>:
<OL>
<LI> Added a <B>MemoryPointStream</B> class in <I>PointStream.inl</I> to support in-memory point clouds.
<LI> Modified the signature of <U>Octree::SetTree</U> in <I>MultiGridOctreeData.h</I> to take in a pointer to an object of type <B>PointStream</B> rather than a file-name.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version6.13a/">Version 6.13a</A>:
<OL>
<LI> Modified the signature of <U>Octree::SetIsoSurface</U> to rerun a <I>void</I>. [<A HREF="http://www.danielgm.net/cc/">cloudcompare</A>]
<LI> Added a definition of <U>SetIsoVertexValue</U> supporting double precision vertices. [<A HREF="http://www.danielgm.net/cc/">cloudcompare</A>]
<LI> Removed <I>Time.[h/cpp]</I> from the repository. [<A HREF="http://www.danielgm.net/cc/">cloudcompare</A>/<A HREF="http://asmaloney.com/">asmaloney</A>]
<LI> Fixed assignment bug in <U>Octree::SetSliceIsoVertices</U>. [<A HREF="http://asmaloney.com/">asmaloney</A>]
<LI> Fixed initialization bug in <U>SortedTreeNodes::SliceTableData</U> and <U>SortedTreeNodes::XSliceTableData</U>. [<A HREF="http://asmaloney.com/">asmaloney</A>]
<LI> Included <I>stdlib.h</I> in <I>Geometry.h</I>. [<A HREF="http://asmaloney.com/">asmaloney</A>]
<LI> Fixed default value bug in declaration of <U>Octree::SetTree</U>. [<A HREF="http://asmaloney.com/">asmaloney</A>]
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/">Version 7.0</A>:
<OL>
<LI> Added functionality to support color extrapolation if present in the input.
<LI> Modified a bug with the way in which sample contributions were scaled.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version8.0/">Version 8.0</A>:
<OL>
<LI> Added support for different degree B-splines.
(Note that as the B-spline degree is a template parameter, only degree 1 through 4 are supported.
If higher order degrees are desired, additional template parameters can be easily added in the body of the <U>Execute</U> function inside of <I>PoissonRecon.cpp</I>.
Similarly, to reduce compilation times, support for specific degrees can be removed.)
<LI> Added the <B>--primalVoxel</B> flag to support to extraction of a voxel grid using primal sampling.
<LI> Changed the implementation of the voxel sampling so that computation is now linear, rather than log-linear, in the number of samples.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version9.0/">Version 9.0</A>:
<OL>
<LI> Added support for free boundary conditions.
<LI> Extended the solver to support more general linear systems. This makes it possible to use the same framework to implement the <A HREF="http://mesh.brown.edu/ssd/">Smoothed Signed Distance Reconstruction</A> of Calakli and Taubin (2011).
<LI> Modified the implementation of density estimation and input representation. This tends to define a slightly larger system. On its own, this results in slightly increased running-time/footprint for full-res reconstructions, but provides a substantially faster implementation when the output complexity is smaller than the input.
</OL>

<A HREF="http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version9.01/">Version 9.01</A>:
<OL>
<LI> Reverted the density estimation to behave as in Version 8.0.
</OL>

<HR>
<A NAME="SUPPORT"><B>SUPPORT</B></A><br>
This work genersouly supported by NSF grants #0746039 and #1422325.

<HR>
<A HREF="http://www.cs.jhu.edu/~misha">HOME</A>
