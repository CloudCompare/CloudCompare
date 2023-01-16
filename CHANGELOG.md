CloudCompare Version History
============================

v2.13.alpha (???) - (??/??/????)
----------------------
- - New features:
	- New command line option:
		- FLIP_TRI (to flip the order of the triangle vertices of all opened meshes)
		- SF_OP_SF {SF 1} {operation} {SF 2}
			- to compute an arithmetic operation between two scalar fields (add, sub, mult, div)
			- works on clouds and meshes
		- SF_INTERP {SF index} with sub-option -DEST_IS_FIRST
			- to interpolate a scalar field from one cloud to another cloud (use DEST_IS_FIRST if destination is first)
		- SF_ADD_CONST {SF name} {const value}
			- to add a constant scalar field to a cloud
		- CLEVELS
			- to edit the color bands histogram of a cloud or a mesh (https://www.cloudcompare.org/doc/wiki/index.php/Colors%5CLevels)
			- syntax: -CLEVELS BANDS INPUT_RANGE_MIN INPUT_RANGE_MAX OUTPUT_RANGE_MIN OUTPUT_RANGE_MAX
				- band can be any mix of 'R', 'G' and 'B' (ex: 'G' or 'RB' or 'RGB')
				- selected color component values will be scaled so that
					[INPUT_RANGE_MIN INPUT_RANGE_MAX] becomes [OUTPUT_RANGE_MIN OUTPUT_RANGE_MAX]
					(values ouside of the input range will also be scaled)
        - RDP {opt: min distance between points}
        	- removes duplicate points of all loaded clouds
			- the min distance between points to consider them as duplicated can be set (1e-8 by default)

	- New display feature: near and far clipping planes in 3D views
		- extension of the previously existing feature to set a near clipping plane
		- can be enabled and modifed in the Camera Parameters dialog or via
			CTRL + mouse wheel (near) or CTRL + SHIFT + mouse wheel (far)
		- the user shall now input actual distances and not percentages
		- works for all projection modes (orthographic and perspective)
		- does not modify the rendering quality of the EDL or SSAO shaders
		- taken into account when using interactive segmentation or point picking
		- not compatible with the Cross Section tool

	- ASCII files:
		- ability to load quaternion coordinates (qw, qx, qy and qz) that will be loaded as individual 'Coordinate System' objects
		- once 4 columns are assigned to one of the 4 quaternion components (w,x,y,z), the user can elect the display scale of the
			'Coordinate System' objects

	- New unified plugin to load LAS files (by Thomas Montaigu)
		- based on LASzip
		- should work on all platforms (Windows, Linux, macOS)
		- manages all versions of LAS files (1.0 to 1.4)
		- gives much more control over extended fields (EVLR) as well as custom mapping between
			the existing fields of a cloud and their destination in the LAS file

	- Display > Restore window geometry on startup
		- Option to control whether the previous window geometry (size and position) should be restored or not
	
	- Display > Display settings > Picking cursor ('Cross' or 'Pointing hand')
		- Default mouse cursor for picking points will now be the cross
		- This can be changed in the 'Display options' dialog

	- Edit > Normals > Shift points along normals
		- to shift the points of a given quantity along their associated normal

- Improvements:

	- Segmentation
		- CC will now preserve some children entities (labels, sensors, etc.) when segmenting clouds or meshes:
			with graphical segmentation, cross section tool (for clouds only), subsampling, filter by SF value, etc.

	- Rasterize:
		- various improvements and glitch fixes
		- if a scalar field projection option is chosen, all SF and colors will be projected at once so that all layers are valid right away
			(no need to 'update' the grid several times)
		- new projection options (inside each cell)
			- Median
			- Inverse variance: the height or scalar value of each cell is equal to a weighted average of the points falling inside the cell.
								The weights are equal to the inverse variance (= 1/std_dev^2). Standard deviation values (std_dev) are
								obtained from an already existing scalar field, which must be selected by the user)
		- statistics can now be computed on the altitudes and/or all scalar fields
		- new statistics: median, percentile (with custom percentage) and nuùber of unique values inside a cell
		- new command line sub-options (to be used with -PROJ or -SF_PROJ)
			- -MED 
			- -INV_VAR (+ std. dev. SF index or name)

	- Edit > Color > Set unique & Edit > Color > Colorize
		- CC will now remember the last input color

	- Global Shift:
		- the suggested global shift will now be rounded to the nearest thousand (so as to suggest values in kilometers, easier to manage)

	- PoissonRecon plugin:
		- the user can now choose the number of threads

	- Edit > Cloud > Paste from clipboard:
		- the shortcut has been changed to ALT+P (so as to not conflict with the already existing CTRL+P shortcut for the 'Trace polyline' tool)
		
	- Graphical Segmentation Tool:
		- the user can now configure the name suffix for the remaining and segmented cloud
		- the display should be faster until the first segmentation is performed
		- unused mesh vertices should now be properly removed from both the segmented and the remaining mesh parts

	- LAS files:
		- The default shift for (GPS) time values is now rounded to the nearest 10^5 value
		- The shift value is now displayed as a property of the currently selected scalar field
		- When using the standard LAS I/O filter (PDAL), the user can now set a custom shift for GPS time values

	- E57 files:
		- Images are now saved internally as JPEG files (faster and smaller)
		- Sensors associated to images should now be properly shifted if the associated cloud is shifted,
			or if they are not associated to a cloud

	- SF Arithmetic tool:
		- New operation: 'SET' (to set a constant value)
			- this operation is also accessible via the command line (with the -SF_OP command)
		- New operation: 'ABS' (to compute the absolue value)
			- this operation is also accessible via the command line (with the -SF_ARITHMETIC command)

	- Command line:
		- It is now possible to pass a SF name after -SET_ACTIVE_SF  instead of the field index
			(use simple quotes if the scalar field name has spaces in it)
		- The -SET_ACTIVE_SF command can now be applied to the loaded meshes
		- The -SF_COLOR_SCALE option now works on meshes (vertices) as well
		- The -FILTER_SF option now works on meshes as well
		- New sub-option for the -C2C_DIST and -C2M_DIST commands: -SPLIT_XY_Z to split the distance between the z component and the xy plane component
		- New sub-option for the -C2M_DIST command: -UNSIGNED, to compute unsigned distances
		- New sub-option for the -SF_ARITHMETIC command: -IN_PLACE, to update the scalar field in place, without creating a new SF

	- New entity picking mechanism (to not rely on the deprecated OpenGL 'names' pushing mechanism)
		- Should hopefully solve most of the random issues with picking

	- New shortcut:
		- use 'ALT + SHIFT + mouse wheel' to change the zFar value (perspective mode)
		  (reminder: use 'ALT + mouse wheel' to change the zNear value)

	- Cloning:
		- ability to clone images
		- ability to clone 2D labels
		- ability to clone 2D viewport labels
		- ability to clone viewport objects

	- EDL shader:
		- contrast and rendering quality improved

	- ICP:
		- the computed registration transformation will now be applied to the aligned entity and all its children (if any)

	- STP:
		- the triangles of meshes tesselated from STEP files should now all be properly oriented
		- the linear deflection is asked only once when multiple files are loaded at the same time

	- Camera sensor edition dialog
		- the user can now control whether the input 'center' is in the world C.S. or in the camera C.S.
			(or more exactly, it's the translation part of a 4x4 transformation matrix - which was the case by default in the previous versions)

	- GBL sensor edition dialog
		- CC will now remember the last parameters used to create a sensor (not to edit one though)

	- CSF plugin
		- the CSF filter can now be applied on multiple clouds at once (both in the GUI and the command line version)

- Bug fix:
	- PCD: when transforming a cloud with a sensor (either manually, or via a registration tool, or via Edit > Apply Tranformation) and then exporting
		the cloud to a PCD file, both the points AND the sensor were transformed, resulting in the PCD file coordinates to be in the wrong coordinate
		system (as if the transformation had been applied twice)
	- The QTextStream::pos method is slow and unreliable. This could cause some issue when loading ASCII files.
	- When creating a new 3D view, it was not possible to pick an entity displayed in it by clicking on it.
	- When clicking on an entity in a 3D view while holding CTRL, it was only possible to select it, not to deselect it if it was already selected.
	- When using the Align tool on an aligned entity with labels, the initial view point could be invalid
	- Zooming in and out or changing the FOV in bubble view mode was not working anymore
	- When pausing the graphical segmentation tool while a contour / polyline was started but not closed, the tool icons would not be clickable anymore
		(forcing the user to exit the tool with ESC)
	- The 'Point List Picking' tool was not transferring the Global Shift information when exporting the list of points as a polyline
	- Rasterize tool: interpolated scalar fields were not properly exported in geotiff files (the interpolated part was missing)
	- The Registration tools (ICP and Point-pair based alignment) transformation filter mechanism would not let filter both translations and rotation at the same time
	- With a lot of NaN values in a scalar field, the Edit > Scalar field > Compute Stat Params tool could show wrong abscissa values below the histogram and curve.
		(note that the average, standard deviation, RMS and Chi2 distance values displayed either above the histogram or in the Console were correct)
	- When loading E57s file with large coordinates, and applying a Global Shift, the associated image sensors were not translated of the same Global Shift
	- When applying the viewport of a calibrated image (= image with an associated sensor), the camera f.o.v. was applied instead of a smarter value,
		taking the 3D view dimensions into account (resulting in a bad match between the 3D scene and the overlaid image)
	- The PCL plugin's 'MLS' tool dialog was broken
	- The CSF plugin command line could truncate the exported clouds filename if floating point values were present in this filename
	- The colors of the 'Compute Stat. Params' histogram could be wrong if the associated color scale was 'absolute'

v2.12.4 (Kyiv) - (14/07/2022)
----------------------

- Bug fixes:
	- CloudCompare 2.12.X (X<4) could miss some triangles when computing the Cloud-to-Mesh distances (resulting in slightly overestimated distances)
	- CloudCompare would still apply the Global Shift even though it was rejected by the user (which is a bad idea ;). But the entity
		Global Shift would then be (0, 0, 0), resulting in the loss of the original coordinate system.
	- the scale in the lower-right part of the 3D view could have the wrong length (if the height of the window is larger than its width)
	- FBX: CloudCompare couldn't properly load the materials (textures) of meshes with quads
	- the Compass plugin was exporting large coordinates with a too small precision
	- the command line could ignore the input global shift (set with -GLOBAL_SHIFT) if it was not sufficient
		to make the points coordinates small enough. This was not consistent with the previous behavior.
	- the segmentation tool in Perspective mode could keep points behind the camera/viewpoint

v2.12.3 (Kyiv) - (13/06/2022)
----------------------

- Bug fixes:
	- CloudCompare would still apply the Global Shift even though it was rejected by the user (which is a bad idea ;). But the entity
		Global Shift would then be (0, 0, 0), resulting in the loss of the original coordinate system.
	- the scale in the lower-right part of the 3D view could have the wrong length (if the height of the window is larger than its width)

v2.12.2 (Kyiv) - (23/05/2022)
----------------------
- Improvements:
	- CSF plugin:
		- 'Cloth resolution' and 'Classification threshold' input precision has been increased from 0.1 to 0.001

- Bug fixes:
	- The LAS PDAL filter could fail to save additional fields with space characters in their name on some architectures (Linux mostly).
		The filter will now automatically replace space characters (and equal characters) by specific sub-strings to be able to restore
		the original name when loading the LAS file later.
	- The LAS 1.3 or 1.4 filter was applying the Global Shift to the points even if the original 'LAS offset' was different. But it was declaring in the LAS file header that it has used the original LAS offset.
		This would lead to a shifted LAS file with the wrong global coordinates.
	- When double-clicking on a file with local characters in the Windows explorer, CC was not able to interpret the filename or file path correctly
	- Computing per-triangle normals (or methods requiring to update per-triangle normals such as Laplacian smoothing) could lead to a crash on some OBJ meshes

v2.12.1 (Kyiv) - (07/05/2022)
----------------------
- Improvements
	- ICP registration:
		- new option to prevent all rotations in the 'Research' tab ('None' option of the Rotation combo-box), to be consistent with the command line version

	- Rasterize tool:
		- the number of grid cells is now displayed (next to the grid size)
		- the number of non-empty cells is now displayed (next to the cloud name and size)

	- Most of the progress dialogs of CloudCompare should not steal the focus anymore (when loading multiple files in the background for instance)

	- Segmentation tool:
		- the icons are now visually disabled (grayed out) while drawing a polyline. Keyboard shortcuts are still working.

- Bug fixes:
	- The '-ICP -ROT NONE' command line option was not working as expected
	- When saving in a Shapefile the polyline exported from the Interactive Segmentation tool, an invalid global bounding-box was computed and saved by CC.
	- Label markers were preventing the user from picking the points below (in the Point picking tool as well as other tools)
	- LAS filters will now automatically convert NaN values to a default value when saving fields
		(as NaN is not valid according to specifications and not accepted by most third party libaries and tools)

v2.12.0 (Kyiv) - (30/03/2022)
----------------------
- New tools:
	- Menu 'Edit > Cloud'
		-  'Edit > Cloud > Create single point cloud': to create a cloud with a single point (set by the user)
		-  'Edit > Cloud > Paste from clipboard' (shortcut: CTRL+P): to create a cloud from ASCII/text data stored in the clipboard

	- Menu 'Edit > Scalar fields > Split clouds (integer values)'
		- Will split the cloud in multiple sub-clouds based on the (integer) values of its active scalar field. To be used with a classification SF typically.

	- Menu 'Edit > Scalar fields > Add classification SF'
		- Shortcut to 'Edit > Scalar fields > Add constant SF' to create a 'Classification' SF with a constant (integer) value

- New plugins:
	- qCloudLayers (by Wiggins Tech - https://www.wigginstech.com/)
		- manual classification of point clouds (with ASPRS classes by default, but can be customized / extended to any class values)
		- Developed by Neurodat (https://neurodat.com/contact-us/)
		- see https://www.cloudcompare.org/doc/wiki/index.php?title=QCloudLayers_(plugin)
	- PCL > Fast Global Registration (https://github.com/isl-org/FastGlobalRegistration)
		- automatic registration of point clouds (with normals) with no initial rough alignment
		- Based on: Q.-Y. Zhou, J. Park, and V. Koltun, "Fast Global Registration", ECCV, 2016
		- see https://www.cloudcompare.org/doc/wiki/index.php?title=Fast_Global_Registration
	- MPlane (by AIT - Austrian Institute of Technology - https://www.ait.ac.at/)
		- to perform normal distance measurements against a defined plane
		- see https://www.cloudcompare.org/doc/wiki/index.php?title=MPlane_(plugin)
	- Colorimetric Segmenter (by University of Nantes Graduate School of Engineering / Taglabs)
		- color-based segmentation of point clouds (see https://gitlab.univ-nantes.fr/E164955Z/ptrans)
		- see https://www.cloudcompare.org/doc/wiki/index.php?title=Colorimetric_Segmenter_(plugin)
	- Masonry Segmentation (Historic Digital Survey - https://cyberbuild.eng.ed.ac.uk/projects/historic-digital-survey)
		- segmentation of dense point clouds of masonry structures into their individual stones (see: https://github.com/CyberbuildLab/masonry-cc)
		- see https://www.cloudcompare.org/doc/wiki/index.php?title=Masonry_Segmentation_(plugin)
	- qMeshBoolean
		- Boolean operations on meshes, based on libigl (https://libigl.github.io/)
		- Slower, but supposedly more robust than Cork
		- see https://www.cloudcompare.org/doc/wiki/index.php?title=Mesh_Boolean_(plugin)
	- STEP I/O filter (by Raphael Marc, EDF R&D)
		- to load STEP files (as a single mesh for now)
		- based on Open Cascade (https://www.opencascade.com/)

- New command line options:
    - Added N_SIGMA_MIN and N_SIGMA_MAX options to the FILTER_SF command.
		- Specify the option followed by a numeric value to filter by N * standardDeviation around the mean.
	- NORMALS_TO_HSV
		- To convert normals to HSV colors
	- RGB_CONVERT_TO_SF
	 	- To convert RGB colors to 5 scalar fields (red, green, blue, alpha, composite)

- New file format:
	- STEP CAD files (see above)
	- Google DRACO format .DRC (compressed point clouds and meshes - https://github.com/google/draco)
	- For now, allows to save the geometry of clouds and meshes (as well as normals or RGB colors,
		but not the texture, etc.) with a configurable compression ratio

- Improvements
	- New Display options:
		- option to select the application style (Windows Vista, Windows, Fusion, etc.)
		- single click picking can be disabled (can be very slow for very large point clouds) 
	- Graphical Segmentation Tool (scissors):
		- the tool can now segment polylines (it will only keep segments with both vertices visible)
		- various improvements (visibility of segmented entities is forced for more clarity, etc.)
		- new option to flag the points inside the polygon with a given class value (instead of segmenting the points).
		  - a 'Classification' SF will be added to the cloud if none already (otherwise the 'Classification' SF will be updated)
		  - shortcut: 'C'
	- Rasterize tool
		- RGB and scalar field based layers can now be exported to standard image formats
		- improved interpolation scheme on the raster borders
		- new 'max edge length' option when interpolating grid cells, to avoid using large triangles
		- the rasterize tool grid is 'centered' on the raster (image) boundary pixels (the grid min corner coordinates
			correspond to the lower left raster cell center, and the grid max corner coordinates correspond to the upper
			right raster cell center). This has been made more explicit in the 'Edit grid' sub-dialog.
	- Section extraction tool
		- polylines (imported or drawn) can now be unselected when not in 'edition' mode. Just click anywhere else.
		- the 'unfold' option can now be used on all the polylines at once if no polyline is selected.
    - RANSAC plugin
        - Can save all leftover points into a new cloud (leftovers were points not assigned to a shape)
        - Can select whether to use Least Squares fitting on found shapes (some shapes take a very long time to process this step specifically Cones)
        - Can select whether to attemt to simplify shapes (torus->cone/cylinder/sphere/planes cone->cylinder/sphere/plane  cylinder->sphere/plane, sphere->plane)
        - Can choose whether or not to have a random color assigned to each shape found.
        - Ability to select min and max radii for various shapes (helps prevent giant spheres and cylinders from beating out the more likely plane feature)
	- ICP registration:
	    - new option to take the normals into account (if both entites have normals)
			(several matching modes are available: same side, opposite side, or double-sided)
 		- new 'research' option to use signed distances when registering a cloud with a (reference) mesh
		  (helfpul to prevent the cloud from sinking below the mesh surface if used in conjunction of a small overlap percentage)
	- Clipping box tool:
		- former 'contours' renamed 'envelopes' for the sake of clarity
		- ability to extract the real contours of the points inside each slice (single slice mode or 'repeat' mode)
			(CC will rasterize the slice and apply the 'contour plot' extraction algorithm)
	- Compass plugin:
		- planes fitted with the 'Plane tool' should now always have the normal pointing towards the user instead of a random orientation
	- Animation plugin:
		- option to smooth the trajectory
		- option to choose the video output codec/format
	- M3C2 plugin:
		- new options to orient normals: with the previous normal (if any) or with the associated sensor origin
	- Normal computation:
		- New preferred orientation: sensor origin
	- ATI cards:
		- the display should now be faster with ATI cards thanks to a smarter way to manage (2D text) textures
	- Localization:
		- Korean is now supported (thanks to Yun-Ho Chung)
		- Russian translation has been updated (thanks to Gene Kalabin)
		- Chinese is now supported (thanks to https://github.com/jindili)
	- The option 'Edit > Normals > Invert' can now be used on meshes
	- Roughness computation:
		- new option to set a 'up direction' to compute signed roughness values
	- Command line:
		- CSF plugin: added support for command line mode with all available options, except cloth export
			- use -CSF to run the plugin with the next optional settings:
				- -SCENES [scene]: name of the scene (SLOPE|RELIEF|FLAT)
				- -PROC_SLOPE: turn on slope post processing for disconnected terrain
				- -CLOTH_RESOLUTION [value]: double value of cloth resolution (ex 0.5)
				- -MAX_ITERATION [value]: integer value of max iterations (ex. 500)
				- -CLASS_THRESHOLD [value]: double value of classification threshold (ex. 0.5)
				- -EXPORT_GROUND: exports the ground as a .bin file
				- -EXPORT_OFFGROUND: exports the off-ground as a .bin file
		- Command 'Rasterize':
			- New output option '-OUTPUT_RASTER_Z_AND_SF' to explicitly export altitudes AND scalar fields.
				The former '-OUTPUT_RASTER_Z' option will only export the altitudes as its name implies.
			- New option '-MAX_EDGE_LENGTH' to set a maximum edge length when interpolating values
				(to be used with the '-EMPTY_FILL INTERP')
		- New sub-option for the RANSAC plugin command line option (-RANSAC)
			- OUT_RANDOM_COLOR = generate random colors for the output clouds (false by default now)
        - New sub-option for the 'FILTER_SF' command:
			- N_SIGMA_MIN and N_SIGMA_MAX: specify the option followed by a numeric value to filter by N * standardDeviation around the mean.
		- new option '-INVERT_NORMALS':
			- Inverts the normals of the loaded entities (cloud or mesh, and per-triangle or per-vertex for meshes)
		- new option '-RENAME_SF' {scalar field index} {name}:
			- To rename a scalar field
		- new option '-REMOVE_SF' {scalar field index}:
			- To remove a specific scalar field
		- new option '-NOISE KNN/RADIUS {value 1} REL/ABS {value 2} {RIP}':
			- To apply the Noise filter to the loaded point clouds
			- value 1: number of neighbors if KNN, radius if RADIUS
			- value 2: relative error (standard deviation multiplier) if REL, absolute error if ABS
			- RIP: remove isolated poins (optional)
		- the 'OCTREE_NORMALS' option has been updated:
			- MINUS_ZERO and PLUS_ZERO can now also be written MINUS_ORIGIN and PLUS_ORIGIN
			- new sub-options '-ORIENT PLUS_SENSOR_ORIGIN' and '-ORIENT MINUS_SENSOR_ORIGIN' (to use the sensor origin to orient the normals - a sensor must be associated to the cloud of course ;)
        - New sub-option for the 'ROUGH' command (Roughness):
			- '-UP_DIR X Y Z' to specify a 'up direction' to compute signed roughness values.
	- PCD:
		- CC can now load PCL files with integer xyz coordinates (16 and 32 bits) as well as double (64 bits) coordinates
		- CC can now load 'scan grids' corresponding to structured clouds (so as to compute robust normals for instance)
		- the (standard ?) 16bytes alignment for the various fields has been removed, so as to drastically reduce the memory consumption and the output file size!
	- STL:
		- loading speed should be greatly improved (compared to v2.10 and v2.11)
	- LAS:
	    - the Global Shift, if defined, will now be used as LAS offset if no offset was previously set
		- the PDAL LAS I/O filter and the libLAS I/O filter should now both handle LAS offset
		  and scale the same way at export time.
		- the libLAS I/O filter ('LAS 1.3 or 1.4') will now preserve the Coordinate Reference System (if the LAS file is loaded and saved with this filter)
	- OBJ:
		- support of the 'Ke' (emission) option in MTL files
	- Global Shift & Scale:
		- the qRansacSD plugin can now transfer the Global Shift & Scale info to the created primitives
		- the primitive fitting functions (Fit shpere, Fit plane, Fit facet and Fit quadric) as well
		- the 2.5D Volume calculation tool now transfers the Global Shift & Scale info to the exported difference map/cloud
	- Align tool (Point-pair based registration):
		- labels associated to a point cloud will now remain visible and the user can pick them
		- the tool will display the corresponding label title in the registration summary tables
	- Better management of filenames with non latin characters (for raster files, STL files, PDMS scripts, Point List picking exported files)
	- 'X', 'Y' and 'Z' labels are now displayed next to the trihedron axes (in the bottom right corner)

- Bug fixes
	- qBroom: the broom was not working properly on a non horizontal surfaces!
	- qM3C2: M3C2 dialog parameters were not properly restored in command line mode
	- Command line:
		- using the "-FBX" option would lead to an infinite loop
		- filenames local ('foreign') characters were not preserved
		- the '-GLOBA_SHIFT FIRST' option was not working properly
	- Trace polyline: the exported polylines had a wrong unique ID. Saving multiple polylines created this way in the
		same BIN file could lead to a corrupted file.
	- Scissors tool: points were segmented only inside the frustum (screen) even if some segmentation polygon vertices were outside
	- E57:  "invalid data" flags were wrongly interpreted
	- The 'Clean > Noise' tool was mixing the number of neighbors (knn) and the 'kernel radius' parameters
	- PLY filter was exporting large coordinates with a limited accuracy when creating ASCII files
	- the 'flip normals' checkbox of the C2M comparison dialog was not accessible anymore
	- minimal LAS scale suggested by CC was sometimes too small, potentially triggering a PDAL exception
	- when loading raster files, GDAL can sometimes report wrong min and max altitudes. This would then let CC think that invalid pixels were
		present. And when telling CC to keep these invalid points, their altitude would be replaced by a weird/random value...
	- CloudCompare was reporting truncated (integer) dip and dip direction values instead of rounded values
	- SHP files
		- CloudCompare was not able to read shapefiles with missing measurements (while this field is generaly optional for polylines, polygons and point clouds)
		- the SHP I/O filter was writing the local bounding-box in the file header instead of the global one (if the saved entities were shifted)
	- PLY files with point elements containing a 'list' property would be considered as face elements (preventing the user from loading the cloud)
		- It is now possible to load such PLY files if the 'list' is composed of floating point values
	- when merging two clouds, CC could crash is the LoD structure was currently being built at the same time on one of the clouds
	- the 'Guess parameters' option of the M3C2 plugin was suggesting radii while M3C2 scales are diameters (i.e. ideal values should have been twice as big)
	- ASCII file load dialog: CC would assign an 'Intensity' column to the 'Grey' color role, which is rarely a good idea. It will now assign it to 'Scalar field' by default.
	- trying to merge several clouds, some being ancestors of the others, could lead to a crash
	- calling the Mesh > Smooth (Laplacian) tool on a mesh with the database tree unfloded could make CC crash
	- cloning the 'contour' part of a Facet would result in a locked polyline (that cannot be deleted or moved)
	- when rendering the screen with a zoom > 1, the scale label was wrongly scaled as well
	- Graphical segmentation: when using the shortcuts 'i' and 'o' to segment points inside or outside a polyline not yet closed (with a right click),
		the overlay buttons would become transparent to clicks, and the not yet confirmed vertex of the polyline was not taken into account
	- Rasterize tool: the exported geotiff files were (once again?!) shifted of half the cell size

v2.11.3 (Anoia) - 08/09/2020
----------------------
- Bug fix:
  - when used with only 3 points (pairs), the Align tool could produce a wrong registration matrix

v2.11.2 (Anoia) - 30/08/2020
----------------------
- Bug fixes:
 - command line: the LAST keyword was consistently badly managed (for -SF_OP and -ICP)
 - the new 'Edit > Colors > From Scalar Fields' tool could crash due to a non initialized variable
 - PV files could not be loaded anymore
 - even in the command line 'SILENT' mode, loading a SHP file with additional fields in the associated DBF file would spawn a blocking dialog
	(these fields will now be ignored in SILENT mode)
 - LAS 1.3 or 1.4 (LASlib): Classification values and flags were not always loaded or saved properly
 - [Windows] The contour lines generated with the Rasterize tool could be incomplete (due to GDAL 3.0 --> reverting to GDAL 2.2)

v2.11.1 (Anoia) - 29/07/2020
----------------------
- Bug fixes:
  - [macOS] Fix determination of command line use so opening a file in the GUI using the command line works (#1296).
  - E57, Bundler: when trying to apply a Global Shift translation on load without 'preserving' the shift information, the Global Shift was simply not applied!
  - Global Shift: when unchecking the 'preserve shift on save' option of the Global Shift dialog, and then hitting the "Apply all" button,
                  the selection was not maintained and propagated to the other loaded clouds
  - command line: the 'LAST' keyword was not properly managed after -SF_ARITHMETIC

v2.11.0 (Anoia) - 14/06/2020
----------------------

- New features
	- 3 methods to quickly change the coordinate system of one or several entities
		- Tools > Registration > Move bounding-box center to origin
		- Tools > Registration > Move bounding-box min corner to origin
		- Tools > Registration > Move bounding-box max corner to origin
	- Add smooth zoom when middle mouse button is pressed
	- Roll view around view-vector when pressing SHIFT and left mouse button
	- Edit > Plane > Flip: to flip the selected planes
	- Edit > Plane > Compare: to compare the two selected planes (angle and relative distances)
	- Edit > Mesh > Flip triangles: to flip the triangles (vertices) in case they are defined in the wrong order
	- Tools > Distances > Cloud/Primitive Dist
		- Used to calculate distance to primitive shape (supports spheres, planes, cylinders, cones and boxes) rather than the mesh of that shape (more accurate results for spheres)
		planes are now optionally treated as bounded or unbounded.
		for cones this will not work with Snout mode cones.
	- New tool:
		- Edit > Normals > Export normals to SF(s) (or equivalently Edit > Scalar fields > Export normals to SF(s))
		- Command line argument: -NORMALS_TO_SFS (all dimensions are exported by default as 3 scalar fields)
	- Command line:
		- '-H_EXPORT_FMT' added to select format for hierarchy objects exported
		- The PCV tool can now be accessed via the command line:
			- Option '-PCV' (see https://www.cloudcompare.org/doc/wiki/index.php?title=Command_line_mode for sub-options)
			- Can be called on any number of clouds or meshes
			- (the tool was already accessible in V2.10, but in a very limited way)
		- RANSAC plugin support added, all parameters below are optional and can be added in any order, and will work on all clouds opened and already loaded when called
			- '-RANSAC' (main command)
				- EPSILON_ABSOLUTE - max distance to primitive
				- EPSILON_PERCENTAGE_OF_SCALE - max distance to primitive as a percentage of cloud scale 0.0-1.0 exclusive
				- BITMAP_EPSILON_PERCENTAGE_OF_SCALE - sampling resolution as a percentage of cloud scale 0.0-1.0 exclusive
				- BITMAP_EPSILON_ABSOLUTE - sampling resolution
				- SUPPORT_POINTS - min Support points per primitive
				- MAX_NORMAL_DEV - max normal deviation from the ideal shape normal vector [in Degrees]
				- PROBABILITY - probability that no better candidate was overlooked during sampling, the lower the better!
				- OUT_CLOUD_DIR - path to save detected shapes clouds to, current dir if unspecified
				- OUT_MESH_DIR - path to save detected shapes meshes to, current dir if unspecified
				- OUT_PAIR_DIR - path to save detected shapes clouds & meshes to, current dir if unspecified
				- OUT_GROUP_DIR - path to save all shapes and primitives to as a single file, current dir if unspecified
				- OUTPUT_INDIVIDUAL_SUBCLOUDS - output detected shapes clouds
				- OUTPUT_INDIVIDUAL_PRIMITIVES - output detected shapes meshes
				- OUTPUT_INDIVIDUAL_PAIRED_CLOUD_PRIMITIVE - output detected shapes clouds & meshes
				- OUTPUT_GROUPED - output all detected shapes clouds & meshes as single file
				- ENABLE_PRIMITIVE - each shape listed after this option will be searched for
					- Shapes are: PLANE, SPHERE, CYLINDER, CONE, TORUS
	    - '-NORMALS_TO_DIP': converts the loaded cloud normals to dip and dip direction (scalar fields)
	    - '-NORMALS_TO_SFS': converts the loaded cloud normals to 3 scalar fields (Nx, Ny and Nz)
		- '-REMOVE_RGB': to remove RGB colors (from all loaded entities, i.e. clouds or meshes)
		- '-REMOVE_Normals': to remove normals (from all loaded entities, i.e. clouds or meshes)
	    - The 1st Order Moment tool (Tools > Other > Compute geometric features) can now be accessed via
	    	the command line mode with option -MOMENT {kernel size}
	    	- Computes 1st order moment on all opened clouds and auto saved by default.
	    - The Feature tools (Tools > Other > Compute geometric features) can now be accessed via
    		the command line mode with option -FEATURE {type} {kernel size}
	    	- type is one of the following: SUM_OF_EIGENVALUES, OMNIVARIANCE, EIGENTROPY, ANISOTROPY, PLANARITY, LINEARITY, PCA1, PCA2, SURFACE_VARIATION, SPHERICITY, VERTICALITY, EIGENVALUE1, EIGENVALUE2, EIGENVALUE3.
	- 4 new default color scales:
		- Brown > Yellow
		- Yellow > Brown
		- Topo Landserf
		- High contrast
	- Translate/Rotate Tool:
		- Advanced translate mode to translate along a single segment polyline or plane normal vector.
		- Advanced rotate mode to update the axis of rotation to a single segment polyline or plane normal vector.
		 - Rotation center can be set to either the object center, or the polyline/plane normal.

- Improvements
	- CloudCompare now handles RGBA colors for points, mesh vertices, text, and labels
		- partial ability to display these elements with some transparency (warning: points and triangles are not depth-sorted yet - display of clouds and meshes might not be very nice ;))
		- default materials and colors (Display > Display settings) now have an editable 'alpha' channel
		- the 'Edit > Colors > Set unique' and 'Edit > Colors > Colorize' tools also have an editable 'alpha' channel
		- this fixes a bug with ATI cards (when using VBOs - see below)
	- Better support for High DPI screens (4K) on Windows
	- Both the local and global bounding-box centers are now displyaed in the cloud properties (if the cloud has been shifted)
	- The PoissonRecon plugin now relies on the PoissonRecon V12 library
		- new algorithm
		- option to set the final 'resolution' instead of the octree depth
	- Align (Point-pair based registration) tool
		- can now be used with several entities (both several aligned and several reference entities)
		- option to pick the center of sphere entities as registration point(CC will ask whether to use the sphere center or not when picking a point anywhere on a sphere entity)
	- Clipping box tool:
		- option to select the extracted contour type (LOWER, UPPER or FULL)
		- 'up' direction is always Z for slices normal to X or Y (the local X, Y or Z directions of the active cross-section)
		- 'up' direction is X for slices normal to Z
	- All parameters should now be properly remembered from one call to the other (during the same session)
	- The current box/slice position can now be exported (resp. imported) to (resp. from) the clipboard via the 'Advanced' menu
	- Command line tool:
		- The C2M_DIST command (Cloud-to-Mesh distances) can now be called with 2 meshes as input. In this case the first mesh vertices are used as compared cloud.
		- New suboption for the -O -GLOBAL_SHIFT option: 'FIRST'
			To use the first encountered (non null) global shift for all loaded entities (must be defined for all entities nevertheless ;)
		- The CROP command will now remove the input cloud if it's totally 'cropped out' (instead of leaving the full original cloud loaded)
		- The 'FWF_O' command (to load LAS files with associated waveform data) nows properly supports the '-GLOBAL_SHIFT' option
		- No more popup will appear when loading a raster file via the command line mode in SILENT mode (raster is never loaded as a textured quad, and invalid points are always ignored and not loaded)
		- One can now use the 'auto' value for the radius value input after -OCTREE_NORMALS to automatically guess the normal computation radius
	- Raster import:
		- new "Apply all" option when CC asks whether invalid pixels of a raster should be ignored or not
	- Point picking:
		- points can now be picked on meshes (triangles)
		- the Point List Picking tool can now be used on meshes
		- when using 'Shift+left click' or the Point picking tool on a mesh, CC won't spawn a 3-point label anymore, but a single point label at the right position
		- labels picked on a mesh can be used to re-orient the camera (right-click on the label entry in the DB)
		- for now, labels associated to meshes are not preserved when cloning or merging the meshes
		- points are now exclusively picked inside the current fustrum (no more points behind the camera)
	- The center of sphere (entities) can now be picked (a popup will let the user select the sphere center or a point on the surface)
	- Graphical segmentation:
		- points are now exclusively segmented inside/outside the frustrum
	- Plugins:
		- plugins may now be enabled/disabled in the plugin info window
		- to take effect, CloudCompare must be restarted
		- all plugins are still available on the command line
	- PCD now supports loading more field types (16 bit signed and unsigned, 32 bit unsigned, 64 bit floating point)
	- OBJ files:
		- we now correctly handle faces with more than 4 vertices! (they should be properly tesselated)
		- support of escaped lines ('\' at the end of the line)
		- now accepts MTL files with the 'Tf' keyword (well, CC just ignores it and doesn't complain about a wrong MTL file anymore ;)
		- enhanced progress report (thanks to https://gitlab.com/Epic_Wink)
	- Translation:
		- new Argentinian Spanish translation
	- M3C2:
		- the computation speed should be improved when using a small projection radius (smarter selection of the octree level)
	- LAS files:
		- the standard LAS Filter now handles the OVERLAP classification bit (for point format >= 6)
		- improved/fixed management of classification and classification flags
		- LAS offset (chosen at saving time) should be a little bit smarter:
			- CC will try to keep the previous one, or use the bounding-box min corner ONLY if the coordinates are too large
			- CC won't use the previous scale if it is too small for the current cloud
			- the 'optimal' scale is simpler (round values + the same for all dimensions)
		- The LAS 1.3/1.4 filter (Windows only) has been improved:
			- option to save any scalar field as extra bytes (and load extra bytes as scalar fields)
			- proper minor version setting
			- proper header size
			- using the latest version of LASlib with proper management of UTF8/UTF16 ('foreign') characters
	- ASCII files:
		- now allows mixed whitespace (spaces / tabs)
		- the ASCII load dialog option has now an option to load numerical values with a comma as digit separator ('use comma as decimal character' checkbox)
	- E57 files:
		- sensors are now automatically created below each scan (if a sensor position is defined in the file)
			- these can be used to orient the normals for instance
		- after loading an E57 file, the scan (sensor) origin and orientation is stored as meta-data and should be properly restored and saved when exporting the scan(s) back to E57
	- Unroll
		- ability to set the start and stop angles for the cone unrolling options
		- ability to unroll meshes
		- new unrolling mode: 'Straightened cone' (the previous one has been renamed 'Straightened cone (fixed radius)'). This new mode unrolls the cone as a cylinder but with a varying radius.
	- The 'Straightened cone' options are now using the real curvilinear abscissa (0 = cone apex)
	- Tools > Others > Compute geometric features
		- option to compute the 1st moment added
		- option to compute the 1st, 2nd and 3rd eigenvalues added
	- Stereo mode updated:
		- new stereo mode (Generic stereo display) to handle more stereo displays (PluraView, etc.)
		- new stereo parameters (screen/display size, distance to screen, and eye separation)
	- SBF files
		- format slightly updated to accomodate with scalar fields 'shift' (backward compatibiltiy maintained)
		- format descritpion is here: https://www.cloudcompare.org/doc/wiki/index.php?title=SBF
	- Others:
		- CC now saves the radius (parameter) after computing normals (as meta-data associated to the cloud)
		- stereogram tool of the Facets plugin now uses the new 'High contrast' color scale by default
		- Contour (extraction) renamed Envelope (extraction) for the sake of clarity

- Changes
	- Command line tool:
		- The `-FBX_EXPORT_FMT` command is now split. Use `-FBX -EXPORT_FMT`.
	- Plugins:
		- The I/O plugin interface has changed, so if you have your own I/O plugins, you will need to update them.
			- The interface name changed from `ccIOFilterPluginInterface` to `ccIOPluginInterface`.
			- The `ccIOPluginInterface::getFilter()` method was removed in favour of `ccIOPluginInterface::getFilters()`.
			- The `FileIOFilter` base class now takes a struct as an argument containing all the static info about a filter - extensions, features (import/export), etc.. See `FooFilter` in the `ExampleIOPlugin` and the comments in `FileIOFilter::FilterInfo`.
			- The use of `FileIOFilter::FilterInfo` means that the following virtual functions in I/O filters are no longer virtual/required:
				- importSupported
				- exportSupported
				- getFileFilters
				- getDefaultExtension
				- canLoadExtension
		- The GL plugin interface has changed, so if you have your own GL plugins, you will need to update them.
			- The interface name changed from `ccGLFilterPluginInterface` to `ccGLPluginInterface`.
	- CC will now handle external matrices (loaded or input via the 'Edit > Apply Transformation' tool) with a 16th component different than 0
		(this 16th component will be considered as the inverse scale)
	- Attempt to improve gamepads detection/connection

- Bug fixes:
	- LAS classification flags were not always properly extracted/saved by the standard LAS filter (depending on the point format)
	- Trace Polyline tool: when changing the OpenGL camera position while tracing a polyline AND using oversampling, strange spikes could appear
	- The Unroll dialog was not enabling all the apex coordinate fields after switching from Cylinder to Cone mode
	- The Clipping-box tool 'edit' dialog would sometimes move the box in an unepected way when opening and closing it without making any change
	- M3C2: the 'subsampling' option was not properly restored when loading the parameters from a file (if 'SubsampleEnabled = false')
	- Orienting normals with a sensor position could lead to a crash
	- Shapefile: at export time, the SHX file (created next to the SHP file) was malformed
		- this prevented loading the file in most GIS tools
		- polylines were mistakenly exported as polygons
	- SRS (Spatial Reference System) information could be lost when loading LAS files
	- The cartesian bounding-box of exported E57 files was wrongly expressed in the file-level coordinate system (instead of the local one)
	- Data could be lost when merging two clouds with FWF data
	- When VBOs were activated with an ATI card, CloudCompare could crash (because ATI only supports 32bit aligned VBOs :p)
	- The LAS 1.3/1.4 filter was not compressing files with a minor case 'laz' extension :(
	- The iteration stop criteria has been changed in the CSF plugin to fix a small bug
	- Point picking on meshes: picking points on triangles that were partially outside of the frustum would lead to the wrong point being picked (or no point picked)

v2.10.3 (Zephyrus) - 13/06/2019
----------------------

- Enhancements
  - Speed up the Fit Sphere tool and point picking in the Registration Align (point pairs picking) tool

- Bug fixes
  - Command line:
    - the 'EXTRACT_VERTICES' option was actually deleting the extracted vertices right after extracting them, causing a crash when trying to access them later :| (#847)
    - fix handling of SF indices in SF_ARITHMETIC and COMMAND_SF_OP
    - the COMMAND_ICP_ROT option of the ICP command line tool was ignored (#884)
    - when loading a BIN file from the command line, only the first-level clouds were considered
  - Fix loading LAS files with paths containing multi-byte characters when using PDAL (#869)
  - When saving a cloud read from LAS 1.0 let PDAL choose the default LAS version (#874)
  - Fix potential crash or use of incorrect data when comparing clouds (#871)
  - Fix potential crash when quitting or switching displays
  - Quitting the "Section extraction tool" (and probably any tool that uses a temporary 3D view, such as the Align tool) would break the picking hub mechanism (preventing the user from picking points typically) (#886)
  - Fix the camera name being displayed in the wrong place (#902)
  - The layers management of the Rasterize tool was partially broken
  - the C2C/C2M distance computation tool called through the command line was always displaying progress dialogs even in SILENT mode
  - the ICP registration tool called through the command line was always displaying progress dialogs even in SILENT mode
  - Fix potential crash with qCSF (see github issue #909)
  - In some cases, the (subsampled) core points cloud was not exported and saved at the end of the call to M3C2 through the command line
  - Some points were incorrectly removed by the 'Clean > Noise filer' method (parallelism issue)
  - The radius was not updated during the refinement pass of the Sphere fitting algorithm  (i.e. the final radius was not optimal)

v2.10.2 (Zephyrus) - 24/02/2019
----------------------

- Bug fixes
  - Rasterize tool:
    - interpolating empty cells with the 'resample input cloud' option enabled would make CC crash
    - change layout so it works better on lower-resolution monitors
  - Command line:
    - the 'EXTRACT_VERTICES' option was not accessible
    - calling the -RASTERIZE option would cause an infinite loop
    - the Global Shift & Scale information of the input cloud was not transferred to the output cloud of the -RASTERIZE tool
  - glitch fix: the 3D window was not properly updated after rendering the screen as a file with a zoom > 1
  - glitch fix: the name of the entity was not displayed at the right place when rendering the screen as a file with a zoom > 1
  - the Surface and Volume Density features were potentially outputting incorrect values (the wrong source scalar field was used when applying the dimensional scale!)
  - the chosen octree level could be sub-optimal in some very particular cases
  - E57 pinhole images:
    - fix sensor array information (it was displaying total image size for the width of the image)
    - fix pixel width & height


- Translations
  - updated Russian translation (thanks to Eugene Kalabin)
  - added Japanese translation (thanks to the translators at CCCP)


- macOS Note
  - I (Andy) had to update ffmpeg, which is used by the animation plugin, for this patch release. Normally I would wait for 2.11, but homebrew changed their policies and started including everything in their build, so I can no longer use it. The good news is that compiling ffmpeg myself and statically linking shaves about 30 MB off the size of CloudCompare.app...
  - it has been reported that this fixes a potential crash in ffmpeg's libavutil.56.dylib

v2.10.1 (Zephyrus) - 01/16/2019
----------------------

- Bug fixes:

  - writing E57 files was broken
  - an exception was being thrown when you close CC after saving an ASCII file (#834)

v2.10 (Zephyrus) - 01/06/2019
----------------------

- new features:

	* Edit > Polyline > Sample points
		- to regularly samples points on one or several polylines

	* New set of geometrical features to compute on clouds:
		- Tools > Other > Compute geometric features
		- features are all based on locally computed eigen values:
			* sum of eigen values
			* omnivariance
			* eigenentropy
			* anisotropy
			* planarity
			* linearity
			* PCA1
			* PCA2
			* surface variation
			* sphericity
			* verticality
		- most of the features are defined in "Contour detection in unstructured 3D point clouds", Hackel et al, 2016

	* Localization support
		- Display > Language translation
		- currently supported languages:
			* English (default)
			* Brazilian Portuguese (partial)
			* French (very partial)
			* Russian (partial)
		- volunteers are welcome: https://www.cloudcompare.org/forum/viewtopic.php?t=1444

- enhancements:

	* Roughness, Density and Curvature can now all be computed via the new 'Tools > Other > Compute geometric features' menu
		(Approx density can't be computed anymore)

	* Global Shift & Scale dialog
		- new option "Preserve global shift on save" to directly control whether the global coordinates should be preserved
			at export time or simply forgotten

	* The 'Display > Lock vertical rotation' option has been renamed 'Display > Lock rotation about an axis' (Shortcut: L)
		- CC will now ask for the rotation axis to be locked (default: Z)

	* The M3C2 plugin can now be called from the command line:
		- the first time you'll need the configuration file saved with the GUI tool
			(Use the 'Save parameters to file' button in the bottom-left corner of the M3C2 dialog --> the floppy icon)
		- then load 2 files (cloud 1 and cloud2)
		- optionally load a 3rd cloud that will be used as core points
		- and eventually call the -M3C2 option with the parameter file as argument:
			CloudCompare -O cloud1 -O cloud2 (-O core_points) -M3C2 parameters_file
		- new option to use the core points cloud normals (if any)

	* The Canupo plugin is now open-source!
		- Thanks (once again) to Dimitri Lague for this great contribution
		- the code is here: https://github.com/CloudCompare/CloudCompare/tree/master/plugins/core/qCanupo

	* The "Classify" option of the Canupo plugin can now be called from the command line:
		- you'll need a trained classifier (.prm file)
		- main option: -CANUPO_CLASSIFY classifier.prm
		- confidence threshold:
			* -USE_CONFIDENCE {threshold}  (threshold must be between 0 and 1)
			* (use the 'SET_ACTIVE_SF' after loading a cloud to set the active scalar field if
				you want it to be used to refine the classification)
		- syntax:
			CloudCompare -O cloud1 ... -O cloudN -CANUPO_CLASSIFY (-USE_CONFIDENCE 0.9) classifier.prm

	* Labels can now be imported from ASCII files:
		- new column role in the ASCII loading dialog: "Labels"
		- labels can be created from textual or numerical columns
		- one "2D label" entity is created per point (don't try to load too many of them ;)
		- labels are displayed in 3D by default (i.e. next to each point), but they can also be displayed in 2D (see the dedicated check-box)

	* FBX units:
		- default FBX units are 'cm'
		- if a FBX file with other units is imported, CC will now store this information as meta-data and will set it correctly
			if the corresponding meshes are exported as FBX again

	* Command line mode:
		- scalar field convert to RGB:
			* '-SF_CONVERT_TO_RGB {mixWithExistingColors bool}'
		- scalar field set color scale:
			* '-SF_COLOR_SCALE {filename}'
		- extract all loaded mesh vertices as standalone 'clouds' (the mesh is discarded)
			* '-EXTRACT_VERTICES'
		- remove all scan grids
			* '-REMOVE_SCAN_GRIDS'
		- new sub-option of 'SAVE_CLOUDS' to set the output filename(s) (e.g. -SAVE_CLOUDS FILE "cloud1.bin cloud2.bin ..."
		- new options for the 'OCTREE_NORMALS' (thanks to Michael Barnes):
			* '-ORIENT' to specify a default orientation hint:
				- PLUS_ZERO
				- MINUS_ZERO
				- PLUS_BARYCENTER
				- MINUS_BARYCENTER
				- PLUS_X
				- MINUS_X
				- PLUS_Y
				- MINUS_Y
				- PLUS_Z
				- MINUS_Z
				- PREVIOUS
			* '-MODEL' to specify the local model:
				- LS
				- TRI
				- QUADRIC

	* Unroll tool:
		- the cylindrical unrolling can be performed inside an arbitrary angular range (between -3600 and +3600 degrees)
		- this means that the shape can be unrolled on more than 360 degrees, and from an arbitrary starting orientation

	* New options (Display > Display options):
		- the user can now control whether normals should be enabled on loaded clouds by default or not (default state is now 'off')
		- the user can now control whether load and save dialogs should be native ones or generic Qt dialogs

	* Normals:
		- ergonomics of 'Normals > compute' dialog have been (hopefully) enhanced
		- normals can now be oriented toward a sensor even if there's no grid associated to the point cloud.
		- the Normal Orientation algorithm based on the Minimum Spanning Tree now uses much less memory (~1/10)

	* PCV:
		- the PCV plugin can now be applied on several clouds (batch mode)

	* LAS I/O:
		- CloudCompare can now read and save extra dimensions (for any file version) - see https://github.com/CloudCompare/CloudCompare/pull/666

	* E57:
		- the E57 plugin now uses [libE57Format] (https://github.com/asmaloney/libE57Format) which is a fork of the old E57RefImpl
		- if you compile CloudCompare with the E57 plugin, you will need to use this new lib and change some CMake options to point at it - specifically **OPTION_USE_LIBE57FORMAT** and **LIBE57FORMAT_INSTALL_DIR**
		- the E57 plugin is now available on macOS

	* RDS (Riegl)
		- the reflectance scalar field read from RDS file should now have correct values (in dB)

	* SHP:
		- improved support thanks to T. Montaigu (saving and loading Multipatch entities, code refactoring, unit tests, etc.)

	* Cross section tool:
		- can now be started with a group of entities (no need to select the entities inside anymore)
		- produces less warnings

	* Plugins (General):
		- the "About Plugins" dialog was rewritten to provide more information about installed plugins and to include I/O and GL plugins.
		- [macOS] the "About Plugins..." menu item was moved from the Help menu to the Application menu.
		- added several fields to the plugin interface: authors, maintainers, and reference links.
		- I/O plugins now have the option to return a list of filters using a new method *getFilters()* (so one plugin can handle multiple file extensions)
		- moved support for several less frequently used file formats to a new plugin called qAdditionalIO
			- Snavely's Bundler output (*.out)
			- Clouds + calibrated images [meta][ascii] (*.icm)
			- Point + Normal cloud (*.pn)
			- Clouds + sensor info. [meta][ascii] (*.pov)
			- Point + Value cloud (*.pv)
			- Salome Hydro polylines (*.poly)
			- SinusX curve (*.sx)
			- Mensi Soisic cloud (*.soi)

	* Misc:
		- some loading dialogs 'Apply all' button will only apply to the set of selected files (ASCII, PLY and LAS)
		- the trace polyline tool will now use the Global Shift & Scale information of the first clicked entity
		- when calling the 'Edit > Edit Shift & Scale' dialog, the precision of the fields of the shift vector is now 6 digits
			(so as to let the user manually "geo-reference" a cloud)
		- the ASCII loading dialog can now load up to 512 columns (i.e. almost as many scalar fields ;). And it shouldn't become huge if
			there are too many columns or characters in the header line!

- bug fixes:

	* subsampling with a radius dependent on the active scalar field could make CC stall when dealing with negative values
	* point picking was performed on each click, even when double-clicking. This could actually prevent the double-click from
		being recognized as such (as the picking could be too slow!)
	* command line mode: when loading at least two LAS files with the 'GLOBAL_SHIFT AUTO' option, if the LAS files had different AND small LAS Shift
	* point picking on a mesh (i.e. mainly in the point-pair based registration tool) could select the wrong point on the triangle, or even a wrong triangle
	* raster I/O: when importing a raster file, the corresponding point cloud was shifted of half a pixel
	* the RASTERIZE command line could make CC crash at the end of the process
	* hitting the 'Apply all' button of the ASCII open dialog would not restore the previous load configuration correctly in all cases
		(the header line may not be extracted the second time, etc.)
	* align tool: large coordinates of manually input points were rounded off (only when displayed)
	* when applying an orthographic viewport while the 'stereo' mode is enabled, the stereo mode was broken (now a warning message is displayed and
		the stereo mode is automatically disabled)
	* the global shift along vertical dimension (e.g. Z) was not applied when exporting a raster grid to a raster file (geotiff)
	* the 2.5D Volume calculation tool was ignoring the strategy for filling the empty cells of the 'ceil' cloud (it was always using the 'ground' setting)
	* [macOS] fixed the squished text in the Matrix and Axis/Angle sections of the transformation history section of the properties
	* [macOS] fixed squished menus in the properties editor
	* the application options (i.e. only whether the normals should be displayed or not at loading time) were not saved!
	* DXF files generated by the qSRA plugin were broken (same bug as the DXF filter in version 2.9)
	* the OCTREE_NORMALS command was saving a file whatever the state of the AUTO_SAVE option
	* the Align tools could make CC crash when applying the alignment matrix (if the octree below the aligned entity was visible in the DB tree)
	* the clouds and contour lines generated by the Rasterize tool were shifted of half a cell
	* in some cases, merging a mesh with materials with a mesh without could make CC crash
	* command line mode: the VOLUME command parser would loop indefinitely if other commands were appended after its own options + it was ignoring the AUTO_SAVE state.
	* some files saved with version 2.6 to 2.9 and containing quadric primitives or projective camera sensors could not be loaded properly since the version 2.10.alpha of May 2018
	* for a mysterious reason, the FWF_SAVE_CLOUDS command was not accessible anymore...
	* when computing C2C distances, and using both a 2.5D Triangulation local model and the 'split distances along X, Y and Z' option, the split distances could be wrong in some times

v2.9.1 (Omnia) - 11/03/2017
----------------------

- enhancements:

	* Primitive factory
		- sphere center can now be set before its creation (either manually, or via the clipboard if the string is 'x y z')

- Bug fixes:

	* DXF export was broken (styles table was not properly declared)
	* PLY files with texture indexes were not correctly read

v2.9 (Omnia) - 10/22/2017
----------------------

- New features:

	* New plugin: qCompass
		- structural geology toolbox for the interpretation and analysis of virtual outcrop models (by Sam Thiele)
		- see http://www.cloudcompare.org/doc/wiki/index.php?title=Compass_(plugin)

	* 3D view pivot management:
		- new option to position the pivot point automatically on the point currently at the screen center (dynamic update)
			(now the default behavior, can be toggled thanks to the dedicated icon in the 'Viewing tools' toolbar or the 'Shift + P' shortcut)
		- double clicking on the 3D view will also reposition the pivot point on the point under the cursor
		- the state of this option is automatically saved and restored when CC starts

	* New tool to import scalar fields from one cloud to another: 'Edit > SFs > Interpolate from another entity'
		- 3 neighbor extraction methods are supported (nearest neighbor, inside a sphere or with a given number of neighbors)
		- 3 algorithms are available: average, median and weighted average

	* New sub-menu 'Tools > Batch export'
		- 'Export cloud info' (formerly in the 'Sand-box' sub-menu)
			* exports various pieces of information about selected clouds in a CSV file
			* Name, point count, barycenter
			+ for each scalar field: name, mean value, std. dev. and sum
		- 'Export plane info'
			* exports various pieces of information about selected planes in a CSV file
			* Name, width, height, center, normal, dip and dip direction

	* New interactor to change the default line width (via the 'hot zone' in the upper-left corner of 3D views)

	* New option: 'Display > Show cursor coordinates'
		- if activated, the position of the mouse cursor relatively to the 3D view is constantly displayed
		- the 2D position (in pixels) is always displayed
		- the 3D position of the point below the cursor is displayed if possible

	* New shortcut: P (pick rotation center)

- enhancements:

	* When a picking operation is active, the ESC key will cancel it.

	* qBroom plugin:
		- now has a wiki documentation: http://www.cloudcompare.org/doc/wiki/index.php?title=Virtual_broom_(plugin)

	* qAnimation plugin:
		- new output option 'zoom' (alternative to the existing 'super resolution' option)
		- the plugin doesn't spam the Console at each frame if the 'super resolution' option is > 1 ;)

	* M3C2 plugin:
		- "Precision Maps" support added (as described in "3D uncertainty-based topographic change detection with SfM
			photogrammetry: precision maps for ground control and directly georeferenced surveys" by James et al.)
		- Allows for the computation of the uncertainty based on precision scalar fields (standard deviation along X, Y and Z)
			instead of the cloud local roughness

	* 'Unroll' tool:
		- new cone 'unroll' mode (the true 'unroll' mode - the other one has been renamed 'Straightened cone' ;)
		- option to export the deviation scalar-field (deviation to the theoretical cylinder / cone)
		- dialog parameters are now saved in persistent settings

	* Plugins can now be called in command line mode
		(the 'ccPluginInterface::registerCommands' method must be reimplemented)
		(someone still needs to do the job for each plugin ;)

	* Trace polyline tool
		- the tool now works on meshes
		- Holding CTRL while pressing the right mouse button will pan the view instead of closing the polyline
		- new 'Continue' button, in case the user has mistakenly closed the polyline and wants to continue

	* Command line mode
		- the Rasterize tool is now accessible via the command line:
			* '-RASTERIZE -GRID_STEP {value}'
			* additional options are:
				-VERT_DIR {0=X/1=Y/2=Z} - default is Z
				-EMPTY_FILL {MIN_H/MAX_H/CUSTOM_H/INTERP} - default is 'leave cells empty'
				-CUSTOM_HEIGHT {value} - to define the custom height filling value if the 'CUSTOM_H' strategy is used (see above)
				-PROJ {MIN/AVG/MAX} - default is AVG (average)
				-SF_PROJ {MIN/AVG/MAX} - default is AVG (average)
				-OUTPUT_CLOUD - to output the result as a cloud (default if no other output format is defined)
				-OUTPUT_MESH - to output the result as a mesh
				-OUTPUT_RASTER_Z - to output the result as a geotiff raster (altitudes + all SFs by default, no RGB)
				-OUTPUT_RASTER_RGB - to output the result as a geotiff raster (RGB)
				-RESAMPLE - to resample the input cloud instead of generating a regular cloud (or mesh)
			* if OUTPUT_CLOUD and/or OUTPUT_MESH options are selected, the resulting entities are kept in memory.
				Moreover if OUTPUT_CLOUD is selected, the resulting raster will replace the original cloud.
		- 2.5D Volume Calculation tool
			* '-VOLUME -GRID_STEP {...} etc.' (see the wiki for more details)
		- Export coord. to SF
			* '-COORD_TO_SF {X, Y or Z}'
		- Compute unstructured cloud normals:
			* '-OCTREE_NORMALS {radius}'
			* for now the local model is 'Height Function' and no default orientation is specified
		- Clear normals
			* '-CLEAR_NORMALS'
		- New mesh merging option
			* '-MERGE_MESHES'
		- Compute mesh volume:
			* '-MESH_VOLUME'
			* optional argument: '-TO_FILE {filename}' to output the volume(s) in a file
		- LAS files:
			* when loading LAS files without any specification about Global Shift, no shift will be applied, not even the LAS file internal 'shift' (to avoid confusion)
			* however, it is highly recommended to always specify a Global Shift (AUTO or a specific vector) to avoid losing precision when dealing with big coordinates!
		- Other improvements:
			* the progress bar shouldn't appear anymore when loading / saving a file with 'SILENT' mode enabled
			* the ASCII loading dialog shouldn't appear anymore in 'SILENT' mode (only if CC really can't guess anything)
			* the default timestamp resolution has been increased (with milliseconds) in order to avoid overwriting files
				when saving very small file (too quickly!)

	* Rasterize tool
		- contour lines generation is now based on GDAL (more robust, proper handling of empty cells, etc.)
		- new option to re-project contour lines computed on a scalar field (i.e. a layer other than the altitudes)
			on the altitudes layer
		- the grid step bounds have been widened (between 1E-5 and 1E+5)

	* Edit > SF > Compute Stat. params
		- the RMS of the active SF is now automatically computed and displayed in the Console

	* PLY I/O filter
		- now supports quads (quads are loaded as 2 triangles)

	* DXF I/O filter
		- now based on dxflib 3.17.0
		- point clouds can now be exported to DXF (the number of points should remain very limited)
		- see fixed bugs below

	* LAS I/O filter
		- the 'Spatial Reference System' of LAS files is now stored as meta-data and restored
			when exporting the cloud as a LAS/LAZ file.

	* [Windows] qLAS_FWF:
		- the plugin (based on LASlib) can now load most of the standard LAS fields
		- the plugin can now save files (with or without waveforms)
		- the plugin can now be called in command line mode:
			-FWF_O: open a LAS 1.3+ file
			-FWF_SAVE_CLOUDS: save cloud(s) to LAS 1.3+ file(s) (options are 'ALL_AT_ONCE' and 'COMPRESSED' to save LAZ files instead of LAS)

	* New method: 'Edit > Waveforms > Compress FWF data'
		- To compress FWF data associated to a cloud (useful after a manual segmentation for instance
			as the FWF data is shared between clouds and remains complete by default)
		- Compression is done automatically when saving a cloud with the 'LAS 1.3 / 1.4' filter (QLAS_FWF_IO_PLUGIN)
			(but it's not done when saving the entity as a BIN file)

	* Oculus support
		- CC now displays in the current 3D view the mirror image of what is displayed in the headset
		- using SDK 1.15

	* Point List Picking tool
		- the list can now be exported as a 'global index, x, y, z' text file

	* Scale / Multiply dialog:
		- new option to use the same scale for all dimensions
		- new option to apply the scale to the 'Global shift' (or not)

	* New Menu Entry: 'Edit > Grid > Delete scan grids'
		- scan grids associated to a cloud can now be deleted (to save space when saving the cloud to a BIN file for instance)

	* qEllipser plugin:
		- option to export the image as a (potentially scaled) point cloud

	* Normal computation tool:
		- new algorithm to compute the normals based on scan grids (faster, and more robust)
		- the 'kernel size' parameter is replaced by 'the minimum angle of triangles' used in the internal triangulation process
		- Plane and Quadric modes will now automatically increase the radius adaptively to reach a minimum number of points and to avoid creating 'zero' (invalid) normals

	* Edit the scalar value of a single point
		- create a label on the point (SHIFT + click)
		- make sure a scalar field is active
		- right click on the label entry in the DB tree and select 'Edit scalar value'

	* Merge (clouds)
		- new option to generate a scalar field with the index of the original cloud for each point

	* Other
		- color scales are now listed in alphabetical order
		- polylines exported from the 'Interactive Segmentation' tool will now use the same Global Shift as the segmented entity(ies)
		- when changing the dip and dip direction of plane parallel with XY, the resulting plane shouldn't rotate in an arbitrary way anymore
		- the filter and single-button plugin toolbars are now on the right side of the window by default (to reset to the default layouts, use "Reset all GUI element positions" at the bottom of the Display menu)
		- the Plane edition dialog now lest the user specify the normal plane in addition to its dip and dip direction
		- new 'Clone' icon with a colored background so as to more clearly spot when the icon is enabled (Nyan sheep!)
		- now using PoissonRecon 9.011
		- the default maximum point size and maximum line width increased to 16 pixels

- Bug fixes:
	* STL files are now output by default in BINARY mode in command line mode (no more annoying dialog)
	* when computing distances, the octree could be modified but the LOD structure was not updated
		(resulting in potentially heavy display artifacts)
	* glitch fix: the 'SF > Gradient' tool was mistakenly renaming the input scalar field ('.gradient' appended)
	* glitch fix: the picking process was ignoring the fact that meshes could be displayed in wireframe mode (they are now ignored in this case)
	* command line 'CROSS_SECTION' option: the repetition of the cuts (<RepeatDim> option) could be incomplete in some cases (some chunks were missing)
	* raster loading: rasters loaded as clouds were shifted of half a pixel
	* the 'Edit > Sensors > Camera > Create' function was broken (input parameters were ignored)
	* merging clouds with FWF data would duplicate the waveforms of the first one
	* invalid lines in ASCII (text) files could be considered as a valid point with coordinates (0, 0, 0)
	* Point-pair based alignment tool:
		- extracting spheres on a cloud with Global Shift would create the sphere in the global coordinate system instead of the local one (i.e. the sphere was not visible)
		- deleting a point would remove all the detected spheres
	* The FARO I/O plugin was associating a wrong transformation to the scan grids, resulting in weird results when computing normals or constructing a mesh based on scan grids
	* When editing only the dip / dip direction of a plane, the rotation was not made about the plane center
	* qSRA plugin: profile polyline automatically generated from cylinders or cone were shifted (half of the cylinder/cone height), resulting in a 'shifted' distance map
		(half of the cloud was 'ignored')
	* DXF export
		- the I/O filter was mistakenly exporting the vertices of polylines and meshes as separate clouds
		- the I/O filter was not exporting the shifted point clouds at the right location
	* Render to file:
		- when the 'draw rounded points' option was enabled, pixel transparency could cause a strange effect when exported to PNG images
	* Octree rendering:
		- the 'Cube' mode was not functional
		- the 'Point' mode with normals was not functional

v2.8.1 (Hogfather) - 16/02/2017
----------------------

- Bug fixes:

	* LAS I/O filter: if the points of a LAS file were not saved in increasing GPS time order,
		the corresponding 'Time' scalar field could be rejected later at export time
	* A visible 2D label with the 'show 2D label' option disabled could break the picking process
	* The sphere fitting algorithm was not always finding the optimal sphere
	* Rasterize tool: scalar fields were vertically mirrored when exported to a geotiff raster
	* [macOS] Fix the packaging of the qAnimation plugin so it can find the correct libraries
	* qAnimation plugin: the export to separate frames was broken
	* the polyline width was not correctly set in the properties dialog (always set to 'Default')
	* the clipping box arrows were not displayed correctly when the lights were turned off

v2.8 (Hogfather) - 12/18/2016
----------------------

- New features:

	* New plugin: qBroom (Virtual Broom) [Windows, macOS]
		- smart and interactive selection of points on a surface (globally flat, e.g. like a road ;)
		- selection of the points inside the 'broom', above, below or both
		- option to invert the selection
		- automation mode

	* New plugin: qCSF (Cloth Simulation Filtering) [Windows, macOS]
		- automatic ground / non-ground classification of aerial LIDAR point clouds
		- based on the article: "An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation", W. Zhang, J. Qi, P. Wan, H. Wang, D. Xie, X. Wang, G. Yan. Remote Sensing. 2016; 8(6):501.

	* New plugin: qHoughNormals (Normal Estimation in Unstructured Point Clouds) [Windows, macOS, Linux]
		- based on "Deep Learning for Robust Normal Estimation in Unstructured Point Clouds" by Alexandre Boulch and Renaud Marlet, Symposium of Geometry Processing 2016, Computer Graphics Forum

	* New plugin: qM3C2 (Multiscale Model to Model Cloud Comparison) [Windows, macOS, Linux]
		- based on "Accurate 3D comparison of complex topography with terrestrial laser scanner: application to the Rangitikei canyon (N-Z)", Lague, D., Brodu, N. and Leroux, J., 2013, ISPRS Journal of Photogrammmetry and Remote Sensing

	* Support for FWF (Full WaveForm) airborne LIDAR data [Windows only]
		- use the 'LAS 1.3 or 1.4 (\*.las \*.laz)' filter to open LAS files with full waveform data
		- new 'Edit > Waveform > 2D Waveform viewer' to visualize waveforms associated to each point (as 2D curve)
		- option to export the waveform as a CSV file

	* New 'Geological' plane creation / edition methods:
		- Edit > Plane > Create: lets the user create a plane with specific dip / dip direction, center (can be picked on a cloud or a mesh), width and height
		- Edit > Plane > Edit: edit the above parameters on an existing plane entity

	* New Polyline export format:
		- 2D height profile (curvilinear abscissa + Z coordinate)

	* New tool: Edit > Mesh > Create surface between two polylines
		- Creates a surface patch between two polylines

	* New tool: Edit > Mesh > Mesh scan grids
		- Creates a surface from a cloud with one or several scan grids (one mesh per grid)

	* New tool: Edit > Color > Enhance with intensities
		- Enhances the RGB colors thanks to the intensity scalar field (RGB-IHS method)

	* Gamepad support (XBox, etc.) [Windows only]
		- Enable it with the 'File > Gamepad > Enable' menu entry
		- A: toggle viewer-based perspective mode
		- B: toggle object-based perspective mode
		- left stick: move body (viewer-based mode) or move object [left/right and forward/backward]
		- right stick: rotate head (viewer-based mode) or rotate object
		- cross: move body (viewer-based mode) or move object [left/right and up/down]
		- L1/R1: change point size
		- L2/R2: roll left/right
		- start: zoom and center on the visible entities

	* New color scales:
		- two colorscales for dip and dip direction display (thanks to T. Dewez)
		- matplotlib's veridis colormap (perceptually-uniform)

	* Adds an "Open Recent" item to the File menu to quickly access the last 10 files that you've worked with.

	* New 'display' option to draw round points instead of square ones (when the point size > 1).
		Warning, the display may be slower then.

	* New formats supported:
		- Photoscan PSZ
		- [Windows] Riegl RDBX

- Enhancements:

	* Animation plugin
		- new 'Export frames' button to generate individual frames instead of an animation

	* Poisson Reconstruction plugin:
		- based on the latest version of PoissonRecon by Misha (V9.1)
		- the user can now choose the boundary conditions (free / Dirichlet / Neumann)

	* Cross-section tool
		- the tool now supports multiple clouds and/or meshes

	* DB Tree (select children by type and/or by name)
		- regular expressions can now be used to select entities in the DB tree

	* Facets plugin
		- meta-data (normals, dip/direction, etc.) is now updated when the facet is rotated / transformed
		- the 3D representation of the normal vector now depends on the facet size and not on its min. bounding-box size

	* New shortcuts 'a la Meshlab':
		- CC now supports the 'CTRL + mouse wheel' shortcut to change the point size
		- CC now supports the 'ALT + mouse wheel' shortcut to change the zNear value (perspective mode)
		- CC now supports the 'SHIFT + mouse wheel' shortcut to change the field of view (perspective mode)

	* 2D labels:
		- most of the 2D labels parts (segment, point legend, etc.) are now displayed in 2D
			(this way they appear above the entities to be visible all the time).

	* SHP files:
		- when loading 2D points, the Z coordinate can now be exported from an associated DBF field

	* File loading:
		- when a multiple files are loaded, cancelling the loading process of one file will stop the whole loading procedure

	* LAS/LAZ files:
		- load dialog reorganized
		- info about the file are now displayed in a 'Info' tab (point count, bounding box)
		- new 'Tiling' option to tile a (big) LAS/LAZ file into smaller ones (the cloud is not actually loaded in memory)

	* Global Shift & Scale:
		- the PoissonRecon plugin now transfers the Global Shift & Scale information from the cloud to the resulting mesh
		- the 'Tools > Projection > Contour plot (polylines) to mesh' tool transfers the Global Shift & Scale information
			from the (first) polyline to the resulting mesh

	* 2.5D Volume Calculation tool:
		- new option to export the height difference grid as a cloud
			(warning: the exported points height will actually be equal to the height difference)
		- default color scale is now symmetrical if the height differences are not only positive or only negative

	* Rasterize tool:
		- the 'interpolate' option for empty cells now also interpolates the scalar fields and RGB color layers
		- the rasterize tool now uses the 'PixelIsArea' convention (i.e. the grid min corner coordinates correspond to the
			first grid cell center). This allows one to apply the Rasterize tool on a regular grid without any
			interference / sampling issues.
		- as a result, the volume calculation tool has been updated. Notably, results from the rasterize tool can be used
			in the 2.5D Volume calculation tool without any sampling artefact
		- exported rasters (geotiff) are using the same convention. They are also now properly oriented (they could be loaded
			flipped in some GIS tools).
		- ASCII matrix is now exported from top (highest Y coordinates) to bottom (lowest)
		- mixing RGB bands and other layers (heights, scalar fields, etc.) in a geotiff is in fact a bad idea. It results in
			64 bits color bands that are not properly handled by most of the other GIS tools. CC will now warn the user about
			this.
		- exported clouds and meshes are now properly exported in the same coordinate system as the input cloud
			(it was not the case for clouds projected along X or Y)
		- a coarse estimation of the grid volume (relative to Z = 0) is now available in the 'volume' tab

	* Raster file import:
		- New option to import the raster as a textured quad (mesh). Only available if the raster has at least R, G and B bands.
		- CC is now able to properly load raster files with multiple undefined bands

	* All the selected lines of the Console can now be copied at once (e.g. with CTRL+C on Windows)

	* Connected Components Extraction:
		- safeguard added if too many components are to be created (CC will ask the user to confirm the creation of more than 500 components)

	* CC should now warn the user when they try to save a file with a filename containing special characters when the third party
		library in charge of the export doesn't support them (see the warning Console message)

	* Display options
		- menu entry changed from 'Display parameters' to 'Display options' for the sake of consistency
		- new option: 'double sided' (light) to control whether triangles should be lit from the back or not

	* Apply Transformation tool
		- new 'Reset' button
		- new 'From dip / dip direction' to initialize the transformation as a rotation matrix passing from (0, 0) to the specified (dip, dip dir.) orientation

	* Normals computation
		- glitch fix: when computing normals with the 'LS plane' model, only 3 neighbors are requested instead of 12
		- CC now handles properly the (0, 0, 0) normal resulting from an insufficient number of neighbors
		- Dip / Dip direction computation takes the (0, 0, 0) normal and outputs NAN values for these normals
			--> this way it is possible to filter out the points with bad normals with 'Edit > SF > Filter by value' and using the full (valid) range

	* Other
		- Transformation history is now saved in BIN files
		- The normal of a plane entity can now be visualized as a 3D arrow (just as the Facet entities)
		- Dip and dip direction are now displayed in the properties of Facets and Planes
		- M3C2 sources are now publicly released! (GPL license)
		- New option to update an existing viewport object (with the current camera parameters)
		- [macOS] Now looks for the global_shift.txt file beside the .app instead of inside the application bundle
		- [macOS] Hides the 3D mouse and Gamepad menus since they are not yet supported on macOS
		- [macOS] Increases the default font sizes for the 3D viewer (these may be set in the Display Options)
		- I/O plugins (Faro, DP, Riegl, etc) are now loaded even when using CC in command line mode

- Bug fixes:
	* the custom light was broken (enabled and displayed in the 2D screen coordinates space instead of the 3D world!)
	* the 2D labels marker size (in 3D) was too dependent on the perspective camera position
	* the fields named 'intensity', 'grey', 'gray' or 'col***i' in PLY files are now automatically loaded as scalar fields (instead of 'intensity', i.e. grey level RGB)
	* when using a DBF field as height for polylines (in SHP files), the height value could be read from the wrong field
	* qSRA (Surface or Revolution Analysis): the profile origin was not always properly taken into account. The latitude angles were not always computed relative to
		the profile origin and changing the origin via the distance map dialog could be misleading.
	* when clicking on the 'Apply all' button of the LAS/LAZ loading dialog, the global shift set for the first file could be ignored by the subsequent ones
	* the 3-points label vertex names (A,B,C) were not displayed by default
	* Rasterize tool:
		- the 'Hillshade' computation tool was considering the grid upside-down (hence the sun azimuth angle was inverted)
		- contour lines generated with a projection direction other than Z were not displayed in the right orientation compared
			to the grid raster (inside the Rasterize tool only)
	* Command line tool:
		- the CROP2D option applied with an orientation other than Z was not performing the cut at the right position
		- the CURV option was no longer accessible
	* Connected Components Extraction: the tool couldn't be used properly with an octree level > 10
	* CC failed to save E57 files with multiple clouds with normals
	* ccViewer was transforming input arguments to upper case, hence preventing files to be opened this way on Linux
	* Documentation: contrarily to what was written in the wiki, the Level tool does not use the first picked point as origin for the new coordinate system!
		It only rotates the cloud about this point.
	* The new plane fitting algorithm (Least Squares fit) was giving strange results in some particular cases. Rolling back to the previous algorithm.
	* The sandbox tool 'Distance map to best fit 3D quadric' was broken
	* When computing normals with the Least Squares best fitting plane, CC required at least 12 points in the neighborhood, when only 3 are theoretically
		sufficient.
	* The SOR filter was broken (it was potentially using much more points than the number specified by the user, and it was changing over the cloud!)
	* Fixed a problem when creating a new 3D view with the view not updating until the window was resized or refreshed.
	* [macOS] Fix plugins on case-sensitive file systems
	* [macOS] Fix problem with the main window jumping around and resizing when dragging toolbars
	* [macOS] Fixes the layout of the 2.5D Volume Calculation dialog
	* Global shift & scale information could be lost when cloning entities or merging them with other non-shifted entities

v2.7.0 - 04/22/2016
-------------------

- Enhancements:
	* Cross section tool
		- Now based on OpenGL for a much faster display (+ proper integration in the new LOD mechanism)

	* Rasterize tool improved
		- Now handles RGB colors
		- Raster export dialog updated
		- The 'Cancel' button of the grid update progress bar is now handled properly
		- Now exports images with the active (default) color scale

	* Support for the Oculus Rift device
		- new option of the 'Stereo' mode (you have to use the 'Stereo' version of CloudCompare)
		- development is still in 'alpha' state
		- units must be expressed in meters
		- works best with a 3D mouse
		- works best in 'bubble view' mode (i.e. with FARO or PTX files)

	* The 'Global Shift & Scale' mechanism now allows for shifting values above 1.0e9
		(for georeferenced clouds expressed in millimeters!)

	* New 'File > Global Shift settings' dialog
		- can be used to set the limits that trigger the 'Global Shift & Scale' mechanism
		- settings are persistent
		- settings are used for both the standard application and the command line mode
		- default triggering value (for the coordinates) have been lowered to 10^4

	* New 'Tools > Sand-box > Distance Map' tool
		* allows to compute distance maps for clouds (= distances from regularly sampled positions around the points)
		* the output points can be filtered inside a specified distance range

	* The 'Edit > Multiply / Scale', 'Tools > Registration > Match bounding-box centers'
		and 'Tools > Registration > Match scales' methods now update the 'transformation history matrix' of the entities

	* Bubble View mode
		- now allows for horizontal AND vertical rotation
		- the 3D mouse is now properly handled

	* Command line mode
		- I/O plugins are now loaded in command line mode (i.e. the FARO, DP and PCD formats can now be used)
		- new option 'MAX_TCOUNT' to specify the maximum number of threads to be used for computing distances (C2C or C2M)
			as well as for ICP registration
		- the SPLIT_XYZ and MAX_DIST options can now be used together for C2C distance

	* Default light position
		- the default (sun) light position is now in the middle of the screen so as to get a brighter visualization of meshes

	* Distance computation
		- C2C distance can now be split along the 3 dimensions (X, Y, Z) even when using local models or a maximum distance
			(in which case some values may be NaN)

	* ICP registration
		- the user can now set the maximum number of threads/core to use

	* LOD mechanism enhanced (faster and smarter)

	* The 'Link camera' checkbox has been moved to the 'Display menu'

	* Mac OS X adds support for OpenGL Frame Buffer Objects (FBOs)

	* Mac OS X adds support for the CSV Matrix I/O file format

	* Mac OS X adds support for the following plugins:
		- qFacet
		- qAnimation

- Bug fix:
	* The HSV to RGB method was broken
	* The 'Convert normals to HSV colors' method doesn't rely on the Dip / Dip direction anymore as the way these values
		are computed have been changed recently (with a symmetry about the plane Z = 0)
	* When playing with the 'skip lines' parameter of the ASCII file loading dialog, the roles assignments could be cleared
		(when a line was reappearing while it had less elements than the other lines)
	* Mac OS X properly saves and restores the main window's state & geometry
	* Mac OS X uses the standard menu names and keyboard shortcuts for entering and exiting full screen

v2.6.3.1 - 03/17/2016
---------------------

- Enhancements:
	* Camera sensors can now be created 'freely' (i.e. not necessarily attached to a given entity) and moved freely in the DB tree
	* The cross section box orientation can now be setup via the 'advanced' button

- Bug fixes:
	* The meshes over the user-specified limit (display options) were ALWAYS decimated (and not only when the mouse is moved)
	* 2D area label picking was broken (first corner was misplaced)
	* 3D point picking on a cloud with a temporary GL transformation (e.g. in the Align tool) was not working properly
	* The camera sensor dialog was presenting the camera orientation vectors in the wrong order (horizontal instead of vertical)
	* OpenMP support disabled for Ransac Shape Detection on Windows (the process loops infinitely if enabled)
	* When custom labels were defined for a custom color scale, the values that were not present in the active SF range were ignored

v2.6.3 (= pre 2.7.0 for systems that don't support Qt5) - 03/13/2016
--------------------------------------------------------------------

- New features:
	* Polyline tracing/picking tool:
		- accessible via a dedicated icon in the main tool bar ('Trace a polyline by point picking')
		- allows to pick points on all the visible clouds in the active 3D view to define one or more 3D polylines
		- allows to generate a tight contour polyline by automatically picking several points between each vertex ('oversampling')

	* 2.5D volume calculation tool:
		- Tools > Volume > Compute 2.5D volume
		- can compute the volume of a single cloud (relatively to a constant height) or between two clouds
		- the tool projects the points in a 2.5D grid (therefore the interface is similar to the 'Rasterize' tool)
		- outputs several other statistics (added/removed volume, surface, etc.)

	* Exclusive full screen mode for 3D views

	* Support for the NVidia 3D Vision glasses (thanks to Amfax (UK) - www.amfax.co.uk)
		- new option of the 'Stereo' mode
		- the graphic card must support OpenGL quad buffering (i.e. latest GeForce or Quadro cards)
		- the 3D stereo mode must be enabled in the NVidia Control Panel
		- the screen frequency must be manually set to the right frequency (i.e. 100 or 120Hz) if not already
		- (the 3D view is forced to exclusive full-screen mode)
		- shaders (EDL, etc.) are supported

	* New method: Edit > Colors > Convert to grey scale

- Enhancements:
	* Point-pair based alignment tool:
		- the tool 3D view now has the same viewport/camera parameters as the source 3D view

	* Section Extraction tool:
		- new 'Unfold cloud' option: lets the user 'unfold' the cloud along the active polyline
			(very useful to unwrap a cloud that has not a cylinder shape)
		- the tool now keeps the active GL filter active (if any)

	* qPoissonRecon now uses PoissonRecon V8.0 (https://github.com/mkazhdan/PoissonRecon)

	* qAnimation:
		- the plugin has been fixed and can now output mp4 videos
		- the user now sees all the viewports in the list and can enable/disable each one separately
		- new 'loop' option (to close the loop)
		- the preview can be started from the selected step (see dedicated checkbox)
		- new 'super resolution' option (to smooth the output video)

	* Point sampling (on a mesh): maximum density is now 100 M. per square units

	* Sensors:
		- Sensors (camera, TLS) are now properly updated when their associated cloud is scaled
		- principal point added to Camera sensor parameters
		- differentiation of the vertical and horizontal focal lengths for Camera sensors

	* Command line mode:
		- new option '-ORIENT_NORMS_MST' (+ number of neighbors): to (re)orient the normals of the loaded clouds with a Minimum Spanning Tree
		- new option '-SF_OP' (+ SF index + operation + scalar value): to add / sub / multiply or divide a scalar field by a constant value
		- new option '-DROP_GLOBAL_SHIFT': to drop the Global Shift information of all loaded entities

	* SF Arithmetic tool:
		- a constant value can now be used instead of a second scalar field
		- the first scalar field (SF1) can now be updated directly (instead of creating a new SF)
		- division by zero is now properly handled

	* Structured point clouds:
		- the sensor position of associated scans is now properly updated when the cloud is rotated

	* M3C2 plugin:
		- new option to output the scalar fields (distance, confidence, etc.) on the original core point entity
			(i.e. either the original cloud or the subsampled version).
		- the user can now set the maximum number of threads/core to use

	* CANUPO plugin:
		- the statistics (Training mode) now takes the actual boundary into account (and not the default one)
		- bug fix: the classes (labels) could be inverted in some cases
		- the user can now set the maximum number of threads/core to use

	* Distance computation tool:
		- the user can now set the maximum number of threads/core to use

	* Rasterize tool:
		- the Rasterize tool can now compute Hillshade
		- the 'resample' option can now be used with the 'average height' projection. In this case
			only the coordinates of the original point are kept (the point which is the closest to the cell center)
			and the height is replaced by the average cell height.

	* Align tool:
		- the user can now set the max RMS for sphere detection

	* the FARO I/O plugin now relies on the 5.5.3 FARO LS SDK

	* The DP I/O plugin can now export DP files
		- either to save original DP files
		- or to convert other structured clouds (FARO or PTX for now) to the DP format

	* File I/O
		- E57: big coordinates are now properly handled (Global Shift & Scale mechanism)
		- Smarter progress dialogs

	* Cross section tool
		- new button to hide the clipping-box
		- new button to restore the previous clipping-box settings (per-entity)

	* Triangulation:
		- due to licensing issue with Triangle lib (incompatibility with GPL/LGPL) CGAL is now the prefered library for 2.5D
			triangulation CCLib can now be compiled with CGAL support using COMPILE_CC_CORE_LIB_WITH_CGAL.
		- please note that as CGAL 2D triangulation module is released under GPL license. A version of CCLib compiled with CGAL
			support is no longer licensed under LGPL (GPL is "viral"). To keep CCLib original license scheme, you *must*
			compile it without CGAL.
		- as a positive side effect, micro benchmarks show that CGAL tends to offer better performances.


- Bug fixes:
	* The 'Edit > Colors > Convert to Scalar Field' method was returning invalid scalar fields
	* The 'ADD_HEADER' and 'ADD_PTS_COUNT' options of the command line mode were causing an infinite loop
	* The point-pair based alignment tool could sometimes fail (especially when there were at least 4 picked
		points almost all in a plane)
	* CC 2.6.2 would crash when loading older BIN files containing primitive cylinders
	* FARO files: the sensor position associated to each grid/scan structure was wrong. This could lead to
		a wrong orientation when computing normals.
	* The rasterize tool was placing the points at the lower-left corners of the cells when exporting the grid
		as a cloud (instead of the centers of the cells)
	* The window picking slots were not properly disconnect after each session of the manual segmentation tool
		(at least) causing an increasing slow down each time the tool is restarted
	* Polylines vertices could be dragged and dropped or deleted (potentially causing a crash)
	* For clouds with a number of points that is exactly a multiple of 65536, the last 65536 points were not displayed
	* OpenGL warning removed when rendering a 3D view to a file (or as an animation with the qAnimation plugin)
	* Command line mode: the mesh obtained with the -DELAUNAY command could only be saved as a BIN file
	* Rasterize: applying the tool a second time on a cloud already generated with the rasterize tool could make CC crash
	* E57: some files may be rejected due to a minor warning (about duplicate extensions) that was considered as a critical error

v2.6.2 10/08/2015
-----------------

- New features:
	* New option in the context menu of the DB tree (right click on an item)
		- 'Select children by type and/or name' - to select items either by type and/or by name
	* New shortcuts:
		- Zoom in (+)
		- Zoom out (=)
	* New 'stereo' mode for anaglyph-like rendering
		- to be used with red-blue or red-cyan glasses
		- only works in perspective mode
		- Note that shaders (qEDL, qSSAO) are not supported in stereo mode
		- Shortcut: F10
		- Also supported by ccViewer
	* qAnimation plugin (initiated by 2G Robotics - http://www.2grobotics.com/)
		- animation rendering
		- prior to use the plugin, the user must create several viewport objects
			(with CTRL+V or 'Display > Save viewport as object') corresponding to
			the various 'keypoints' of the animation
		- the plugin can create video files (MPEG, h232, etc. - depending on the installed codecs)
	* qFacet plugin (created by BRGM - http://www.brgm.eu)
		- automatic fracture planes detection (by region growing)
		- orientation-based classification
		- stereogram display
		- stereogram based segmentation
		- export to SHP files
	* qGMMReg plugin
		- non-rigid registration of clouds (or mesh vertices)
		- based on the GMMReg library (https://github.com/bing-jian/gmmreg)
			(see "Robust Point Set Registration Using Gaussian Mixture Models", B. Jian and B.C. Vemuri, PAMI 2011)
		- meant to be used on small entities (i.e. a few thousand points)
	* Gridded/structured clouds handling
		- CC will now remember the structure of gridded clouds (PTX, DP, FARO, etc.)
			(see the new 'Scan grid' section of the cloud properties)
		- this structure can be used when computing normals for instance (see below)
		- this information is preserved if multiple gridded clouds are merged, cloned or segmented
	* Brand new 'Normal Computation' dialog
		- grid structures (see above) can now be used in the 'Normal Computation' dialog
			(they can be used for computation but they are more interesting to orient the normals)
		- new 'auto' button for the standard 'octree-based' computation algorithm (if a single cloud is selected)
			(CC will automatically estimate a good local radius for neighbors extraction)
		- new 'preferred orientation' option:
			'Use previous normal' to use the former normal (if any) to orient the new one
		- all the orientation options can now be set in the dialog (gridded/preferred/Minimum Spanning Tree)
			no more nagging about using the Minimum Spanning Tree and other questions...
	* New method 'Edit > Sensor > TLS/GBL > Compute points visibility'
		- if a TLS/GBL sensor is selected and this method called, the user can select any point cloud
			and CC will classify its points in terms of 'visibility' relatively to the selected sensor
			(classes are: VISIBLE / HIDDEN / OUT OF RANGE / OUT OF FIELD OF VIEW).

- Enhancements:
	* Rasterize tool
		- the user can now change the displayed 'layer' (either the height or one of the input cloud SFs)
		- the input cloud SFs can now be properly interpolated in empty cells
		- the contour lines are now computed on the active layer
		- the user can set the default width for the contour lines
		- the contour lines can be colored (according to the layer associated scalar field settings)
	* SOR filter:
		- the PCL Statistical Outlier filter (for noise cleaning) has been integrated in CloudCompare
		- can be accessed via 'Tools > Clean > SOR filter'
		- its icon has been moved to the main toolbar
		- the tool is now accessible through the command line (see below)
		- the other noise filtering tool has been renamed ('Tools > Clean > Noise filter')
	* Display:
		- CloudCompare now supports 'Level Of Detail' (LoD) display for big clouds
		- if enabled, the clouds are now displayed first at a low octree level when moved
			and are then regularly refined when the user doesn't interact with them.
		- the user can now specify the minimal number of points and the minimal number of
			triangles necessary to activate LoD/decimated display
	* The 'Edit > Crop' and 'Tools > Segmentation > Cross Section' tools can now be applied on
		triangular meshes. Moreover, the applied cut is clean (i.e. true re-meshing is performed
		on the edges)
	* Histogram dialog:
		- new icon 'Export to CSV'
		- new icon 'Export to image'
		- the SF parameter 'Show NaN values in gray' is now taken into account
			(i.e. the gray/hidden values won't appear in the histogram if this option is not checked)
	* Command line:
		- the 'Cross Section' tool (single or multiple slices extraction) can now be called via
			the command line (with a dedicated XML parameters file)
		- the SOR filter can now be called with via the command line:
			* 'SOR' + number of neighbors (knn) + std. dev. multiplier (nSigma)
			* the loaded clouds are replaced by their filtered version
		- the 'SF Arithmetic' tool can now be called via the command line:
			* 'SF_ARITHMETIC' + SF index + operation
			* supported operations are: sqrt, pow2, pow3, exp, log, log10, cos, sin, tan, acos, asin, atan, int, inverse)
		- new options for the 'ICP' tool:
			* 'DATA_SF_AS_WEIGHTS' + SF index (to use the given scalar field of the data entity as weights)
			* 'MODEL_SF_AS_WEIGHTS' + SF index (to use the given scalar field of the model entity as weights)
			* the 'LAST' option can be used instead of an explicit SF index
		- new options 'POP_CLOUDS' and 'POP_MESHES' to remove the last loaded cloud or mesh
		- new options for ASCII export:
			* 'ADD_HEADER' to add a header with each column's name to the saved file
			* 'ADD_PTS_COUNT' to add the number of points at the beginning of the saved file
	* Fine registration (ICP)
		- CC will now compute real cloud/mesh distances if the reference/model entity is a mesh
			* the registration will be much more accurate
			* (in this case the sampling parameter will only be used for the registered/data entity)
		- Weighted ICP is now functional
			* the user can use the currently active scalar field on the registered and/or the reference
			clouds as weights. The bigger the weights, the more influence the points will have.
			* weights are automatically normalized
			* only absolute values are considered
		- Initial point selection (when using a final overlap below 80%) is now much faster
	* Distances computation tool:
		- the dialog has been slightly updated (simplified)
		- the approx distances and the best octree level doesn't need to be automatically updated any time
			a parameter is updated anymore
		- C2C: if the 'reference' cloud has one or several associated sensors, the user can now check the
			new option 'use reference sensor to filter hidden points'. This will tell CC to use the sensor
			information to determine whether the points of the compared cloud were hidden at the time of
			the reference acquisition (and should therefore be ignored).
	* 'Render to file' method:
		- rendering zoom is now more properly taken into account when displaying 2D labels
		- zoomed text size is estimated in a more accurate way
		- the displayed size for the scale was wrong when the zoom was not 1
	* 3D views now recognize the 'pinching' gesture (with 2 fingers) on multi-touch screens (to zoom)
	* If available the version of the Visual Studio compiler and the version of Qt used to compile
		CC are now displayed in the 'About' dialog
	* PLY files with more than one texture can now be properly opened
	* ASCII save dialog:
		- the output precision of scalar values can now be set up to 12 (instead of 8)
		- Note that the internal representation of scalars remains 32-bits floats (i.e. with a relative precision ~10^-7)
	* Poisson Surface Reconstruction:
		- now using version 7 (https://github.com/mkazhdan/PoissonRecon/)
		- built-in color interpolation (faster & cleaner)
	* Ransac Shape Detection:
		- detected cones are now properly truncated so as to visually fit the associated subset of points
	* Point list picking:
		- label names are now preserved (if the user quits the tool, changes the labels name and restart the tool)
		- labels title in 3D have now a semi-transparent background
		- the point list can now be exported in a text file with 'label name, x, y, z' on each line
	* Profile extraction (both in the Cross Section tool and the Section Extraction tool):
		- new 'multi-pass' option to enable/disable the additional passes where longer edges can be generated
			(in order to ultimately get a better fit... or not). It was enabled by default in the previous version.
	* Console: multiple lines can be selected (and copied) at once
	* Color Scales Manager:
		- custom labels can be defined for each color scale (if that's the case then only those labels will be used
			when displaying the color ramp next to the cloud in the 3D view)
		- when the 'relative/absolute' parameter of an existing color scale is modified, the saturation of the scalar
			fields relying on this scale is now properly updated
		- new 'Apply' button to apply the modifications without leaving the tool dialog
	* the SOR and MLS filters of the qPCL plugin (Point Cloud Library wrapper) now preserve the global shift & scale information
	* Normals are now internally coded on 32 bits (instead of 16 bits) so as to increase the normals (direction) precision
		(instead of 32K different directions - which is visually sufficient - 2M directions can now be coded - which
		 is much better for normal-based computation, such as Dip/Dip direction computation or the new qFacet plugin)
	* Fit Quadric:
		- the matrix local quadric coordinate system is now displayed in the console along with its equation
	* qSRA:
		- the plugin can now be used directly with a cone or cylinder primitive (in place of a 'profile')
	* LAS files:
		- the 'time' field of LAS files is now automatically shifted at loading time so as to prevent any loss
			of accuracy (as this field is originally coded on 64 bits but then stored on 32 bits once loaded)
		- the original value of a single point can be queried anytime by spawning a label on this point
		- the original value is restored automatically when saving the cloud

- Bug fixes:
	* The point size couldn't be modified in the 3D view of the 'Rasterize' tool
	* The 'inverse (1/x)' function of the SF arithmetic tool was not accessible (the integer part was applied instead)
	* If only one plugin was loaded (with multiple methods such as qPCL) then the 'Plugins' menu was staying disabled
	* The scale value displayed in the 3D views (under the scale bar) was wrong when the zoom of the 'Render to file'
		method was greater than 1
	* The 'Apply all' option of the 'Global shift & scale' dialog wasn't taken into account when loading LAS/LAZ files
	* The main window could sometimes disappear in the background when the progress bar was appearing (Qt 5 issue)
	* Some options of the context menu of the DB tree were incorrectly using the term 'siblings' instead of 'children'
	* The VBO was not always updated when the cloud was translated (as a consequence, the clouds were not displayed in the right place)
	* While importing a Bundler file, if the images were to be 'kept in memory' but not the keypoints, then CC would crash
	* When saving a mesh composed of multiple sub-meshes to OBJ format, the triangles and texture information could be inconsistent
	* When merging two or more meshes, material were merged if their name were the same even if the texture or other parameters were different
	* When filtering a cloud through the command line (with FILTER_SF), the cloud was not updated in memory if the AUTO_SAVE feature was disabled
	* The 'o' keyword in OBJ files was ignored (this could led to missing textures, etc.)
	* Registration (either ICP or point-pair based picking) could crash in some particular cases
	* qSRA:
		- whether the profile origin is absolute or relative was not always taken into account!
		- a flat profile (e.g. perfect cylinder) would prevent the plugin from exporting DXF files
		- the 'units' field in the DXF export dialog was ignored
	* PTX files: the sensor position was overlooked (as it's almost always the same as the scan position... almost ;)
	* Section extraction: CC was crashing when extracting section clouds from multiple clouds at the same time
	* Command line: when automatically saving a file, CC will now add the suffix after the last point of the origin filename
	* qCork was crashing on Windows 8 (Cork also relies on the 'triangle' library ;)
	* FBX files: the exported 'specular' component of materials was in fact the 'ambient' component (resulting in a much brighter look)
	* In command line mode, the Delaunay command was ignoring the 'AUTO_SAVE OFF' option and was creating a looping hierarchy
		(potentially causing infinite loops later...)
	* OBJ files: when merging multiple models with textures having the same (local) filename, the images could overwrite each other once saved
	* Rasterize: saving a raster with the currently displayed scalar field as additional layer would make CC crash (+ memory leak fixed)
	* Point-pair based alignment: the markers picked on the aligned entity could become very big (or very small) when the scale was to be adjusted
		and the scale of the aligned entity was very different from the reference one.
	* The 'Apply Transformation' tool can now be applied on primitives

v2.6.1 02/20/2015
-----------------

- New features:
	* Level Tool
		- accessible via 'Tools > Level' or via the left toolbar
		- let the user pick three points (typically on the floor) so as to make the selected entity
			level (the selected entity can be a cloud, a mesh or even a group of entities)
	* New tool: 'Tools > Fit > Sphere'
		- to robustly fit a sphere on a cloud (up to 50% outliers)
		- automatically detects the center and radius
	* New tool: 'Tools > Register > Match scales'
		- to rescale a of entities so that they all have the same scale
		- several methods to compute the scale (longest bounding-box dimension, bounding-box volume,
			principal dimension according to PCA, ICP registration)
	* Section Extraction Tool
		- let the user define or import '2D sections' (polylines) over one or several clouds
		- 2D sections can also be generated automatically along a path with given step and
			width values
		- eventually cross sections can be extracted:
			* either as clouds
			* or as contours/profiles (either for the lower part of the section cloud,
				the upper part or the full section cloud)
		- output profiles can be exported to the 'Mascaret' profile format (see http://www.opentelemac.org/)
			or to the more standard DXF and SHP formats
	* Rasterize Tool (former 'Height Grid Generation' tool)
		- accessible via 'Tools > Projection > Rasterize (and contour plot)'
		- brand new interface with real-time preview of the raster
		- new option to fill empty cells by means of interpolation of the neighbors values
		- new tool to generate contour lines (aka isolines, contour plot, etc.)
		- the following scalar fields can now be generated when exporting the raster as a cloud:
			per-cell population, min height, average height, max height, height std. dev. and height range
		- the tool icon is now directly accessible in the main toolbar
	* SHP file I/O filter
		- supports polylines (input/output) and clouds (input/output) for now
		- input: if the SHP file contains 2D polylines and some numerical fields in the associated DBF
			file, CloudCompare will let the user choose one of those field as 'altitude' for the polylines
		- output: if the user wants to save 3D polylines, CloudCompare let him choose to export
			them as 2D polylines, and also to export their altitude (assuming it's constant) as a 'height'
			field in the associated DBF file
	* CSV Matrix I/O filter
		- to load CSV tables (typically with height values)
		- the output can be a mesh or a cloud
		- a texture file can be mapped on the grid:
			* if the output is a mesh, the image will be stretched to the mesh extents
			* if the output is a cloud, each image pixel color will be assigned the equivalent
				grid cell (therefore the image size must be equal or greater than the grid size)
	* New tool: 'Tools > Contour plot (polylines) to mesh'
		- triangulates one or several polylines (e.g. contour lines)
		- the triangulation tries to keep the input polylines as triangle edges
			(if the poylines are not crossing once projected in 2D)
	* DP file I/O filter (plugin)
		- to open DP (DotProduct - http://www.dotproduct3d.com/) files
		- imports each frame as a separate cloud (all frames can be merged afterwards with 'Edit > Merge')
		- creates the corresponding 'sensor' object for each frame
		- CC can compute robust normals at loading time (just as with PTX files)
	* Subsample tool:
		- when performing 'Spatial' resampling, the user can now choose to use the active scalar field
			to modulate the sampling distance depending on the scalar field values.
		- the user simply has to set the sampling distances associated to the minimal and maximal SF
			values and CC will linearly map the values in-between.
	* Color scale editor:
		- the icon is now directly accessible in the main toolbar
		- the user can export (and import) individual color scales as XML files
		- the color of new steps is now interpolated from the existing color ramp instead of being white by default
	* CPU-based point picking
		- CloudCompare now performs point picking (or triangle picking) with a CPU-based approach
			instead of relying on OpenGL. The process should be faster, especially on low-end
			graphic cards (Intel chipsets, etc.)

- Enhancements:
	* Fine registration tool (ICP)
		- new option 'Final overlap' to specify the theoretical (final) overlap of the clouds to register
		- this allows to register clouds with a quite small overlap (down to 10%)
		- the tool now only displays RMS errors (input parameter, progress bar and report)
		- the tool also output the number of points that have been actually used during the last
			iteration (and have therefore been used to compute the final RMS)
		- various glitches have been fixed
		- the default number of sampled points has been increased (50.000 instead of 20.0000)
	* C2C distances computation
		- computing distances with a 'max distance' boundary is now much more efficient
	* 2D labels:
		- brand new look for labels (values are displayed in a more compact and tabulated way)
		- new displayed information (dXY, dXZ and dYZ for 2-points labels, edges length for 3-points labels, etc.)
		- new default titles
			* single-point label: if the cloud has an active scalar field, the title incorporates the scalar field value
			* two-points label: the title is simply the distance between the two points
			* three-points label: the title is now the area of the corresponding triangle
		- more display options (opacity, marker color, label font size, etc.)
	* Contour extraction
		- contour extraction (either in the 'Cross Section' tool or the new 'Section Extraction ' tool is now smarter
		- a 'visual debugging mode' is also available to (try to) understand how the algorithm works and
			which parts of the cloud may cause a strange behavior / bad result
	* Poylines can now be associated to 'Global shift & scale' information
		- works just as point clouds
		- when creating a polyline (with the 'Point Picking List' tool, the 'Cross Section' tool, the 'Section extraction'
			tool, or 'Rasterize > Contour plot' tool, etc.) the global shift & scale information should be automatically
			transferred from the cloud
		- it should be properly restored when saving as SHP, Mascaret or Sinusx formats
	* Interactive Segmentation tool:
		The segmentation polyline can now be exported (to the DB tree) or imported (from the DB tree)
		- the polyline is always exported as a 3D polyline (i.e. the coordinates are relative to the segmented entities).
		- the current viewport is also exported (as a child of the polyline)
		- if such a polyline is imported later in the tool, CC will propose to apply the associated viewport so as the
			polyline is in the exact same position/orientation relatively to the segmented entities
	* Interactive Transformation tool:
		- the rotation center can now be modified with the 'Pick rotation center' tool (reticle icon in the left toolbar)
	* Point-pair based alignment:
		- the tool can now be used with meshes (points are picked directly on the mesh surface)
		- a cloud can be aligned with a mesh
		- the user can pick spheres (on clouds only) - just set the (very) rough sphere diameter and CC will do the rest
		- the tool can now properly align entities with only 3 pairs (even though it's better to pick more ;)
		- once at least 3 pairs have been picked, the tool will automatically display the current RMS and error per pair
			(the user still needs to click on the 'align' button to see the entities moving and to validate the tool)
		- the tool will now remember the main parameters
	* 3D views:
		- the rotation center of a 3D view can now be picked anywhere on the surface of a mesh
	* GUI frozen mode:
		- plugins toolbars are now properly deactivated
		- the GL filters toolbar is no longer deactivated (so as to let the user change the active shader anytime)
	* LAS/LAZ files support:
		- the original LAS offset information is now proposed by default as Global Shift
		- A dialog now appears at save time so as to let the user choose the scale values (important for compression
			when saving to LAZ format)
		- the original LAS scale is stored as meta-data so as to let the user choose to save the data with the same
			scale values (warning: accuracy might not be preserved if the cloud has been transformed, etc.)
	* Command line:
		- the command line tool can now load and process multiple meshes coming from the same file
		- New option '-DELAUNAY': to mesh the loaded clouds with Delaunay 2.5D triangulation
		- New option '-PLY_EXPORT_FMT': to specify the format of output PLY files (ASCII, BINARY_LE or BINARY_BE)
		- New option '-APPLY_TRANS': to apply a transformation (read from a simple text file with the 4x4 transformation
			matrix rows on each line)
		- New option '-REMOVE_ALL_SFS': to remove all scalar fields (from all loaded entities, i.e. clouds or meshes)
		- New option '-CBANDING': to apply color 'banding' (the user must also specify the dimension and the frequency)
		- the 'COMPUTE_PTX_NORMALS' option has been renamed 'COMPUTE_NORMALS' (as it can now be used for both PTX and DP files)
		- Default behavior of some commands has slightly changed (see http://www.cloudcompare.org/doc/wiki/index.php?title=CommandLine)
	* SF Arithmetic:
		- new operator 'INT' (integer part) to extract the integer part of scalar values
		- new operator 'INVERSE' (1/x)
	* Edit > Multiply/Scale
		- polylines can now be scaled (just as clouds and meshes)
		- new 'keep in place' option: the user can choose whether the entities (center) stay at the same position (new default) or not
		- if the coordinates after scaling become too big, CC will warn the user but will let proceed anyway
	* qSRA plugin (Surface of Revolution Analysis):
		- the conical projection now handles 'inverted' cases (with negative latitude values)
	* Curvature estimation:
		- a new curvature estimation has been added: 'Normal change rate'
		- faster to compute, smoother, and less sensitive to noise (but also less accurate, and dimensionless)
	* New display parameter:
		- the 'zoom speed' in perspective mode can now be set by the user
	* DXF filter:
		- the DXF I/O filter now handles global shift (i.e. large coordinates)
	* Transformation history is now properly maintained:
		- on a mesh vertices after cloning
		- when manually segmenting an entity
	* Meta-data are now preserved when cloning, subsampling or segmenting an entity
	* FARO I/O plugin:
		- the plugin can now load the RGB colors (instead of the reflection values - as a scalar field)
	* DotProduct I/O plugin:
		- updated SDK version (2.0) with bug fixes and other enhancements

- Bug fixes:
	- Delaunay 2D1/2 was crashing on Windows 8
	- the 'Sample points' tools was not handling texture coordinates with negative values or values above 1
		(i.e. repetitive textures)
	- In OBJ files (.mtl) material names containing space characters were not correctly handled by CC
	- when merging meshes with similar material names, 'funny things' could happen
	- option to save multiple clouds in a group as multiple ASCII files restored
	- CC would crash when selecting multiple entities in the 3D view (ALT + left mouse click to define
		a rectangular region)
	- the 'Edit > Colors > Levels' was not updating the VBO display when changing colors (so the user couldn't see
		any change on screen while the RGB colors were actually changed!)
	- FBX I/O filter: the transformation of meshes was considered as a column-wise matrix while it was in fact row-wise
	- STL vertices merging was sometimes failing with the 64 bits version as it was using a too small
		threshold distance (the bounding-box dimensions divided by 2^21!)
	- when comparing a cloud and a mesh, the best octree level guessed by CloudCompare could go much higher than 9
		(with the 64 bits version which has now a maximum octree level of 21 by default). However this could cause
		a much too high memory consumption and eventually a crash.
	- when the order of entities was changed in the DB tree, the display was not updated right away
	- the actual transformation applied to entities at the end of the 'Interactive Transformation Tool' could be slightly
		shifted in some cases
	- Regarding the local modeling option '2D1/2 triangulation' of the C2C distances computation:
		* it was much slower than it should (potentially much more points were triangulated than necessary)
		* it could crash if many duplicate points were present in the reference cloud
	- Performing the ICP registration with very few points (e.g. 4) could be much slower than expected
	- Files in a folder with 'local' language characters (accents, etc.) could not be opened by several I/O filters
		(LAS, E57, PCD and PDMS)
	- Memory leaks fixed in PDMS I/O filter (the R-TORUS primitive was not even imported to the main DB!)
	- Polylines could be selected when performing graphical segmentation (while they are not supported yet) causing
		a potential crash

v2.6.0 10/24/2014
-----------------

- New methods:
	* Tools > Clean > Noise filter
		- this tool is similar to qPCL plugin's SOR filter (Statistical Outliers Filter) but
		  with more options:
			* possibility to set a (sphere) radius for the nearest neighbors search instead of
			 a fixed number of points (much better if the cloud density is not constant)
			* possibility an absolute error instead of a relative error
			* possibility to remove isolated points by the way
	* Mesh quality measurement tool: 'Edit > Mesh > Flag vertices by type'
		- flags all the vertices of the mesh (with a dedicated scalar field) with different
			values depending on whether they are belonging to a normal edge (0), to an edge
			on a border (1) or to a non-manifold edge (2).
		- displays in the console the total number of each type of edges (normal/border/non-manifold)
			as well as the total number of edges
	* Mesh volume measurement tool: 'Edit > Mesh > Measure volume'
		- the result is only valid if the mesh is closed (CloudCompare will warn you if it's not the case
			and you can check that yourself with the new 'Flag vertices by type' method (see above).
	* Meshes can now be exported in DXF (Autocad) files
	* New 'Display' options:
		- 'Display > Lock rotation around vert. axis' to lock the manual rotation around the screen
			vertical axis (shortcut: 'L')
		- 'Display > Enter bubble-view mode' to enable 'bubble view' mode, a viewer-based perspective
			blocked on the current camera position, with Z axis locked and a high field of view by
			default (shortcut: 'B'). This is the new default mode enabled when clicking on the 'Apply'
			button of a TLS/GBL sensor (typically opened with a PTX or Faro file for instance)
	* New feature: "Transformation history"
		- All rigid transformations applied to any 3D object are now 'tracked'. In practice it is
			accumulated in a 'history' transformation matrix for each entity. For instance this allows
			to easily determine the global transformation applied to an entity since loading time (even
			after multiple manual transformation or automatic registration steps).
		- a dedicated widget ('Transformation history') has been added to the the 'Properties' dialog.
			This widget displays the matrix either as a "standard" 4x4 matrix, or as a rotation axis
			associated to a rotation angle and a translation vector. This widget also allows the user
			to copy the matrix to the clipboard or to save it as a text file (for opening it later in
			'Edit > Apply transformation' for instance)
	* New buttons in the 'Apply Transformation' dialog:
		- 'ASCII file' to import a transformation matrix from an ASCII file (see above)
		- 'Clipboard' to paste the clipboard contents (hopefully corresponding to a transformation matrix ;)
	* New plugin: qCork for Mesh Boolean Operation (= Constructive Solid Geometry)
		- this plugin is based on the LGPL Cork Library (https://github.com/gilbo/cork)
		- allows to compute the difference, union or intersection of two CLOSED meshes
		- current limitations:
			* doesn't keep the mesh(es) attributes (color, normals, etc.)
			* only available on Windows

- Enhancements:
	* For 64 bits versions, the max octree depth is now 21 instead of 10. This way CC will be more efficient
		on clouds above 20 or 30 M. points. The resulting increase in memory consumption shouldn't be an issue
		for recent computers (the new octree needs 12 bytes per point instead of 8, which corresponds to +380 Mb
		for 100 M. points)
	* The 'Edit > Merge' method now works with meshes
	* qPoissonRecon upgraded:
		- now uses the latest PoissonRecon version 6.11
		- screening activated by default
		- the user can now choose to output the 'density' parameter as a SF
			(this let him afterwards reduce the output mesh extents)
	* qEDL (Eye Dome Lighting) now behaves correctly even in perspective mode!
	* The vector coordinates have been restored in the 2-points labels (Point picking tool)
	* Texture handling:
		- textures are now handled in a much smarter and proper way (OBJ/PLY/FBX formats)
		- less memory consumption + original filenames are preserved
	* FBX file format:
		- Materials and texture are now supported (input/output)
	* New options in the Bundler import dialog:
		- Choose the keyponts/orthophotos vertical axis with radio buttons (X, Y, Z and Custom)
		- Choose the ortho-rectification method:
			* Optimized: the 'classic' method based on CC's own parameter optimization process
			* Direct: an alternate method based only on Bundler's output parameters
	* qSRA (Surface of Revolution Analysis) plugin:
		- the triangles of the exported meshes (surface of revolution textured with the displacement map)
			are now in the right order (right hand rule)
		- memory leak fixed (when displaying a conical projection map)
	* ASCII import/export:
		- RGB colors can now be imported or exported as float values (between 0 and 1)
			instead of unsigned bytes (0-255)
		- New 'Apply' button (replaces 'Ok')
		- New 'Apply all' button to attempt to load all the files with the same column assignation
	* PLY import:
		- New 'Apply' button (replaces 'Ok'): the dialog will remember the previous configuration and will try
			to restore it the next time
		- New 'Apply all' button : the dialog is skipped whenever CC can successfully restore the previous configuration while opening new files.
	* PTX import:
		- grid translation is now also checked for 'big coordinates' (Global Shift & Scale mechanism)
		- cancel button also cancels the normals computation!
		- sensors corresponding to each scan are automatically created
		- CC will now ask the user if the normals should be computed or not
		- when loading PTX file from the command line, normals are not computed by default.
			Use the option '-COMPUTE_PTX_NORMALS' to force their computation.
	* LAS import:
		- new handles LAS 1.4 files: the dialog has a new section that allows the user to load the
			custom fields defined in the LAS file 'extra bytes' section
	* VTK import:
		- VTK I/O filter is now smarter and can read Paraview VTK files with multiple elements per line
	* PCL import/export:
		- now fully integrated in the main application if the qPCL (I/O) plugin is present
			(i.e. all PCD files can be loaded and saved  via the standard 'File > Open' and 'File > Save' menus)
		- the equivalent icons of the qPCL plugin has been removed
		- ccViewer can also load PCD files with the qPCL I/O plugin
	* 'Edit > Sensors' menu has been reorganized
	* Camera sensor management enhanced:
		- Camera sensors are now created automatically when clouds are loaded from Bundler or ICM files (TODO: from E57)
		- A dialog let the user create or edit camera sensors
		- the camera symbol in 3D is fully configurable (display scale, with or without frustum, etc.)
		- camera sensors (and images) can now be saved in BIN files
		- the 'Camera Sensor > Project uncertainty' and 'Camera Sensor > Compute points visibility' are now
			smarter and can be applied on any point cloud
	* Ground Based Laser sensors management enhanced:
		- GBL sensors are now created automatically when clouds are loaded from PTX files (TODO: from E57)
		- the user can now apply the GBL sensor 'viewport' to the 3D display ("bubble view' mode)
			(new 'Apply' button in the sensor properties)
		- the sensor symbol in 3D is now correctly centered on the optical center
			(and can be used with the magnifier icon to zoom/focus on it)
		- 'Theta' and 'Phi' are replaced by the clearer 'Yaw' and 'Pitch' names
		- 'Edit > Sensor > View from sensor' method is now cleverer
		- 'Edit > Sensor > GBL Sensor > Show depth map' is now cleverer
	* Camera parameters dialog:
		- should not 'disconnect' from the current 3D view anymore
		- new setting: 'zNear relative position' to change the position of the near clipping plane
			in perspective mode (due to the way near and far clipping planes are handled in CC, this
			is only a relative position, and it cannot be set at any given depth).
	* New keyboard shortcuts:
		- 'A' to toggle all selected entities activation (i.e. enable/disable state)
	* Density computation:
		- only one method now with a dedicated dialog:
			* ability to choose either the precise or approximate methods
			* ability to choose how the density should be computed (number of neighbors, surface or volume)
	* Command line:
		- New options:
			* '-FBX_EXPORT_FMT' to specify the format when exporting meshes to FBX. Available formats
				are currently (FBX_binary, FBX_ascii, FBX_encrypted, FBX_6.0_binary, FBX_6.0_ascii, FBX_6.0_encrypted).
				If not set CC will display a message box to let the user select the output format (even in
				command line mode ;)
			* '-AUTO_SAVE' + ON/OFF: to set whether output clouds or meshes should be saved after each
				applied algorithm (currently ON by default)
			* new option after 'SAVE_CLOUDS' or 'SAVE_MESHES': 'ALL_AT_ONCE' to tell CC to (try to)
				save all loaded clouds (resp. meshes) in a single file. The format must support this
				features (i.e. BIN or E57 for clouds, BIN or FBX for meshes)
		- Global Shift can now be configured when opening a file in command line mode:
			After '-O' (open file) use the '-GLOBAL_SHIFT' option and:
				* either 'AUTO' to let CC handle the big coordinates automatically
				* or the 3 values of the shift vector to apply
			Example: 'CloudCompare -O -GLOBAL_SHIFT AUTO myfile.asc'
		- New option '-COMPUTE_PTX_NORMALS' to force CC to compute normals when loading PTX files
		- Default timestamp for output files now incorporates seconds (so as to avoid overwriting
			files generated too quickly)
		- New option '-TYPE' (after '-DENSITY' or '-APPROX_DENSITY') to specify the type of density to compute
			* options are: KNN (number of neighbors), SURFACE (surface density) or VOLUME (volume density)
	* qRansac Shape Detection plugin:
		- parameters have been updated so as to be (much) clearer
	* CC now detects points with NaN coordinates and will automatically replace them by (0,0,0)
		to avoid (big) issues later
	* Primitive parameters can now be interactively changed:
		- Sphere (radius), Cone (height, bottom radius, top radius) or Cylinder (height, radius) can now be
			modified directly via the 'Properties' dialog
	* New 'sand-box' methods:
		- Tools > Sand box > Create cloud from selected entities centers
		- Tools > Sand box > Compute best registration RMS matrix
	* Point list picking tool:
		- new checkbox to display the global coordinates (instead of shifted ones) for shifted clouds
	* 2D Labels:
		- For points with an associated scalar value, the scalar field name is used instead of 'Scalar'
			(e.g. 'Density = 1.23' instead of 'Scalar = 1.23')

- Bug fixes:
	* the spinboxes of the bounding-box editing dialog were oversized
	* the 'Cancel' button of progress dialogs was invisible on the x64 version
	* in some very limit cases, the distance computation could enter in an infinite loop
	* qSRA: memory leak fixed (when displaying a conical projection map)
	* the 'original' coordinates displayed in 2D labels (or in the console for picked points) was wrong
		(global shift was applied in the opposite way).
	* the global shift/scale infos were wrongly applied when exporting points picked with the
		'Point list picking' tool
	* for some clouds loaded from an ASCII file and with very particular sizes, the associated scalar fields
		size could be wrongly set (while the data was correctly loaded) and CC wouldn't accept to display them...
	* memory leak when loading PTX files
	* starting the 'Manual Segmentation tool' (scissors icon) in perspective mode could dramatically change
		the EDL rendering (anyway, using EDL in perspective mode is not really supported yet ;)
	* qPoissonRecon: the multi-threaded version (i.e. the 32 bits version on Windows) could crash sometimes
	* 3D mouse: on some configurations, the 3D mouse was not handled properly
	* The global shift information was not applied to the cameras when importing Bundler files with large coordinates
	* Saving E57 files could make CC crash sometimes (due to an internal inconsistency in libE57)
		+ CC is now more robust to this kind of errors!
	* 3D names now follow the entity when they are interactively translated (scissors tool)
	* Cloud cloning was not preserving the point size
	* When manually segmenting a mesh, its textures could be wrongly released from the OpenGL context
		(causing display errors in the best case)
	* Funny things (crash, etc.) could happen when choosing another option than 'leave empty' to fill empty cells...

v2.5.5.2 21/06/2014
-------------------

- Bug fix: primitives color couldn't be changed with 'Edit > Colors > Set unique'

v2.5.5.1 17/06/2014
-------------------

- Mac OS packaging issue

v2.5.5 07/06/2014
-----------------

- New methods:
	* Edit > Colors > Levels
		- allows the user to edit the color histogram (either all channels at once or channel by channel)
	* Display > Reset all GUI elements positions
		- removes all persistent information about the position and state of GUI elements (so that the
			GUI elements are back to their default on next start)
	* Edit > Scalar field(s) > Convert to random RGB
		- Converts a scalar field to random RGB colors: the user only specifies the number of random colors
			that will be generated and regularly sampled over the scalar field interval ([sfMin ; sfMax])

- Enhancements:
	* Point-pair based alignment tool:
		- minimum number of pairs set to 4 now (3 was not reliable enough)
		- the error contribution of each point (pair) to the total RMS is displayed in a new column
			(once the 'Align' button has been pressed)
		- a new button next to each point allows the user to delete any point at any time
		- the window rotation center can now be properly picked on the aligned cloud after having clicked
			on the aligned button
	* Point-pair based & ICP registration method:
		- the registration rotation can be constrained around one axis (X, Y or Z)
		- the registration translation can be constrained along one or several dimensions (Tx, Ty or Tz)
	* Global Shift & Scale mechanism:
		- brand new dialog (hopefully clearer)
		- A 'global_shift_list_template.txt' file can now be found next to CloudCompare's executable file.
		  The user can edit it and follow the instructions inside:
			* this file should be renamed 'global_shift_list.txt' and it should contain 5 values per line
			  ("name; Tx; Ty; Tz; scale;" - mind the semicolon characters)
			* all entries in this file will automatically be added to the dedicated combo-box of the new dialog
			* this should help a lot users working always with the same global coordinate system(s).
		- the 'Edit > Edit global shift' and 'Edit > Edit global scale' methods have been merged into a single
			method: 'Edit > Edit global shift and scale'.
			* It uses approximately the same new dialog but the user can now choose if the modification
				of the shift and scale parameters should impact the global coordinates system (i.e. at
				export time) or the local coordinate system (in which case the cloud(s) will be translated
				and or/rescaled automatically)
			* this new method can be called on several clouds at once
		- when a transformation (applied with 'Edit > Apply transformation') causes the cloud coordinates
			to go out of bounds, the same dialog appears in order for the user to optionally update the global
			shift/scale information instead.
	* Bundler (.out) import:
		- Big coordinates (for keypoints) are now properly handled (with the Global Shift & Scale mechanism)
		- The user can now input a 4x4 transformation matrix in order to change the keypoints orientation before
			generating the orthophotos (i.e. this lets defining a custom orthorectification 'Z' axis)
	* The 'SF arithmetics' method has been enhanced with new operations (applicable on a single SF)
		- SQRT, POW2, POW3, EXP, LOG, LOG10, COS(radians), SIN(radians), TAN(radians), ACOS, ASIN, ATAN
	* Point clouds are now pre-loaded in the graphic card memory if possible (via VBOs)
		- allows for much faster display (up to 15 times!)
		- this feature can be disabled in the "Display Options" dialog ("Other display options" tab)
	* Histogram display enhanced with the QCustomPlot library (http://www.qcustomplot.com/)
	* Scalar-field properties editing dialog enhanced:
		- sliders are replaced by interactors displayed over a representation of the SF histogram
		- round interactors are used to set the min and max displayed values
		- triangles interactors are used to set the min and max saturation values
	* 'Color' and 'Scalar field' visibility checkboxes in the entity 'Properties' dialog have been merged:
		- a unique combo-box now lets the user choose between no color, RGB or SF colors
	* ASCII files loading:
		- the dialog now handles the header line (if any) in a smarter way (the 'skip lines' count is correctly
			updated, the extracted header is displayed, etc.)
		- new command line option:
			* '-SKIP [number of lines]' after '-O' to specify the number of lines that should be skipped
		* new mechanism to detect the column type based on the header line (if any)
			* when loading an ASCII file in CloudCompare this should set the columns type automatically
			* when loading an ASCII file from the command line this should prevent the dialog from appearing
				(if standard names are used: X.., Y.., Z.., Nx, Ny, Nz, Normx, Normy, Normz, R, Red, G, Green, B, Blue, etc.)
	* New option in the LAS file opening dialog:
		- 'Force 8-bit colors': to cope with files in which colors are mistakenly coded on 8 bits (instead of 16)
	* The 'Poisson Surface Reconstruction' plugin (qPoissonRecon) is now based on the version 5.71 of PoissonRecon lib
		(http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.71/)
	* The 'Interactive Transformation' tool is now much more accurate
		- only double-precision matrices are used
		- before that, especially when lots of rotations were applied to an entity, the resulting transformation matrix
			could have had accumulated too many numerical errors resulting in a slightly shrunk cloud
	* (Mac OS X) Fonts on "retina" displays are no longer fuzzy
	* 3D mouse support: CloudCompare now relies on the official 3dConnexion SDK
		- wireless devices should now be handled correctly
		- the option menu is replaced by the official/default one
		- (support is still limited to Windows only for now)
	* Default point size editing dialog (appearing inside 3D views when the mouse hovers their top-left corner) is now
		displayed upon a semi-transparent background so as to be always visible even when displayed over a green entity.
	* Non ascii characters in filenames (accents, etc.) are now better handled by file I/O filters
		(all filters but PLY, OBJ and STL (output only), FBX, DXF, GDAL rasters, PCD
		 and older/internal formats such as POV, ICM, MA, depth maps)

- Bug fix:
	* Loading only the keypoints from a Bundler file (without the images) would result in a corrupted import
		(wrong values were read)
	* qPCL: bug solved in MLSsmoothing with colored clouds
	* Memory leak fixed (when using sub-meshes, typically coming from an OBJ file with multiples parts)
	* The scalar fields generated by the 'Height Grid Generation' tool were seen empty by CC

v2.5.4.1 04/22/2014
-------------------

- Bug fix:
	* Loading a PLY file with scalar field(s) would make CC crash

v2.5.4 04/19/2014
-----------------

- New file format supported:
	* OFF meshes
	* PTX clouds (import only)

- Enhancements:
	* 'Sensor' framework upgraded:
		- Camera 'projective' sensors added (early support)
		- 'View from sensor' method added
	* STL import filter is smarter (collapsed triangle after vertex fusion are now automatically removed)
	* Global shift:
		- a new button 'use last' will now appear on the 'Global shift/scale' dialog that
			popups when loading a file with very big coordinates (current session only - the button
			will appear when the dialog has been "accepted" at least once before)
		- the Z coordinate (if small) is not shifted by default anymore
		- shift/scale information should be (more) properly handled by both the Point pair based and
			ICP registration tools
	* The console visibility is now forced when a warning message is generated
	* The 'Point-pair based align' tool keeps the activated GL filter (if any)
	* Normal vectors can now be converted to two scalar fields:
		- 'dip' and 'dip direction' (see http://en.wikipedia.org/wiki/Structural_geology#Measurement_conventions)
			(the north is assumed to be +Y, and the altitude is +Z)
		- this new method is now in a common sub-menu with the "conversion to HSV" method
			(in 'Edit > Normals > Convert to')
	* The way the 'roughness' is computed (Tools > Other > Roughness) has been slightly changed:
		- for each point, the best fit plane is computed on all the neighbors except the point itself
			(this gives a less biased measure). The roughness value is then computed as the distance
			between the point and this plane.
	* New 'sand box' method: "Export cloud info"
		- exports various pieces of information for all selected clouds in a CSV file (cloud name, size,
			mean, std.dev. and sum of all scalar fields, etc.)
	* 'Camera link' feature enhanced:
		- modification of the global zoom (magnifier icon) is now propagated
		- modification of the rotation center is now propagated
	* 'Command line' mode enhanced:
		- loaded clouds and meshes can be now saved anytime with the '-SAVE_CLOUDS' and '-SAVE_MESHES' options
			(this is not necessary by default as all modified or newly created entities are automatically saved,
			but it can be useful for performing format conversion through the command line)
		- ASCII files with less than 6 columns are now loaded silently (i.e. no dialog should pop-up)
		- new option '-CROP2D' to crop a cloud inside a 2D polygon (either in the XY, XZ or YZ planes)
	* New method: 'Edit > Colors > Interpolate from another entity'
		- set the colors of a cloud (or mesh vertices) by interpolating the colors of another cloud (or mesh vertices)
		- right now the only interpolation mode available consists in taking the nearest neighbor's color
	* New method: 'Tools > Fit > 2D1/2 Quadric'
		- fits a 2D1/2 quadric on the selected cloud(s)
		- this quadric is represented as a 'primitive' (kind of mesh) and its equation is displayed in the console
	* The 'Subsample' method can now be called on multiple clouds at once
	* Cross Section tool:
		- contour extraction parametrization is much more clever (for 'single' and 'multiple' contour extraction modes)
		- the extraction with a rotated clipping box is now properly handled

- Bug fixes:
	* Information displayed in 2D labels was 'shifted' ( 'P(index,x,y)' was displayed instead of 'P(x,y,z)')
	* Depending on the selection order, CC could crash when merging two clouds (one with normals and
		the other without)
	* A too big step (causing a 1x1 cell grid) would make the Rasterize tool crash!
	* When scaling a cloud (with 'Edit > Apply scale') the 'global scale' information was (wrongly) updated.
		Therefore the cloud was rescaled to its original state at export time!
	* EDL / SSAO filters were performing badly on entities not centered on Z=0
		(OpenGL Z-buffer was badly initialized in ortho. mode)
	* When GL filters were enabled, labels text or the color ramp title were shifted in the Y direction
	* Command Line mode:
		- when loading a BIN file with multiple clouds inside, only the first one was kept!
		- when saving multiple clouds coming from the same file, all files had the same name
			(and were therefore overwritten)
	* Gaussian curvature formula was wrong (missing square exponent)
	* Several memory leaks have been fixed (2D1/2 mesh triangulation, etc.)
	* Cloning a mesh which was a child of its vertices would cause the duplication of the mesh structure
		in the resulting entity


v2.5.3 02/21/2014
-----------------

- New method: "Edit > Normals > Orient > With Minimum Spanning Tree"
	* This is an alternative method to find the correct orientation of normals based on the
		determination of a Minimum Spanning Tree
	* It may require a lot of memory but it works better on smooth shapes than the former
		method based on Fast-Marching (formerly called "Resolve normals direction" and now
		renamed and move to "Edit > Normals > Orient > With Fast-Marching")
	* It is also automatically suggested by CC if no preferred orientation has been set in
		the Normals computation tool.
- New method: "Tools > Other > Density > Accurate (at a given scale)"
	* More accurate method to compute the local density (= number of points inside a sphere
		centered on each point)
	* The old version is now accessible via "Tools > Other > Density > Approximate"
		(and it simply outputs the distance to the nearest neighbor now)
	* Command line options have been changed accordingly
		(see http://www.cloudcompare.org/doc/wiki/index.php?title=CommandLine)
- New method: "Tools > Other > Remove duplicate points"
	* Removes duplicate points (creates a new cloud)
- New method: "Edit > Crop"
	* To crop a point cloud inside a box
- New method: "Color banding" (thanks to M.J. Smith)
	* New option of the 'Edit > Color > Height Ramp' method

- Enhancements:
	* Spatial sub-sampling tool ('SPACE' mode):
		- about twice as fast
		- consumes much less memory
		- now the default method when the dialog opens
		- the slider has now a 'logarithmic' behavior (i.e. spacing value will grow much slower)
	* The ASCII file loading dialog is a bit smarter: you can now open ASCII files with
		invalid columns (such as text labels, etc.) as long as the column is not assigned
		to any property.
	* The OBJ importer now supports polylines ('l' tag)
	* Global shift/scale:
		- CC will ask the user when exporting points with the 'Point List Picking' tool if he wishes
			to keep global shift/scale information
		- CC will warn the user if the translation applied with the 'Edit > Apply transformation' tool
			is too big (in which case it can be set as 'global shift' instead)
	* New 'preferred' orientations for the Normals computation tool:
		- relatively to (0,0,0) (positive or negative)
	* Distance computation accuracy slightly enhanced (use of double precision internally)
	* The DXF I/O filter can now even load polylines with no descriptor
	* Command line mode has a lots of new options (thanks to A. Bevan, University College London)
		- computation of the best fitting plane
		- ICP registration
		- crop
		- make the (bounding-box) centers of loaded entities match
		- specify export format (clouds or meshes) as well as the extension
		- prevent CC from automatically a timestamp as suffix for output files
		(see the updated documentation here: http://www.cloudcompare.org/doc/wiki/index.php?title=CommandLine)
	* ccViewer now supports GL filters (i.e. 'shader' plugins: qEDL and qSSAO)
	* The GL filter banner (yellow) is now smaller, and is displayed at the top of the 3D view
		(it doesn't hide the scale anymore!)
	* Normals are no longer computed by default when loading a mesh without normals
	* Normals can now be computed on a mesh either 'per-vertex' (mean normal vector of
		all connected triangles) or 'per-triangle' (gives a 'faceted' look)
	* The 'Point pair-based registration' tool now handles virtual points with very large coordinates
		(useful to register a cloud with GPS control points for instance)
	* Labels now display both the shifted and the original points coordinates if
		the cloud is shifted/scaled
	* The VTK I/O filter now handles TRIANGLE_STRIP elements
	* Cross section tool: the box position can now be edited ('advanced' button)

- Other:
	* ATOM feed enabled on the forum (http://www.cloudcompare.org/forum/)
		(use the native ATOM support on IE or Firefox, or install an ATOM feed reader app
		on Chrome, e.g. "RSS Feed Reader")

- Bug fixes:
	* Deleting a 3D view and then selecting an entity previously displayed in this view
		would make CC crash
	* The default value for scaling of the 'shift on load' dialog was 0.
	* A bug in the (spatial) sub-sampling tool has been fixed (could make CC crash)
	* The cloud/cloud distance computation dialog was always computing the octrees twice!
	* DXF files generated by CloudCompare were corrupted (bug from dxflib)
	* The Z coordinate of polyline vertices exported in DXF format was wrongly replaced by the Y coordinate
	* Vertical profiles generated by the qSRA plugin were all the same
	* In some cases no SF or invalid SF were generated by the Rasterize tool (point cloud export)
	* The translation obtained by ICP registration with adaptive scale was wrong! (scale was not applied)
	* The 'repetitive slice extraction' option of the 'Cross section' tool was behaving strangely (some slices
		were sometimes fused together)
	* Pixel distances were squared twice in the Bilateral filter (shader)
	* The 'Point pair-based registration' dialog was not automatically reset
	* Wrong error code returned by PLY I/O Filter when failing to create the output file
	* Meshes color couldn't be changed anymore (with 'Edit > Set color')
	* GL filters rendering was downgraded when switching from perspective to ortho. view

v2.5.2 12/19/2013
-----------------

- GDAL library support added:
	- CC can now load dozens of standard GIS 2D1/2 raster formats (Arc Grid, GeoTiff, etc.)
- 'Height grid generation' tool enhancement:
	- when selecting the 'min' or 'max height' projections, the user can now choose
		to 'resample' the original cloud in order to produce a new cloud (instead
		of generating a regularly sampled cloud using the grid cell's centers)
	- the tool can now export the resulting grid as a true multi-band raster (geotiff)
	- menu entry renamed: "Tools > Projection > Rasterize (Height grid generation)"
- It is now possible to compile CloudCompare with 64 bits floating point values
	(i.e 64 bits 'doubles' instead of 32 bits 'floats') for coordinates and/or scalars.
	It doubles the memory consumption but it increases a lot the accuracy.
	Moreover there's absolutely no loss of information when importing/exporting clouds
	with very big coordinates (however OpenGL still requires rather small coordinates
	for a proper display, therefore the 'shift on load' mechanism might still be necessary)
- New tool: 'Display > Adjust zoom'
	- let the user set the current window zoom either directly or by specifying a pixel
	size in the (implicit) units of the current entities.
- New tool: Edit > Scalar fields > Set SF as coordinate(s)
	- allows the user to map a scalar field to one or several dimensions (X,Y or Z)
	- useful to convert a 2D raster with a scalar field (gray level, etc.) to a 2D1/2 point cloud
- New method: File > Close all
	- to remove all loaded entities

- New formats:
	- Autodesk(R) FBX (http://en.wikipedia.org/wiki/FBX)
		* meshes only
		* support for materials is planned but not finished yet
	- Any geo-localized raster (geotiff, ArcGrid, etc.) thanks to GDAL library (http://www.gdal.org/)
		* on input: use 'File > Open' (raster will be converted to a point cloud)
		* on output: use the Height Grid Generation tool (this way, any cloud can be converted to raster)

- New plugin: qSRA (Surface of Revolution Analysis)
	- for comparison between a point cloud and a surface of revolution
	- generates a map of deviations (with cylindrical or conical projection)
	- exports resulting map as an image, a textured mesh or a point cloud
	- can generate vertical and horizontal profiles in DXF format

- Other enhancements:
	- 'Clipping-box' tool:
		* the clipping box position and extension can now be edited
			(see the 'advanced' button in the 'Box thickness' frame)
		* when extracting contours, CC will now ask the user if he wishes/accepts
			to split the initial contour in several parts so as to really respect
			the 'max edge length' parameter (this gives a much nicer result).
			This feature works both with the single and multiple slice extractors.
	- the qPoissonRecon plugin is now based on the latest version of PoissonRecon (5.5)
		* see http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.5/
		* new parameters dialog
		* the input cloud colors can now be mapped on the resulting mesh (quick & dirty approach:
			simply assigns to each mesh vertex the color of the nearest input point)
	- the 'Edit > Transformation' tool now offers 3 different ways to input a transformation:
		* classical 4x4 transformation matrix
		* rotation axis, rotation angle and translation vector
		* Euler angles and translation vector
	- if you paste a transformation matrix copied from the console, the 'Edit > Transformation' tool
		will now automatically remove the timestamp (between square brackets)
	- when using 'Local models' when computing cloud-to-cloud distances, CC will now take the
		smallest distance between each point and either the local model or the nearest neighbor
		(in order to avoid clearly erroneous distances due to badly shaped local models).
	- the 'facet' entity's normal vector can now be displayed or hidden via the facet properties
	- BIN file loading/saving is now performed in a separate thread
		* a progress dialog is now displayed during loading/saving
		* multiple loading sessions can be done concurrently (use drag & drop on a 3D view
			- note: only interesting when loading files from different drives)
		* additional check added to detect corrupted meshes
	- Global rescaling applying at loading time is now properly handled
		* works just like global shift
		* appears in the entity properties as well
		* dedicated menu entry (Edit > Edit global scale)
	- pause button added to the "Graphical Transformation" tool (to allow rotating/panning the 3D view)
	- new shortcuts added to the "Graphical Transformation" tool:
		* space bar = pause/unpause
		* return key = apply transformation (close the tool)
		* escape key = cancel transformation (close the tool)
	- new shortcuts added to the "Graphical Segmentation" tool:
		* space bar = pause/unpause
		* tab key = switch between polygonal and rectangular selection mode
		* I key = segment points inside
		* O key = segment points outside
		* return key = apply segmentation (close the tool)
		* delete key = apply segmentation and delete hidden points (close the tool)
		* escape key = cancel transformation (close the tool)
	- New types of entities can now be cloned:
		* polylines
		* facets
	- Max coordinate absolute value lowered (100.000 instead of 1.000.000)
		in order to avoid accuracy loss and display issues. However the user
		still has the choice to ignore CC's warnings.
	- For the sake of consistency and clarity:
		* the 'Free scale parameter' option of the ICP registration dialog and the 'fixed scale' option
			of the Point-pair based alignment dialog are both renamed 'Adjust scale'
		* the 'Edit > Synchronize' tool is moved and renamed 'Tools > Registration > Match barycenters'
		* the ICP registration tool now issues the same textual report as the Point-pair based alignment tool
	- the Connected Components tool dialog now:
		* displays the octree cell size as the currently selected level
		* the components are automatically 'selected' in order to clearly identify them without random colors
			(the 'random colors' checkbox is not selected by default anymore)
		* the components are sorted by their size (starting from the biggest)
	- new command lines options (see http://www.cloudcompare.org/doc/wiki/index.php?title=CommandLine):
		* C2C_DIST: cloud to cloud distance computation
		* C2M_DIST: cloud to mesh distance computation
		* SAMPLE_MESH: to sample points on a mesh
		* FILTER_SF: to filter a cloud based on its scalar values
		* STAT_TEST: to apply the local statistical filter
		* MERGE_CLOUDS: to merge all loaded clouds
	- 'Edit > Fuse' entry renamed 'Edit > Merge'

- Bug fixes:
	- the 'dip' value (in "dip & dip direction" computation with e.g. the plane orientation tool)
		was inverted from the currently accepted definition (i.e. 0° for horizontal and 90° for vertical planes)
	- blank lines or commentaries were causing an infinite loop when importing ASCII files
	- primitives (sphere, cylinder, etc.) were not correctly loaded from BIN files (CC would crash when loading them)
	- sub-meshes' bounding boxes were not updated when a transformation was applied to their parent mesh or its
		associated vertices
	- at loading time when an entity was both too far and too big, the (optional) applied transformation to
		recenter and rescale the entity was wrong.
	- a call to the 'Zoom on selected entities' tool on entities not displayed in the active window would be ignored.
	- the 'camera link' tool would sometimes make CC crash (windows were playing ping-pong ;).
	- CloudCompare can now recover from errors encountered in OBJ normals or texture coordinates (normals or textures
		are simply discarded)

v2.5.1 10/19/2013
-----------------

- The 'Clipping-box' tool has a new feature: 'contour' extraction
	* Contour extraction on the current slice
	* Contour extraction on multiple slices (with the 'repeat' button)
	* Early prototype: computes the concave hull of the slice's points
		- one parameter: maximum edge length (if possible)
		- the smaller the finer the contour (+ the slower)
	* Exports result as one (or several) poyline(s)
- The 'plane orientation' tool is replaced by two tools:
	* Tools > Fit > Plane: same as the old tool
	* Tools > Fit > 2D polygon: almost the same, but the fitted entity is a facet
		(composite entity with a polyline corresponding to the contour and a mesh
		corresponding to the 'inside')
	* Both tools now output the orientation with the "dip direction / dip angle" convention
		(see http://en.wikipedia.org/wiki/Structural_geology#Geometries)
- 3D polylines can now be defined with the 'Point List Picking' tool
	* select a list of points then 'export' it as a polyline
	* the polyline displayed width is customizable
	* the 'Tools > Other > Plane orientation' tool can be used on polylines
- DXF (Autocad) support:
	* only to output polylines for now
	* warning: DXF can save 3D coordinates but Autocad doesn't seem to handle them properly.
		The user is advised to save polyines in the (X,Y) plane only.

- Other improvements:
	* Mesh groups are replaced by standard meshes with 'sub-meshes' structures (simple subsets of faces)
	* OBJ materials (and textures) export is now supported
	* New option for Delaunay mesh computation:
		* a max edge length can be specified to automatically remove elongated triangles
	* New check-box in the LAS open dialog: apply same parameters to all files that will be opened next (current session)
	* User can now choose whether overlay items (scale, trihedron, etc.) should be rendered or not by the
		'Display > Render to file' tool
	* the progress bar "Cancel" button has no "focus" anymore:
		for instance, this allows the user to keep the "Enter" key pressed while loading multiple files with the default
		loading parameters (i.e. manually "skipping" the opening dialog for ASCII files for instance)
	* the 'Align camera' tool (right-click context menu for planes, 2D polygons, labels, etc.) now outputs the applied matrix
		so that you can apply it with the 'Edit > Apply transformation' tool to actually rotate an entity the same way
	* Polyline's length now appears automatically in the 'Properties' view

- bug fixes:
	* text was misplaced when rendering screen with a zoom factor greater than 1.
		Other features were also badly reshaped (fonts, scale, color ramp, trihedron, etc.)
	* the 'max distance' parameter for cloud/mesh distance computation was incorrectly squared before being used!
	* materials where not associated to the base mesh when loading composite OBJ files (this could lead to crash when
		saving the composite mesh as PLY for instance)
	* curvature or normal computation would sometimes fail on perfectly flat axis-aligned clouds (division by zero).
	* the 'multiple slices auto extraction' tool (clipping box mode) would crash if at least one 'repeat dimension'
		was unchecked.
	* Height Grid Generation tool would crash when generating a grid with SF interpolation and the 'leave empty'
		option for empty cells.
	* when cloning a cloud, the scalar field display options (color scales, steps, etc.) were not duplicated.
	* when trying to save an entity which name made an invalid filename, the file saving dialog wouldn't open
		(now a warning message is issued and the default 'project' name is used instead)
	* Graphical segmentation area type selection icons would 'disappear' when toggled

v2.5.0 07/12/2013
-----------------

- New version numbering scheme (build index appended instead of the release date)
- New tool: "Cross section"
	* accessible via 'Tools > Segmentation > Cross Section' or a dedicated icon in the main
		toolbar (next to the scissors icon)
	* manually scale, rotate and translate the clipping box
	* extract the corresponding selection as a new cloud
	* extract multiple slices in an automated way
	* works only with clouds for the moment
- New DB tree context menu options:
	* right click on a plane primitive or a triangle label and choose 'Align camera' to make the camera look perpendicularly to it
	* same thing with 'Align camera (reverse)' (to look in the opposite direction)
- New option for ICP registration (thanks to Luca Penasa once again!):
	* check "Free scale parameter" to register objects with different scales
- New dialog for computing the octree:
	* the user can specify the cell size at the maximum subdivision level
	* or define a custom bounding box
- "Height grid generation" dialog enhanced:
	* the user can now define custom grid limits
	* the tool spawns dialogs to let the user specify the output files (ascii grid and image)
- New tool: "Convert RGB to scalar field"
	* Edit > Colors
- New option in ASCII file load/save dialogs:
	* maximum number of points per cloud on load
	* option to write the number of points as a dedicated header line on save
- Menus modifications:
	* "Primitive factory" menu entry has been moved to the "File" menu
	* "Point picking" and "Point list picking" menu entries have been moved to the "Tools" menu
	* "Edit" and "Tools" menus are now always enabled even if no entity is selected
- The (old) limit of 128 million points per cloud has been removed on the 64 bits version
	(new - theoretical - limit is 2 billions)
- Global shift information can now be edited with the 'Edit > Edit global shift' menu entry
- LAS import/export enhanced:
	* new dialog to choose the fields to import
	* all official fields are now properly imported/exported ("Intensity", "Return Number",
		"Number of Returns", "Scan Direction", "Flightline Edge", "Classification",
		"Scan Angle Rank", "User Data", "Point Source ID", "Red", "Green", "Blue", "Time")
	* the 'classification' field can be split into its 'value' and the 3 associated
		flags ("Synthetic", "Key-point" and "Withheld")
	* a warning is issued if the user attempts to save a cloud with SFs having non "official"
		names
- Stippling (fake transparency) option has been added to meshes
- ccViewer enhanced:
	* GUI updated (pivot visibility options added, isometric default views, new icons, etc.)
	* PDMS format support added
	* 3D mouse support added

- Misc. improvements:
	* new icon for the 'clone' method (thanks to JF Hullo): two sheep instead of one!
	* when computing local density, if two or more points are overlapping, the result density
		will be 'NAN' and the point will appear in gray (in order to be hidden or removed easily)
	* global shift information is now kept by qRansacSD plugin (shape detection)
	* qHPR (Hidden Point Removal) plugin now generates a new cloud with the visible points only
		(instead of hacking the points visibility) along with the corresponding viewport
	* scaled transformation matrix are now properly inverted (i.e. scale is taken into account)
	* point-pair based alignment tool outputs a summary on completion (RMS, scale, etc.)
	* scale estimation during point-pair based alignment has been enhanced

- Bug fixes:
	* applying the interactive 'rotate/translate' tool to an entity and its parent at the
		same time would make CC crash
	* "Intensity", "Time" and "Return number" fields are now properly exported with LAS files
	* "Height Grid Generation" tool accuracy fixed (when dealing with coordinates above 10^5)
	* mesh with a single texture AND colors couldn't be saved properly in PLY format (CC would
		refuse to save the texture)
	* deleting a child object (e.g. labels) in ccViewer would make it crash!
	* old viewports (prior to version 25) were wrongly imported (panning info)
	* deleting groups containing clouds used by shared labels would make CC crash

v2.4 04/25/2013
---------------

- 3D mouses (3dConnexion devices) are now supported:
	* OBJECT mode corresponds to ortho. and object-centered perspective
	* CAMERA mode corresponds to viewer-centered perspective
	* can be enabled/disabled via the 'File > 3D mouse' menu
- Dynamic color lookup for displaying scalar fields greatly accelerated
	* a dedicated shader is automatically activated if GPU supports it
	* speed-up of scalar fields display of up to 300%!
- Introducing the "Color Scale Manager":
	* edit, copy and create custom color scales
	* accessible via a button next to the 'current scale' combo-box in the properties view
		or via the 'Edit > Scalar Field > Color Scale Manager' menu entry
	* custom scalar scales are saved along with the entity (in BIN format)
		and are automatically imported when loading the file (on another computer typically)
	* color scales can be relative (to current scalar fields bounds) or absolute (the user
		can specify custom bounds and steps, which can be useful to apply the same color ramp
		to multiple clouds)
- New Scalar Field Properties dialog
	* most color ramp parameters are now saved per-scalar field, and not per-cloud anymore
- New color ramp (with histogram)
	* color ramp is "smarter"
- New entity picking mechanism:
	* press ALT + left mouse button to define a rectangular selection area to select multiple
		entities at once
	* picking speed enhanced
- 3D camera management enhanced:
	* transitions between perspective modes are smoother
	* custom light appears and can be interactively moved (with CTRL+right click) in all modes
	* interactive transformation can be performed in all modes
	* 3D rotation center symbol added (can be 'always visible', 'only when moving' or 'hidden')
	* default views (top, bottom, left, etc.) are more 'intuitive'
	* new default views added: front and back 'isometric' views
	* default FOV (field of view) is now 30 degrees for perspective mode
- New method:
	* 'Edit > Mesh > Export materials/textures to RGB' (per-vertex color)
- New pop-menus added to the 'View' toolbar:
	* selection of the current rotation center visibility (see above)
	* selection of the current view mode (orthographic, object-based and viewer-based perspective)
- New context menu entry for the DB tree: 'Information (recursive)'
	* recursively gather various information (number of points, triangles, colors, etc.)
- Local Statistical Test now works on signed scalar fields!
- No more distinction between (antiquated) 'positive' scalar fields and the standard ones
- New dialog for saving ASCII files
	* choose output numerical precisions, separator, order, etc.
- STL and PLY files can now be saved in both BINARY and ASCII formats
- PLY files can now save texture coordinates (and the texture in a separated file) if only one
	texture is associated to the mesh (otherwise CC will suggest to convert materials/textures to RGB)
- Default materials and lights have been updated

- Bug fixes:
	* BIN files saved with labels would make CC crash when re-opened!
	* Closing the 'About' dialog would make CC crash!
	* ccViewer no longer displays the colorbar if the scalar field is hidden!
	* The cloud/cloud distance with the 'Height Function' model refinement was giving weird results!
	* The cones produced with the Primitive Factory were upside down
	* The following methods were broken:
		- Edit > Normals > Resolve direction
		- Tools > Projection > Unroll (on cone)
		- Tools > Statistics > Local Statistical Test

v2.4 03/10/2013
---------------

- Points size can now be independently set for each cloud
	* see the 'Point size' combo-box in the cloud's properties
- The 'Align with point pairs' tool can now be used with only one cloud selected
	(in which case reference points must be manually added with the 'pencil' icon)
- Point picking mechanism enhanced:
	- point marker is now a (selectable) 3D sphere
	- point marker default size is customizable via display options dialog
	- mesh triangles can now be picked (early version: creates a 3-points label)
	- "Point list picking" tool
		* automatically pastes last picked point in clipboard
		* string format: " CC_POINT_#i(x,y,z) " (where i is the index in the current list)
	- "Point picking" tool:
		* when picking 3 successive points ('triplet' mode), the 3 corresponding angles are displayed
- New tool: 'primitive factory'
	* lets you create custom primitives (plane, box, sphere, cylinder, cone, torus or dish)
	* accessible via the 'Tools > Primitive factory' menu entry or a dedicated icon in the main toolbar
- 'Height grid generation' tool enhanced:
	* the number of points per cell can now be saved as a scalar field ('save per-cell count as SF' checkbox)
- qRansacSD ('Ransac Shape Detection') plugin enhanced:
	* proper handling of cone and torus primitives (display in 3D, etc.)
	* the expected primitive types can now be selected (with checkboxes)
- qKinect plugin enhanced:
	* continuous display of video/3D flow
	* multiple acquisitions are now possible without closing the dialog each time
- Manual transformation tool enhanced:
	* user can constrain rotation around a given axis and translation along custom dimension(s)
- New option "Show 3D name" for 3D entities:
	* display the name of the entity in the 3D view (in the middle of its bounding-box by default)
	* can be (recursively) toggled with 'Edit > Toggle > 3D name' (shortcut: D)
	* with the dedicated checkbox in the properties dialog of each entity
	* with a right click on the entity entry in the DB tree
- "Toggle materials/textures" entry added to the DB tree context menu (and in the 'Edit > Toggle' sub-menu)
- New display options (display dialog):
	* point marker size (see above)
	* histogram background color
	* label color
- New option for computing normals (CC's way, not qPCL's)
	* +/-barycenter: normals are oriented by pointing outwards or towards the barycenter
- Limits for auto-decimation are now:
	* 10M. points for a cloud
	* 2.5M. faces for a mesh
- New command lines for ccViewer:
	* '-top' to make the window 'always on top'
	* '-win [X] [Y] [W] [H]' to set the position (X,Y) and size (W,H) of the window
- New icons for CloudCompare and ccViewer
- PCL (Point Cloud Library) 'PCD' files can now be opened without the qPCL plugin
	* only works with ascii and uncompressed binary versions for the moment

- Bug fixes:
	* in the 'Point list picking' tool, when exporting a list of picked points THEN canceling the picking process, CC will crash
	* SF scale is now rendered by the 'Render to file' tool
	* qPCL's MLS tool would crash if the input cloud has no displayed scalar field! (+ dependency to Luca's PCL patch is now optional)
	* 'shift on load' information is now preserved when sampling points on a mesh (and during several other operations)
	* a group of labels could be freely displaced, eventually leading to a crash (under some circumstances) if the destination is not
		one of the label's associated clouds. On the contrary, labels can now be regrouped and displaced as long as they stay below one
		of their associated clouds.
	* applying an interactive transformation to an entity with an octree could make CC crash (octree was deleted but not removed from DB tree)
	* CC would refuse to open an OBJ file declaring using a material while not defining texture coordinates

v2.4 01/21/2013
---------------

- New tool: 'Edit > Mesh > Subdivide' (recursive subdivision)
	* interpolates mesh original colors (if any)
	* only works with single meshes for the moment (not with mesh groups)
	(+ 'Edit > Mesh > Measure surface' now also outputs the mean triangle surface)

- 'Set color' and 'Colorize' can now be applied to groups

v2.4 01/13/2013
---------------

- LAZ files support (with laszip - thanks to Ingo Maindorfer)
- STL files support (certainly the ugliest mesh file format ever created, but definitely a largely used one...)
- New tool: export clouds coordinates (X, Y or Z) as scalar fields
	* you'll find it in 'Tools > Projection > Export coordinate(s) to scalar field(s)'
	* very useful to display heights with a color scale or to segment out points depending on their height
- New shortcut: 'Z' to zoom and center camera on selected entities (magnifier icon)
- Toggle shortcuts are now applied RECURSIVELY to any kind of object (even a group for instance)
	(reminder: 'N' for normals, 'C' for colors, 'S' for active scalar field and 'V' for visibility)
- Enhancements:
	* primitives don't loose their attributes (colors, SF, etc.) when cloned
		(they still loose them if display precision is changed however...)
	* CC will only display one global progress bar when loading E57 files with many scans (>10)

- Bug fixes:
	* a space character was missing when exporting a cloud with normals in ASCII format!
	* texture files associated to PLY files should be vertically inverted (at least when the files come from PhotoScan?)
	* e57 scan names were ignored on import
	* OBJ (sub)meshes names were ignored on import
	* a selection composed of parents and (some of their) siblings would produce a corrupted BIN file on export
	* in some cases users could freely segment mesh vertices causing a crash
	* in some cases mesh cloning would fail for no (good) reason
	* when loading primitives from a BIN V2 file, a supplementary (empty) 'vertices' cloud was created
	* mesh groups normals are now correctly handled (they can be safely deleted, exported, etc.)

v2.4 12/16/2012
---------------

- Introducing a new alignment/registration method: "Align (point pairs picking)"
	* aligns clouds by interactively picking 'equivalent' points in both clouds (3 pairs at least)
	* replaces the old "Align (auto)" research tool in the main toolbar (same icon)
	* more information on the wiki: http://www.cloudcompare.org/doc/wiki/index.php?title=Alignment_and_Registration
- Perspective mode and lights activation state are now automatically saved and restored (on the creation of a new 3D view or when CC restarts)
- Overlay dialogs (manual segmentation and transformation, point picking, etc.) don't prevent CC from being closed anymore
- 'Height grid generation' tool now gives more choice for the management of empty cells
	+ all types of output (cloud, ASCII grid file, image) behave the same way
- On-screen message display mechanism upgraded
- Meshes without normals are now displayed with the 'diffuse front' material color (this allows one to change the default color of blank meshes)
- New function: "Add constant SF" ("Edit > Scalar Fields" menu)
	* simple way to add new scalar fields (with a constant unique value)
	* very useful in conjunction with "SF Arithmetic" (to add, subtract, multiply or divide another SF by a constant)
- SF Arithmetic ("Edit > Scalar Fields > Arithmetic") has a new icon (a calculator, instead of the 'diff' text with a colorbar)
- PLY files associated with a texture file (comment with 'TextureFile' keyword - generated by PhotoScan for instance) are now supported
- LAS import filter improved:
	* 'time' and 'return number' fields are now supported (i.e. imported as scalar fields)
	* color coding (8 bits or 16 bits) is automatically detected (see associated bug below)
	* importer is more clever (fields with a unique value are automatically ignored - in case you really need them, they can be created
		afterwards with the "Add constant SF" method - see above)
- ASCII export filter:
	* adaptive precision when writing point coordinates (typically if the cloud has been shifted at loading time due to too large coordinates)
	* if the file extension is 'PTS', color components will be written after the scalar field value(s) so as to be readable by Autocad

- Bug fixes:
	* color from LAS point clouds were badly imported/exported (treated as 8 bits integers instead of 16 bits!).
	* 'Height grid generation' tool was generating badly shaped scalar fields that wouldn't be correctly saved in BIN files (corrupted files)
	* the plane primitive generated by the 'plane orientation' tool was not automatically added to DB tree
	* per-triangle normals are removed with 'Edit > Normals > Clear'
	* the mesh sampling method was not sampling normals in specific cases
	* custom light materials were wrongly set (in practical, custom light wasn't working at all!)
	* a bug in the display mechanism of meshes with textures (coordinates) could make CC crash... randomly
	* segmenting a cloud with labels would make CC crash (for the moment labels are simply removed... we need to be more clever!)
	* OS X portage --> lots of associated bugs... almost all solved ;)

v2.4 11/06/2012
---------------

- Bug fixes:
	* plane orientation tool was always returning a wrong plane normal ( N(0,0,1) by default )
	* segmenting a primitive or an entity with associated primitives would make CC crash
		(primitive segmentation is disabled for the moment!)

v2.4 10/31/2012
---------------

- New primitive objects: cone, torus, dish, box, snout and profile extrusion
- New format handled (and the first 'CAD' one): Aveva PDMS '.mac' scripts
	(support for all above primitives + already existing ones: cylinder, planes and spheres)
- Height grid generation (=rasterization) upgraded:
	* source cloud's scalar field(s) can now be projected as well (+ various options to interpolate the SF values in each cell)
	* main projection dimension can be set (X, Y or Z)
- Points sampling on a mesh upgraded:
	* source mesh per-triangle or per-vertices normals are now taken into account
	* user can choose to interpolate colors and/or textures
- New filters available in qPCL plugin (thanks to Luca):
	* remove outliers using statistical approach (SIFT keypoints)
	* smooth (and optionally upsample) a point cloud using MLS estimators
- New button added in the graphical segmentation tool: "confirm and delete hidden points".
	It's a shortcut: hidden points are automatically be deleted and the original cloud is not split in two.
- New entries in the DB tree context menu: "Expand branch" and "Collapse branch" (to fully expand/collapse a branch)

- Bug fixes:
	* the normal index for each face vertex was missing when saving a mesh with per-vertex normals to an OBJ file.
	(this would typically make Meshlab crash ;). Just open them with this new version of CC and overwrite them to fix them.
	* potential crash when using the research option 'enable furthest points removal' during registration (ICP)
    * PLY files saved by CloudCompare have (once again) field names "compatible" with MeshLab.
	The 'vertex_indices' field has changed to 'vertex_indexes' in the last versions of CC ... and this make Meshlab crash...
	(I've already ranted about that before: the PLY format doesn't impose any particular field name and it's the responsibility
	of the loader to handle that... Paraview do this wonderfully for instance ;)

v2.4 09/22/2012
----------------

- Cloud-cloud distance computation with local models enhanced/fixed: the user can now choose either a number of neighbors or a sphere
	radius that will be used to compute local models. One can also choose to compute a model for every points or, as an approximation,
	to "share" local models between neighbors (faster... but noisier - in fact this was done by default before!).
- New tools accessible in command line mode: density, roughness, curvature and SF gradient.
	(see wiki: http://www.cloudcompare.org/doc/wiki/index.php?title=CommandLine)
- When exporting a cloud to a LAS file, any scalar field named 'LAS classification' will now be handled as a proper LAS classification
	field.
- PCV plugin (ShadeVis-like global illumination algorithm) can now take a point cloud with normals as input. The normals will be used
	as lighting directions (instead of the default directions randomly sampled on a hemisphere or a sphere). Note that only the normals
	are taken into account as the lighting direction is simulated in orthographic mode (no perspective, i.e. the relative position of
	the light source has no effect on the result).

v2.4 07/28/2012
----------------

- Shift applied to clouds with too large coordinates can now be applied to all opened files (in case of multiple selection):
	* "Apply all" button added to dialog
	* shift information is stored in BIN files and is now 'reverted' with ASCII, OBJ, MA, VTK, PLY, LAS & E57 formats (i.e. ASCII or 64 bits formats)
- New menu entry "Display > Active scalar field" (this menu is active if a unique cloud or mesh is selected):
	* Toggle color scale: show or hide color scale (shortcut: SHIFT+C)
	* Show previous SF: show previous scalar field (shortcut: SHIFT + up arrow)
	* Show next SF: show next scalar field (shortcut: SHIFT + down arrow)
- First (skipped) line of ASCII files can now be used to read out 'headers' for columns
	* a new checkbox is available at the bottom of the ASCII import wizard.
	* the header line must have as much blocks as columns in the file (so be sure not to use space characters in names if the file delimiter is also the space character!)
	* for the moment, only the scalar fields names are used by the ASCII importer

- Bugs fixed:
	* 2D Viewport objects can now be dragged and dropped, and can be loaded properly from BIN files
	* centered perspective mode could behave strangely when zooming in close to objects with large coordinates
	* render to file based on FBO (frame buffer object) with a zoom different than 1 was done without depth component!
	* distance computation timing displayed in Console was in microseconds instead of seconds!

v2.4 07/18/2012
----------------

- qRansac_SD plugin is back! (Ransac Shape Detection by Schnabel et. al)
- Bilateral gaussian filter for scalar fields added (Thanks Luca)
- Ranges computation from a sensor added (Thanks Luca)
- Scattering angles computation from a sensor added (Thanks Luca)
- Loaded mesh vertices that are not shared by multiple sub-meshes are not 'locked' by default anymore
- No more system console on Windows

- Bugs fixed:
	* under certain circumstances, when two clouds with different scalar fields were fused,
		random values would be set to the points that hadn't any before (instead of NaN).
	* a bug with the internal timer was preventing CC from properly updating mesh bounding-boxes
		(after a transformation typically) or the messages in 3D views from disappearing
	* PLY files with comment lines before the format line (in header) would be rejected by
		the new version of RPLY (RPLY has been patched to accept this again).

v2.4 07/03/2012
---------------

- LAS file import now handles intensity and classification fields
- The 2D part of labels is no longer displayed in 2D in the 'Point list picking' tool (to mimic the old version)
- New type of label: rectangular area labels can be defined on screen to annotate a particular zone
	(they can be created with the Point Picking tool - don't forget to "save" the label once created)
- Test in progress: weighted ICP with associated scalar field(s) values
- Bugs fixed:
	* some algorithms applied on multiple entities at the same time could make CC crash
	* E57 reading crashed since last version!

v2.4 06/27/2012
---------------

- 64 bits version is now compiled with Visual 2010 (should avoid issues with the Visual 2008 redistributable pack on Seven & Vista)
- normals are now supported with E57 format
- shortcut added: 'Pick rotation center' icon added to the left 'view' tool bar (let the user pick a point to be used as rotation center for its hosting 3D view)
- bugs fixed:
	* point list picking now displays proper labels in the 3D view (+ all points are stored in a separate group)
	* Bundler '.out' file containing keypoints without associated color are now correctly handled
	* Bundler v0.4 files are now correctly handled

v2.4 06/24/2012
---------------

- CloudCompare 64 bits version available (Windows & Linux)
- Introducing BIN "V2" format (highly enhanced):
	* almost any type of entity and their display option can be saved (all selected elements with their siblings)
	* this format can freely evolve while keeping backward compatibility (integrated version management)
	* loading time is much faster
	* 32/64 bits compatible
- Point picking now spawns labels
	* labels can simply be created anytime by holding the SHIFT key and clicking on a 3D point
	* labels are independent objects that can be moved on screen, hidden, saved (BIN V2), etc.
	* labels can be collapsed with a right click
	* point list picking and point/segment/triplet picking dialogs have been updated accordingly
- ccViewer upgraded:
	* ccViewer also supports labels (loading and creating with SHIFT+click)
	* selected entities can now be deleted with the DEL key (or Menu > Options > Selected entity > Delete)
- New format supported: E57 point clouds (ASTM E2807 standard) thanks to libE57 (http://www.libe57.org)
- New plugin: 'qPCL' a wrapper to the PCL library (http://pointclouds.org).
	For the moment, it only allows for loading and saving PCD files and computing normals on a point cloud.
	But many filters will follow! (thanks to Luca Penasa)
- New method: conversion from cloud normals to HSV colors (Edit > Normals > Convert to HSV)
- Scalar fields boundaries can now be set by user (check the 'release boundaries' checkbox in the scalar field properties and manually set the desired values in the min and max 'displayed' values spinboxes)
- Distance computation tool can now split the result along each dimension (X,Y and Z) in the case of cloud-cloud distance
	(very useful to get only the vertical displacement for instance)
- DB tree elements now have a context menu (right click) to enable/disable standard features (visibility, color, normals, scalar field) or delete the selected element(s), etc.
- Entities in the DB tree can now be moved manually (with drag & drop) or sorted alphabetically (via the context menu)
- New (empty) groups can be added in the DB tree (via the context menu)
- Graphical segmentation tool has been enhanced (rectangular selection with CTRL key + right mouse button pressed, constant feedback, etc.)
- Display properties dialog has been reorganized
- Histogram labels are now using the color scale 'precision' parameter value (see Display > Display Settings)
- Image ortho-rectification enhanced (Bundler import filter):
  * bug fix: the previous version was generating ortho-rectified images with a non constant pixel scale!
  * a 'ortho_rectification_log.txt' file is now generated along the images with relative position for each (in pixels, relatively to the first image)
  * images are now processed sequentially, and the user can choose whether they should be kept in memory or not
  * ortho-rectification as images (on disk) is now available through command line arguments (see wiki: http://www.cloudcompare.org/doc/wiki/index.php?title=CommandLine)
- Multiple clouds can now be automatically saved in multiple ASCII files
	* select several clouds then 'Save' them as ASCII files
	* enter a base name with an extension (hint: any occurrence of the string 'cloudname' will be replaced by each cloud name)
	* CC will generate one file per cloud, with an automatic suffix ('basename_000001.asc', etc.)

v2.4 04/24/2012
---------------

- Bug fix: crash when displaying the histogram after computing statistical parameters
- Bug fix: once loaded, a VTK mesh couldn't be manually transformed and its vertices didn't appear in the DB tree
- Bug fix: when computing distances between two clouds with a 'maximum distance' threshold fixed, this maximum distance value was assigned to points with zero distance!
- Bug fix: when saving PLY files with scalar fields having spaces characters in their name, reloading was not possible (hack: even for binary PLY files, the header is in ASCII mode and can manually be edited --> replace all spaces in the SF name by underscores)
- Bug fix: when checking/unchecking an item in the DB tree, its properties were displayed even if the item was not selected
- Snavely's Bundler file import tool can now generate 2D orthorectified images (directly saved alongside original images)

v2.4 01/04/2012
---------------

- Simple Laplacian smoothing algorithm for mesh added (see Edit > Mesh > Smooth (Laplacian))
- VTK cloud/mesh filter added (import & export - early version!)
- The 'Plane orientation' tool now outputs in the console a matrix that can be used to make the resulting plane horizontal (it can be applied to the input cloud with the new 'Apply transformation' tool)
- qPCV plugin slightly optimized
- qHPR plugin now uses qHull V2012.1
- qPoissonRecon now uses PoissonRecon V3 (http://www.cs.jhu.edu/~misha/Code/PoissonRecon). Works also on Linux now.
- PLY files saved by CloudCompare now use 'properties' names compatible with MeshLab (to cope with the lack of flexibility of this - however great - tool ;)
- All scalar fields associated to a cloud are now saved in PLY files (with their name as property name + prefix 'SCALAR_')
- Bug in Delaunay 2D triangulation on least square best fitting plane fixed.
- Crash during OBJ files loading fixed (the crash occurred when the file was referring to materials but no material file was declared)
- The Windows release has been compiled with Visual Studio 2008. You may have to install Visual C++ 2008 Redistributable Package (x86) to launch CloudCompare.
- Project can/should now be compiled with CMake (www.cmake.org). See http://www.cloudcompare.org/doc/wiki/index.php?title=Compilation_CMake

v2.3 02/14/2012
---------------

- Noah Snavely's Bundler import filter is now able to use an alternative cloud (or mesh) as keypoints source.
	If it's a mesh, it can also be used as DTM for generating colored pseudo-DTM vertices
- Transformation matrix is now properly output in console (either after registration process, or after interactive editing).
	This transformation can be copied & pasted in the new 'Edit > Apply transformation' dialog that replaces 'Edit > Translate'.
- A bug in connected components extraction with octree has been fixed (in rare cases, the process could make CC crash)

v2.3 01/28/2012
---------------

- Height function (quadric) fitting fixed and enhanced
- Noah Snavely's Bundler import filter is now able to ortho-recitfy images, undistort them and generate a colored pseudo-DTM
- (calibrated) images can now be saved as standard images
- Distance computation between a point cloud and a mesh has been optimized (new multi-threaded version + the old version is slightly faster & needs less memory)
- Bug in scalar field arithmetic fixed (difference between two positive scalar field is not necessarily positive!)
- the 'Render to file' tool with a zoom factor (different from 1) now works properly when based on FBO

v2.3 01/18/2012
---------------

- Noah Snavely's Bundler output file (with 'out' extension - see http://phototour.cs.washington.edu/bundler/) can now be opened in CloudCompare.
  CloudCompare will extract the point cloud as well as calibrated cameras information and will then try to load associated pictures.
- This version should fix an issue with distance computation on Windows 7 (crash)

v2.3 01/07/2012
---------------

- new shortcuts added (see wiki)
- points and faces count in properties view are now displayed with thousands separator
- 'plane orientation' tool now creates semi transparent planes (polygon stippling in fact)
- color scale now takes 'text color' parameter into account!
- EDL filter has been slightly updated (isolated points appear no more black)

v2.3 12/13/2011
----------------

- scalar fields loading bug fixed
- color ramp type and number of steps are now per-scalar field and not per-cloud

v2.3 12/11/2011
---------------

- resampling algorithms are now accessible in command line (see the wiki for more information)
- algorithms based on local neighbors extraction (such as roughness, curvature, etc.) can now handle much bigger point clouds (> 20 M. points)
- memory shortage is less likely to make CC crash
- scalar field colors can now be displayed with a logarithmic scale
- precision of color scale displayed values can now be setup with the 'display options' dialog
- bug corrected in ASCII file loading wizard when skipping lines
- various display artifacts or minor bugs removed

v2.3 11/22/2011
---------------

- A bug in CCLib (ReferenceCloud) has been corrected. It may have impacted ICP registration for clouds below 20000 points.
- the code has been cleaned-up with 'cppcheck' (http://cppcheck.sourceforge.net/ - great tool!)
- the multi-core octree-based computations mechanism is back! (seems that it has mistakenly been disabled a while ago... hum hum)
- ASCII file loading changed: from now on, only one cloud can be loaded at a time, but the fields order doesn't matter any more
(a scalar field can be set before points coordinates, etc.) and loading is also slightly faster.
- if available, FBO (Frame Buffer Object) is systematically used to store the last displayed 3D scene. This allows very fast redraw
of the 3D view when viewpoint and cloud properties are not changed (i.e. during interactive graphical segmentation, or anytime the
application window needs to be redrawn when requested by the OS or Qt - when a menu overlaps the 3D view, or the window is moved, etc.).

v2.3 10/16/2011
---------------

- Point cloud fusion tool improved (much less buggy!)
- Light colors and materials: default colors changed + user can now specify default specular material for mesh
- Mesh per-triangle normals are now correctly handled during interactive transformation
- Console automatically raises when a warning message appears

V2.3 09/18/2011
---------------

- Major speed up in point cloud display (2 to 3 times faster!) as well as mesh (up to 30% faster in some cases)
- Major improvement of the point picking mechanism (faster, more robust)
- Excel "CSV" files now automatically recognized as ASCII files
- Minor bug corrections and GUI modifications

V2.3 09/04/2011
---------------

- Color scale improvements:
	- user can set multiple settings (square size, font size, whether to always display '0' or not)
	- user can now choose now for signed scalar fields whether saturation is absolute (former default) or signed (new default)
- Scalar field(s) name(s) are now editable (Edit > Scalar Field > Rename)
- Scalar field(s) name(s) are now saved in BIN files

v2.3 07/07/2011
---------------

- clouds names are now saved in BIN files
- points number was missing in cloud description since 06/22/2011
- qEDL plugin (Eye Dome Lighting) is now less sensitive to zoom extent ONLY IN NON PERSPECTIVE MODE!

v2.3 07/06/2011
---------------

- qEDL plugin (Eye Dome Lighting) is now less sensitive to zoom extent
- Number of randomly sampled points in ICP registration can now be specified by user (default was 20000)
- Bug correction: CC crashed when ICP registration's result was the identity matrix!
- Bug correction: "Edit > Bounding-box > Fit principal components" corrected and warning message displayed when called

v2.3 06/28/2011
----------------

- Handling of mesh materials & textures during segmentation
- Storage of cloud center and restoration on save (ASCII only) when cloud has huge coordinates

v2.3 06/22/2011
---------------

- Method 'Height Grid Generation' corrected
- Method 'Plane orientation' enhanced (strike plane name contains now strike AND dip information)

v2.3 06/18/2011
---------------

- Method 'Plane orientation' enhanced (display of strike plane, etc.)
