TreeIso (plugin)
============

Individual-tree isolation (treeiso) from terrestrial laser scanning
-----------------------------------------------------------------

Zhouxin Xi, Chris Hopkinson (Department of Geography & Environment, University of Lethbridge, Canada)

TreeIso can be utilized to separate individual trees from terrestrial laser scanning point clouds, assigning tree IDs as supplementary scalar fields.

Please cite the following paper if you find this tool helpful:

Xi, Z.; Hopkinson, C. 3D Graph-Based Individual-Tree Isolation (Treeiso) from Terrestrial Laser Scanning Point Clouds. Remote Sens. 2022, 14, 6116. https://doi.org/10.3390/rs14236116

This tool relies on the cut-pursuit algorithm, please also consider citing:
Landrieu, L.; Obozinski, G. Cut Pursuit: Fast Algorithms to Learn Piecewise Constant Functions on General Weighted Graphs. SIAM J. Imaging Sci. 2017, 10, 1724–1766. hal link
Raguet, H.; Landrieu, L. Cut-pursuit algorithm for regularizing nonsmooth functionals with graph total variation. In Proceedings of the International Conference on Machine Learning, Stockholm, Sweden, 10–15 July 2018; Volume 80, pp. 4247–4256.

The cut-pursuit files are licensed under the **MIT** license - see the [LICENSE-MIT](LICENSE-MIT.txt) file for details.

A Matlab version shared via:
https://github.com/truebelief/artemis_treeiso

The development of the treeiso plugin was inspired by the CSF plugin originating from: 
Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501


Command line mode
-----------------
Command line is supported. The plugin consists of three stages of segmentation. In the command line mode, please configure the segmentation parameters for each stage to activate the corresponding stage. If no parameters are provided for a specific stage, the segmentation for that stage will not be executed.

Available options
-----------------
<table>
	<tr>
		<th>Command</th>
		<th>Description</th>
	</tr>
	<tr>
		<td><code>-TREEISO</code></td>
		<td>
			<i>Runs the TREEISO plugin</i>
			<p>Optional settings are:</p>
			<ul>
				<li> -LAMBDA1 [value]: Regularization strength for initial segmentation (default: 1.0)</li>
				<li> -K1 [value]: Nearest neighbors to search for initial segmentation(default: 5)</li>
				<li> -DECIMATE_RESOLUTION1 [value]: Decimated point resolution (in m) for initial segmentation (default: 0.05)</li>
				<li> -LAMBDA2 [value]: Regularization strength for intermediate segmentation (default: 20)</li>
				<li> -K2 [value]: Nearest neighbors to search for intermediate segmentation (default: 20)</li>
				<li> -MAX_GAP [value]: Maximum point gap (in m) for intermediate segmentation (default: 2.0)</li>
				<li> -DECIMATE_RESOLUTION2 [value]: Decimated point resolution (in m) for intermediate segmentation (default: 0.1)</li>
				<li> -RHO [value]: Relative height to length ratio (used to detect non-stems for final segmentation) (default: 0.5)</li>
				<li> -VERTICAL_OVERLAP_WEIGHT [value]: Vertical overlapping ratio weight for final segmentation (default: 0.5)</li>
			</ul>
		</td>
	</tr>
</table>