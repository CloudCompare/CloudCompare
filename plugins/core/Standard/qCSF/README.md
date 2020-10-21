CSF (plugin)
============

Airborne LiDAR Data Filtering Algorithm Based on Cloth Simulation
-----------------------------------------------------------------

Wuming Zhang, Jianbo Qi, Peng Wan, Hongtao Wang (School of Geography, Beijing Normal University, China)


Cloth Simulation Filter (CSF) is a tool to extract of ground points in discrete return LiDAR pointclouds.

If you use this tool for a scientific publication, please cite the following paper:

Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.

Command line mode
-----------------
This plugin is a modified version of the CSF plugin that supports the command line mode. You can find more about it [here](http://www.cloudcompare.org/doc/wiki/index.php?title=CSF_(plugin))

It supports every parameter of the CSF plugin, except the export mesh option.

Available options
-----------------
<table>
	<tr>
		<th>Command</th>
		<th>Description</th>
	</tr>
	<tr>
		<td><code>-CSF</code></td>
		<td>
			<i>Runs the CSF plugin</i>
			<p>Optional settings are:</p>
			<ul>
				<li> -SCENES [scene]: name of the scene (SLOPE|RELIEF|FLAT)</li>
				<li> -PROC_SLOPE: turn on slope post processing for disconnected terrain</li>
				<li> -CLOTH_RESOLUTION [value]: double value of cloth resolution (ex 0.5)</li>
				<li> -MAX_ITERATION [value]: integer value of max iterations (ex. 500)</li>
				<li> CLASS_THRESHOLD [value]: double value of classification threshold (ex. 0.5)</li>
				<li> -EXPORT_GROUND: exports the ground as a .bin file</li>
				<li> -EXPORT_OFFGROUND: exports the off-ground as a .bin file</li>
			</ul>
		</td>
	</tr>
</table>