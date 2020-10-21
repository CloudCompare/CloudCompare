Guide: Terra++ Custom Terrain from Lidar data
=============================================

Warning
-------
I do not own the provided qCSF plugin, therefore use it at your discretion. You may use the provide compiled dynamic library file or you may compile it from the source in the plugins folder.

Introduction
------------

This guide is intended for those who wish to use the custom terrain feature for the mod Terra++ for Minecraft and wish to use lidar data as the data source. The following procedure takes quite a while, but this is the nature when working with lidar data. The guide has been tested with CloudCompare v2.11.0 and QGIS v3.14.1.

Short description
-----------------

The following procedure first converts your lidar files to dem, then it imports the generated files, converts them into a suitable format for Terra++ mod and at the end splits them into tiles.

Guide
-----

A. CloudCompare & QGIS preparation
----------------------------------

- First download and install CloudComapre v2.11 from [here](https://www.danielgm.net/cc)
- Donwload and install QGIS from [here]()
- Then navigate to the plugins folder of CloudCompare (default: C:\Program Files\CloudCompare\plugins)
- Download the QCSF_PLUGIN.dll file from the plugins folder on this repository or build it on your own from the source in the plugins folder. More on that [here](https://github.com/CloudCompare/CloudCompare)
- Place the plugin in the plugins folder

B. Generating dem files
-----------------------
- Download and place your lidar files into a folder (ex. Documents\Minecraft\Lidar)
- Open QGIS and navigate to Plugins/Python Console in the toolbar
- In the opened bottom panel click on the "Show Editor" button (Script icon)
- Open the export_dem.py file using the "Open Script..." button (Folder icon)
- Once open navigate to the lidar_directory line (line: 45)
- Here replace the path with your path to the folder and make sure to use \\
- Save the changes using the "Save" button (Floppy disk icon)
- Navigate to View/Panels/Log Messages in the toolbar
- Now, depending on how many lidar files you have this can take quite a while, up to multiple hours. Click on the "Run Script" button (green play button)
- After the script is finished you will get a notification from QGIS
- Move the generated .tif files to a separate folder

C. Importing dem files
----------------------
- Close the previous script and open the loadraster_folder.py file in QUIS Python Console
- Double click on OpenStreetMap in the left panel
- Navigate to the file_directory line (line: 65)
- Here replace the path with your path to the folder where your .tif files are located
- Save the script and run it
- It may lag a bit at the start and at the end but let it run. Again, this can take a while
- The script outputs the progress into the Log Messages panel
- Once It's done QGIS will send you a notification

D. Setting up the project
-------------------------
- Select all your imported layers in the left and right-click on them
- Navigate to Set CRS/Set Layer CRS...
- In the opened window choose the same coordinate reference system your lidar dataset uses (ex. D48_Slovenia_TM)
- Click OK
- In the bottom right corner click the current project coordinate reference system (ex EPSG...)
- In the opened windows search for EPSG:4326 (WGS 84)
- Click on WGS 84 and click OK
- Make sure the layers are in the right location over the map

E. QMetaTiles plugin
--------------------
- Navigate to Plugins/Manage And Install Plugins... in the toolbar
- In the opened window click on All and search for QMetaTiles
- Click on the result and install it
- Close the window

F. Generating tiles
-------------------
- Delete the OpenStreetMap layer
- In the toolbar navigate to Raster/Miscellaneous/Build Virtual Raster
- Set the Resolution to Highest and under Virtual on the right click on Save To File...
- Save the file to a preferred path
- Under Input layers select all your layers and click on run
- Navigate to Plugins/QMetaTiles/QMetaTiles
- Set the Tileset name and choose the Output Directory (Documents\Minecraft\CustomTerrain)
- Set the Layer extent to the virtual layer
- Set the Minimum and Maximum zoom to 17
- Disable Metatiling
- Set the Quality under Parameters to 100
- Select "Make lines appear less jagged at the expense of some drawing performance"
- Click OK
- This will take quite a while depending on how a large area you have
- Once It's finished you can close QGIS

G. Minecraft Terra++
--------------------
- Download and install Terra++ form [here](https://github.com/bitbyte2015/terraplusplus/releases)
- Open Minecraft and create a new world using the usual Build The Earth settings
- Set the Custom Terrain to ON
- In the Custom Terrain Directory textbox, specify the path to your tile, but be sure to use / instead of \ and add a / at the end (ex. C:/Users/david/Documents/Minecraft/CustomTerrain/Flats/)
- Once the world loads navigate to your location using the /tpll command and it should load your custom terrain

qCSF plugin
-----------

The provided plugin is a modified version of the qCSF plugin made to work in the command mode. If you wish to learn more go [here](https://github.com/DavixDevelop/TerraLidar/tree/master/qCSF).

