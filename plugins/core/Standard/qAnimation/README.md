qAnimation
==========

This plugin is used to turn several views into a directory of images that are suitable for being put into a movie.

To use, create a series of views (ctrl-V). Select the views you want to make into a movie (at least two). In the popup dialog, adjust the times, frame rates and save directories. Press preview to preview the movie. Press Render to save a series of images (save with filenames reflecting the time) into the save directory.

The images can be turned into a movie using a program like ffmpeg.

Notes:
- order of selection of the views is important, since the CloudCompare window doesn't allow for re-ordering of views.
- work in small batches, as their is currently no way to stop a preview or render (see improvements)
- interpolation between views is linear
	
Improvements:
- re-ordering of views inside the pluging GUI.
- progress dialog with a cancel for preview and rendering.
- previewing a subset of the views from within the plugin gui.
- add different interpolation routines for smoother transitions (i.e. cubic spline)
- persistent settings in the plugin gui (currently, all settings reset when re-run)
- alternatively, save a session and reload it
- add views in the plugin dialog
- incorporate libavcodec to directly produce a movie, rather than produce a set of files.
	
