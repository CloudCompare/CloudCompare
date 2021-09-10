CloudCompare Coding rules
=======================

## Naming

Names in CloudCompare should be as descriptive as possible, without abbreviations, apart for very clear or common ones (such as `fw` instead of `forward`, etc.). Most of variable names should begin with a lower case letter. If the name is composed of multiple words, the first letter of each word should be in upper case (apart for the first one of course). 

Example: `numberOfPoints`, `ptsCount` (or even `ptsNum` for the laziest ;-)

### Specific cases

- static variables: should always begin with prefix `s_` (in lower case – like `s_defaultFilename`
- static methods: should always begin with a upper case letter (like `InitGLEW`)
- classes: should always begin with prefix `cc` (in lower case – like `ccConsole`)  
- enumerators:
  - all letters in upper case
  - should always begin with prefix `CC_`
  - words are separated by underscore (like `CC_OBJECT_FLAG`)
- macros: begin with prefix `MACRO_` followed by a standard method name (like `MACRO_SkipUnselected`)
- const variables:
  - all letters in upper case
  - words are separated by underscore (like `NORMALS_QUANTIZE_LEVEL`)
- `macro const` (`#define`): should be avoided; same syntax as const

### Files

- File naming follows the same rule as most CloudCompare elements (first letter in lower case, etc.)
- Each class should be saved alone in a header + source file couple. Exceptionally, very small classes that are used by a single class may be saved along with this class. The header + source filename should be the same as the main class. 

Example: `ccConsole` saved in `ccConsole.h` and `ccConsole.cpp`

- Filenames shouldn’t contain any space character. Use underscore instead.
- All data-related classes (data models, database, etc) should be saved in `db` directory.
- Images (icons) should all be saved in the `images` directory (or one of its subdirectories).
- GUI templates (mainly `.ui` Qt files) should be saved in the `ui_templates` directory 

### Tabs and indentation

- Indentation is expected to be made in `Tabs` only, each of size `4`.

### Summary
Element | Example
------- | -------
Class | `ccMyClass`
File  |  `ccMyClass.h` and `ccMyClass.cpp`
Attribute/variable | `myAttribute`
Static attribute/variable | `s_myAttribute`
Method | `getMethod()`
Static method | `GetMethod()`
Structure | `myStruct`
Enumerator | `CC_MY_ENUMERATOR`
Macro | `MACRO_myMethod`
Const variables | `MY_CONSTANT`
Const (`#define`) | `MY_CONSTANT`

## Unix compliance

For avoiding incompatible syntax with Unix environments, the following rules must be respected:
¬	use only "/" for include paths.

Example: `include "../db/ccPointCloud.h"`

## File headers

Any new source file (`.h`, `.cpp`, etc.) integrated to any CloudCompare module (CCLib, qCC, etc.) must present the official header.

Here is the official header for LGPL modules (CCLib, etc.):

```
//##########################################################################
//#                                                                        #
//#                              MODULE NAME                               #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

```

And for official header for GPL modules (qCC, etc.):

```
//##########################################################################
//#                                                                        #
//#                            MODULE NAME                                 #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################
```

Designing a new qCC plugin
==========================

## Introduction

Designing a new plugin is an easy way to extend qCC (CloudCompare) functionalities,
without the pain of having to modify its core and do all the connections.

One can easily design a new function, that may be applied on one or several entities currently loaded in CloudCompare.
Moreover, the plugin can display its own dialog.

Dummy plugin structures are provided to get you started.

## 1. Choose your type

There are 3 types of plugins:
1. Standard: plugins that add actions, processing tools.
2. IO: plugins that adds to CloudCompare the ability to read or write additional file formats.
3. GL: plugins that do things with the OpenGL rendering.

Each type of plugin has a dummy template ready that can be found in the `plugins/example` directory.

Once you know which type of plugin you wish to make, copy the plugin's template
into the directory `plugins/private`. (Create the private folder if it does not exist).

CloudCompare's CMake will scan this `plugins/private` directory to add your plugin to the list of
buildable plugins.

## 2. Renaming

After copying the template the next steps involve renaming the folder, class name, etc
to names of your choice.

Here is a non-exhaustive list of things to be changed:

- Rename the plugin's directory (e.g. from `ExamplePlugin` to `SuperPlugin`)
- Rename the `.qrc` file to have the same name as your plugin's directory (e.g. `ExamplePlugin.qrc` -> `SuperPlugin.qrc`)
- In the top-level `CMakeLists.txt` of your new plugin:
  - Change the name of the option that controls whether your plugin should be built
  - Change the name of the project
- In the "main" `.cpp` and `.h` file (e.g. the `ExamplePlugin.cpp` and `ExamplePlugin.h`) change the class name
- Rename the `.cpp` and `.h` files and don't forget to update the different `CMakeList.txt`.
- Update the `info.json` file.
- If your plugin relies on additional libraries, you can add them to the `CMakeList.txt` 
  See for instance the equivalent files for the qHPR or qPCV plugins.

This list may miss some elements that you should remove, searching/greping for `dummy` should show you the
things left.

Some guidance about what you should do is given in the form of comments
inside the files of the template you started from.

Don't forget to add the correct option to your CMake configuration to make sure you plugin gets built.

## Resources
You can now begin with the real work: implementing the plugin action.

All algorithms (in CCLib) and 3D entities (in CCLib, qCC_db, qCC_io and qCC_gl) are accessible inside the plugin. Check
the doxygen documentation of those projects for more information.

- [CCLib doxygen documentation](http://www.cloudcompare.org/doc/CCLib/html/index.html)
- [qCC doxygen documentation](http://www.cloudcompare.org/doc/qCC/html/index.html)  
  Once again, the other plugin projects are a good source of hints, as the CloudCompare project itself.
