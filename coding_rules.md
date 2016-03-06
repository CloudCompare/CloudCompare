CloudCompare Coding rules
=======================

## Naming

Names in CloudCompare should be as descriptive as possible, without abbreviations, apart for very clear or common ones (such as ```fw``` instead of ```forward```, etc.). Most of variable names should begin with a lower case letter. If the name is composed of multiple words, the first letter of each word should be in upper case (apart for the first one of course). 

Example: ```numberOfPoints```, ```ptsCount``` (or even ```ptsNum``` for the laziest ;-)

### Specific cases

- static variables: should always begin with prefix ```s_``` (in lower case – like ```s_defaultFilename```
- static methods: should always begin with a upper case letter (like ```InitGLEW```)
- classes: should always begin with prefix ```cc``` (in lower case – like ```ccConsole```)  
- enumerators: all letters in upper case + should always begin with prefix ```CC_``` + words are separated by underscore (like ```CC_OBJECT_FLAG```)
- macros: begin with prefix ```MACRO_``` followed by a standard method name (like ```MACRO_SkipUnselected```)
- const (apart for ```const char*```) : all letters in upper case and words are separated by underscore (like ```NORMALS_QUANTIZE_LEVEL```)
- ```macro const``` (```#define```): same as const

### Files

- File naming follow the same rule as most CloudCompare elements (first letter in lower case, etc.)
- Each class should be saved alone in a header + source file couple. Exceptionally, very small classes that are used by a single class may be saved along with this class. The header + source filename should be the same as the main class. 

Example: ```ccConsole``` saved in ```ccConsole.h``` and ```ccConsole.cpp```

- Filenames shouldn’t contain any space character. Use underscore instead.
- All data-related classes (data models, database, etc) should be saved in ```db``` directory.
- Images (icons) should all be saved in the ```images``` directory (or one of its subdirectory).
- GUI templates (mainly ```.ui``` Qt files) should be saved in ```ui_templates``` directory 

### Summary
Element | Example
------- | -------
Class | ```ccMyClass```
File  |  ```ccMyClass.h``` and ```ccMyClass.cpp```
Attribute/variable | ```myAttribute```
Static attribute/variable | ```s_myAttribute```
Method | ```getMethod()```
Static method | ```GetMethod()```
Structure | ```myStruct```
Enumerator | ```CC_MY_ENUMERATOR```
Macro | ```MACRO_myMethod```
Const | ```MY_CONSTANT```
Const (```#define```) | ```MY_CONSTANT```

## Unix compliance

For avoiding incompatible syntax with Unix environments, the following rules must be respected:
¬	use only “/” for include paths.

Example: #include “../db/ccPointCloud.h”

## File headers

Any new source file (```.h```, ```.cpp```, etc.) integrated to any CloudCompare module (CCLib, qCC, etc.) must present the official header.

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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
```
