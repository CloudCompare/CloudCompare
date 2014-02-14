//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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

// Source: http://en.wikibooks.org/wiki/Optimizing_C%2B%2B/General_optimization_techniques/Input/Output

#ifndef INPUT_MEMORY_FILE_HEADER
#define INPUT_MEMORY_FILE_HEADER

//system
#include <cstring> // for size_t

//! Read-only memory-mapped file wrapper.
/** It handles only files that can be wholly loaded
	into the address space of the process.
	The constructor opens the file, the destructor closes it.
	The "data" function returns a pointer to the beginning of the file,
	if the file has been successfully opened, otherwise it returns 0.
	The "size" function returns the length of the file in bytes,
	if the file has been successfully opened, otherwise it returns 0.
**/
class InputMemoryFile
{
public:

	InputMemoryFile(const char *pathname);
	~InputMemoryFile();
	const char* data() const { return data_; }
	size_t size() const { return size_; }

private:

	const char* data_;
	size_t size_;
#if defined(__unix__) || defined(__APPLE__)
	int file_handle_;
#elif defined(_WIN32)
	typedef void* HANDLE;
	HANDLE file_handle_;
	HANDLE file_mapping_handle_;
#else
#error Only Posix or Windows systems can use memory-mapped files.
#endif

};

#endif //INPUT_MEMORY_FILE_HEADER
