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

#include "InputMemoryFile.h"

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#elif defined(_WIN32)
#include <windows.h>
#endif

InputMemoryFile::InputMemoryFile(const char *pathname)
	: data_(0)
	, size_(0)
#if defined(__unix__) || defined(__APPLE__)
	, file_handle_(-1)
{
	file_handle_ = ::open(pathname, O_RDONLY);
	if (file_handle_ == -1)
		return;
	struct stat sbuf;
	if (::fstat(file_handle_, &sbuf) == -1)
		return;
	data_ = static_cast<const char*>(::mmap(0, sbuf.st_size, PROT_READ, MAP_SHARED, file_handle_, 0));
	if (data_ == MAP_FAILED)
		data_ = 0;
	else
		size_ = sbuf.st_size;

#elif defined(_WIN32)
	, file_handle_(INVALID_HANDLE_VALUE)
	, file_mapping_handle_(INVALID_HANDLE_VALUE)
{
	file_handle_ = ::CreateFile(pathname, GENERIC_READ, FILE_SHARE_READ, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (file_handle_ == INVALID_HANDLE_VALUE)
		return;
	file_mapping_handle_ = ::CreateFileMapping(file_handle_, 0, PAGE_READONLY, 0, 0, 0);
	if (file_mapping_handle_ == INVALID_HANDLE_VALUE)
		return;
	data_ = static_cast<char*>(::MapViewOfFile(file_mapping_handle_, FILE_MAP_READ, 0, 0, 0));
	if (data_)
		size_ = ::GetFileSize(file_handle_, 0);
#endif
}

InputMemoryFile::~InputMemoryFile()
{
#if defined(__unix__) || defined(__APPLE__)
	::munmap(const_cast<char*>(data_), size_);
	::close(file_handle_);
#elif defined(_WIN32)
	::UnmapViewOfFile((void*)data_);
	::CloseHandle(file_mapping_handle_);
	::CloseHandle(file_handle_);
#endif
}
