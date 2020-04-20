//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#ifndef CC_CHUNK_HEADER
#define CC_CHUNK_HEADER

//System
#include <vector>

//! Fake chunked array management
class ccChunk
{
public:
	static const size_t SIZE_POWER = 16;
	static const size_t SIZE = (1 << SIZE_POWER); //~ 64K

	inline static size_t Count(size_t elementCount) { return (elementCount >> SIZE_POWER) + ((elementCount & (SIZE - 1)) ? 1 : 0); }
	inline static size_t Size(size_t chunkIndex, size_t elementCount) { return (chunkIndex + 1 < Count(elementCount) ? SIZE : elementCount - chunkIndex * SIZE); }
	inline static size_t Size(size_t chunkIndex, size_t chunkCount, size_t elementCount) { return (chunkIndex + 1 < chunkCount ? SIZE : elementCount - chunkIndex * SIZE); }
	inline static size_t StartPos(size_t chunkIndex) { return chunkIndex * SIZE; }
	template<typename T> inline static T* Start(std::vector<T>& buffer, size_t chunkIndex) { return buffer.data() + StartPos(chunkIndex); }
	template<typename T> inline static const T* Start(const std::vector<T>& buffer, size_t chunkIndex) { return buffer.data() + StartPos(chunkIndex); }
	template<typename T> inline static size_t Count(const std::vector<T>& buffer) { return Count(buffer.size()); }
	template<typename T> inline static size_t Size(size_t chunkIndex, const std::vector<T>& buffer) { return Size(chunkIndex, buffer.size()); }
};

#endif //CC_CHUNK_HEADER
