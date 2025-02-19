#pragma once
// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// Qt
#include <QOpenGLBuffer>

// System
#include <vector>

class ccPointCloud;
class ccScalarField;

class ccVBO : public QOpenGLBuffer
{
  public:
	int rgbShift;
	int normalShift;

	//! Inits the VBO
	/** \return the number of allocated bytes (or -1 if an error occurred)
	 **/
	int init(int count, bool withColors, bool withNormals, bool* reallocated = nullptr);

	ccVBO()
	    : QOpenGLBuffer(QOpenGLBuffer::VertexBuffer)
	    , rgbShift(0)
	    , normalShift(0)
	{
	}
};

//! VBO set
class ccVBOManager
{
  public:
	//! States of the VBO(s)
	enum STATES
	{
		NEW,
		INITIALIZED,
		FAILED
	};

	//! Update flags
	enum UPDATE_FLAGS
	{
		UPDATE_POINTS  = 1,
		UPDATE_COLORS  = 2,
		UPDATE_NORMALS = 4,
		UPDATE_ALL     = UPDATE_POINTS | UPDATE_COLORS | UPDATE_NORMALS
	};

	ccVBOManager()
	    : hasColors(false)
	    , colorIsSF(false)
	    , sourceSF(nullptr)
	    , hasNormals(false)
	    , totalMemSizeBytes(0)
	    , updateFlags(0)
	    , managerState(NEW)
	{
	}

	std::vector<ccVBO*> vbos;
	bool                hasColors;
	bool                colorIsSF;
	ccScalarField*      sourceSF;
	bool                hasNormals;
	size_t              totalMemSizeBytes;
	int                 updateFlags;

	//! Current state
	STATES managerState;
};
