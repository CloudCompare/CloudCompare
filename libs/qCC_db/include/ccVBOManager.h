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

// Local
#include "ccGLDrawContext.h"
#include "ccGenericGLDisplay.h"
#include "ccScalarField.h"

// Qt
#include <QOpenGLBuffer>

// System
#include <cstdint>
#include <vector>

class ccPointCloud;

// DGM: normals are so slow to display that it's a waste of memory and time to load them in VBOs!
#define DONT_LOAD_NORMALS_IN_VBOS

class ccVBO : public QOpenGLBuffer
{
  public: // methods
	ccVBO()
	    : QOpenGLBuffer(QOpenGLBuffer::VertexBuffer)
	    , rgbShift(0)
	    , normalShift(0)
	    , pointCount(0)
	{
	}

	//! Inits the VBO
	/** \return the number of allocated bytes (or -1 if an error occurred)
	 **/
	int init(int count, bool withColors, bool withNormals, bool* reallocated = nullptr);

  public: // members
	int      rgbShift;
	int      normalShift;
	uint32_t pointCount;
};

//! VBO Manager
class ccAbstractVBOManager
{
  public: // enums
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

	static bool CatchGLErrors(GLenum err, const char* context)
	{
		// catch GL errors
		{
			// see http://www.opengl.org/sdk/docs/man/xhtml/glGetError.xml
			switch (err)
			{
			case GL_NO_ERROR:
				return false;
			case GL_INVALID_ENUM:
				ccLog::Warning("[%s] OpenGL error: invalid enumerator", context);
				break;
			case GL_INVALID_VALUE:
				ccLog::Warning("[%s] OpenGL error: invalid value", context);
				break;
			case GL_INVALID_OPERATION:
				ccLog::Warning("[%s] OpenGL error: invalid operation", context);
				break;
			case GL_STACK_OVERFLOW:
				ccLog::Warning("[%s] OpenGL error: stack overflow", context);
				break;
			case GL_STACK_UNDERFLOW:
				ccLog::Warning("[%s] OpenGL error: stack underflow", context);
				break;
			case GL_OUT_OF_MEMORY:
				ccLog::Warning("[%s] OpenGL error: out of memory", context);
				break;
			case GL_INVALID_FRAMEBUFFER_OPERATION:
				ccLog::Warning("[%s] OpenGL error: invalid framebuffer operation", context);
				break;
			}
		}

		return true;
	}

  public: // methods
	ccAbstractVBOManager()
	    : hasColors(false)
	    , colorIsSF(false)
	    , sourceSF(nullptr)
	    , hasNormals(false)
	    , totalMemSizeBytes(0)
	    , updateFlags(0)
	    , managerState(NEW)
	{
	}

	virtual void releaseVBOs(const ccGenericGLDisplay* currentDisplay) = 0;

	void resetFlags()
	{
		hasColors         = false;
		hasNormals        = false;
		colorIsSF         = false;
		sourceSF          = nullptr;
		totalMemSizeBytes = 0;
		managerState      = ccAbstractVBOManager::NEW;
	}

	virtual bool updateVBOs(const ccPointCloud& pc, const ccGenericGLDisplay* currentDisplay, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) = 0;

	//! Dependency on point cloud is needed by the "Regular" VBO Manager (ccPointCloudVBOManager) to handle normals
	virtual bool renderVBOs(const ccPointCloud& pc, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) = 0;

  public: // members
	bool           hasColors;
	bool           colorIsSF;
	ccScalarField* sourceSF;
	bool           hasNormals;
	size_t         totalMemSizeBytes;
	int            updateFlags;

	//! Current state
	STATES managerState;
};

class ccPointCloudVBOManager : public ccAbstractVBOManager
{
  public: // methods
	void releaseVBOs(const ccGenericGLDisplay* currentDisplay) override;

	bool updateVBOs(const ccPointCloud& pc, const ccGenericGLDisplay* currentDisplay, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override;

	bool renderVBOs(const ccPointCloud& pc, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override;

  protected: // members
	std::vector<ccVBO*> m_vbos;
};

using ccGLDrawContext = CC_DRAW_CONTEXT;
