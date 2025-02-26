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

#include "ccVBOManager.h"

#include "ccChunk.h"
#include "ccColorTypes.h"
#include "ccLog.h"
#include "ccPointCloud.h"

// CCCore lib
#include <CCTypes.h>

int ccVBO::init(int count, bool withColors, bool withNormals, bool* reallocated /*=nullptr*/)
{
	// required memory
	int totalSizeBytes = sizeof(PointCoordinateType) * count * 3;
	if (withColors)
	{
		rgbShift = totalSizeBytes;
		totalSizeBytes += sizeof(ColorCompType) * count * 4;
	}
	if (withNormals)
	{
		normalShift = totalSizeBytes;
		totalSizeBytes += sizeof(PointCoordinateType) * count * 3;
	}

	if (!isCreated())
	{
		if (!create())
		{
			// no message as it will probably happen on a lot on (old) graphic cards
			return -1;
		}

		setUsagePattern(QOpenGLBuffer::DynamicDraw); //"StaticDraw: The data will be set once and used many times for drawing operations."
		                                             //"DynamicDraw: The data will be modified repeatedly and used many times for drawing operations.
	}

	if (!bind())
	{
		ccLog::Warning("[ccPointCloud::VBO::init] Failed to bind VBO to active context!");
		destroy();
		return -1;
	}

	if (totalSizeBytes != size())
	{
		allocate(totalSizeBytes);
		if (reallocated)
			*reallocated = true;

		if (size() != totalSizeBytes)
		{
			ccLog::Warning("[ccPointCloud::VBO::init] Not enough (GPU) memory!");
			release();
			destroy();
			return -1;
		}
	}
	else
	{
		// nothing to do
	}

	release();

	return totalSizeBytes;
}

void ccPointCloudVBOManager::releaseVBOs(const ccGenericGLDisplay* currentDisplay)
{
	if (managerState == ccAbstractVBOManager::NEW)
		return;

	if (currentDisplay)
	{
		//'destroy' all vbos
		for (size_t i = 0; i < m_vbos.size(); ++i)
		{
			if (m_vbos[i])
			{
				m_vbos[i]->destroy();
				delete m_vbos[i];
				m_vbos[i] = nullptr;
			}
		}
	}
	else
	{
		assert(m_vbos.empty());
	}
	resetFlags();
}

bool ccPointCloudVBOManager::updateVBOs(const ccPointCloud& pc, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams)
{
	if (!pc.m_currentDisplay)
	{
		ccLog::Warning(QString("[ccPointCloud::updateVBOs] Need an associated GL context! (cloud '%1')").arg(pc.getName()));
		assert(false);
		return false;
	}

	if (pc.isColorOverridden())
	{
		// nothing to do (we don't display true colors, SF or normals!)
		return false;
	}

	if (managerState == ccAbstractVBOManager::FAILED)
	{
		// ccLog::Warning(QString("[ccPointCloud::updateVBOs] VBOs are in a 'failed' state... we won't try to update them! (cloud '%1')").arg(getName()));
		return false;
	}

	if (managerState == ccAbstractVBOManager::INITIALIZED)
	{
		// let's check if something has changed
		if (glParams.showColors && (!hasColors || colorIsSF))
		{
			updateFlags |= ccAbstractVBOManager::UPDATE_COLORS;
		}

		if (glParams.showSF
		    && (!hasColors
		        || !colorIsSF
		        || sourceSF != pc.m_currentDisplayedScalarField
		        || pc.m_currentDisplayedScalarField->getModificationFlag() == true))
		{
			updateFlags |= ccAbstractVBOManager::UPDATE_COLORS;
		}

#ifndef DONT_LOAD_NORMALS_IN_VBOS
		if (glParams.showNorms && !hasNormals)
		{
			updateFlags |= UPDATE_NORMALS;
		}
#endif
		// nothing to do?
		if (updateFlags == 0)
		{
			return true;
		}
	}
	else
	{
		updateFlags = ccAbstractVBOManager::UPDATE_ALL;
	}

	using ccChunk = ccPointCloud::ccChunk;
	size_t chunksCount = ccChunk::Count(pc.m_points);

	// allocate per-chunk descriptors if necessary
	if (m_vbos.size() != chunksCount)
	{
		// properly remove the elements that are not needed anymore!
		for (size_t i = chunksCount; i < m_vbos.size(); ++i)
		{
			if (m_vbos[i])
			{
				m_vbos[i]->destroy();
				delete m_vbos[i];
				m_vbos[i] = nullptr;
			}
		}

		// resize the container
		try
		{
			m_vbos.resize(chunksCount, nullptr);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning(QString("[ccPointCloud::updateVBOs] Not enough memory! (cloud '%1')").arg(pc.getName()));
			managerState = ccAbstractVBOManager::FAILED;
			return false;
		}
	}

	// init VBOs
	unsigned pointsInVBOs         = 0;
	size_t   totalSizeBytesBefore = totalMemSizeBytes;
	totalMemSizeBytes             = 0;
	{
		// DGM: the context should be already active as this method should only be called from 'drawMeOnly'
		assert(!glParams.showSF || pc.m_currentDisplayedScalarField);
		assert(!glParams.showColors || pc.m_rgbaColors);
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		assert(!glParams.showNorms || (m_normals && m_normals->chunksCount() >= chunksCount));
#endif

		hasColors = glParams.showSF || glParams.showColors;
		colorIsSF = glParams.showSF;
		sourceSF  = glParams.showSF ? pc.m_currentDisplayedScalarField : nullptr;
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		hasNormals = glParams.showNorms;
#else
		hasNormals = false;
#endif
		QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);
		// process each chunk
		for (size_t chunkIndex = 0; chunkIndex < chunksCount; ++chunkIndex)
		{
			int chunkSize = static_cast<int>(ccChunk::Size(chunkIndex, pc.m_points));

			int  chunkUpdateFlags = updateFlags;
			bool reallocated      = false;

			if (!m_vbos[chunkIndex])
			{
				m_vbos[chunkIndex] = new ccVBO;
			}

			ccVBO* currentVBO = m_vbos[chunkIndex];

			// allocate memory for current VBO
			int vboSizeBytes = currentVBO->init(chunkSize, hasColors, hasNormals, &reallocated);

			CatchGLErrors(glFunc->glGetError(), "ccPointCloud::vbo.init");

			if (vboSizeBytes > 0)
			{
				if (reallocated)
				{
					// if the vbo is reallocated, then all its content has been cleared!
					chunkUpdateFlags = ccAbstractVBOManager::UPDATE_ALL;
				}

				currentVBO->bind();

				// load points
				if (chunkUpdateFlags & ccAbstractVBOManager::UPDATE_POINTS)
				{
					currentVBO->write(0, ccChunk::Start(pc.m_points, chunkIndex), sizeof(PointCoordinateType) * chunkSize * 3);
				}
				// load colors
				if (chunkUpdateFlags & ccAbstractVBOManager::UPDATE_COLORS)
				{
					if (glParams.showSF)
					{
						// copy SF colors in static array
						ColorCompType* _sfColors = ccPointCloud::GetColorsBuffer4u();
						if (sourceSF)
						{
							size_t chunkStart = ccChunk::StartPos(chunkIndex);
							for (int j = 0; j < chunkSize; j++)
							{
								// SF value
								ScalarType sfValue = sourceSF->getValue(chunkStart + j);
								// we need to convert scalar value to color into a temporary structure
								const ccColor::Rgb* col = sourceSF->getColor(sfValue);
								if (!col)
								{
									col = &ccColor::lightGreyRGB;
								}
								*_sfColors++ = col->r;
								*_sfColors++ = col->g;
								*_sfColors++ = col->b;
								*_sfColors++ = ccColor::MAX;
							}
						}
						else
						{
							assert(false);
							for (int j = 0; j < chunkSize; j++)
							{
								// we need to convert scalar value to color into a temporary structure
								*_sfColors++ = ccColor::lightGreyRGB.r;
								*_sfColors++ = ccColor::lightGreyRGB.g;
								*_sfColors++ = ccColor::lightGreyRGB.b;
								*_sfColors++ = ccColor::MAX;
							}
						}
						// then send them in VRAM
						currentVBO->write(currentVBO->rgbShift, ccPointCloud::GetColorsBuffer4u(), sizeof(ColorCompType) * chunkSize * 4);
						// upadte 'modification' flag for current displayed SF
						sourceSF->setModificationFlag(false);
					}
					else if (glParams.showColors)
					{
						currentVBO->write(currentVBO->rgbShift, ccChunk::Start(*pc.m_rgbaColors, chunkIndex), sizeof(ColorCompType) * chunkSize * 4);
					}
				}
#ifndef DONT_LOAD_NORMALS_IN_VBOS
				// load normals
				if (glParams.showNorms && (chunkUpdateFlags & UPDATE_NORMALS))
				{
					// we must decode the normals first!
					CompressedNormType*  inNorms  = pc->m_normals->chunkStartPtr(chunkIndex);
					PointCoordinateType* outNorms = s_normalBuffer;
					for (int j = 0; j < chunkSize; ++j)
					{
						const CCVector3& N = ccNormalVectors::GetNormal(*inNorms++);
						*(outNorms)++      = N.x;
						*(outNorms)++      = N.y;
						*(outNorms)++      = N.z;
					}
					currentVBO->write(currentVBO->normalShift, s_normalBuffer, sizeof(PointCoordinateType) * chunkSize * 3);
				}
#endif
				currentVBO->pointCount = chunkSize;
				currentVBO->release();

				// if an error is detected
				if (CatchGLErrors(glFunc->glGetError(), "ccPointCloud::updateVBOs"))
				{
					vboSizeBytes = -1;
				}
				else
				{
					totalMemSizeBytes += static_cast<size_t>(vboSizeBytes);
					pointsInVBOs += chunkSize;
				}
			}

			if (vboSizeBytes < 0) // VBO initialization failed
			{
				currentVBO->destroy();
				delete currentVBO;
				currentVBO = nullptr;

				// we can stop here
				if (chunkIndex == 0)
				{
					ccLog::Warning(QString("[ccPointCloud::updateVBOs] Failed to initialize VBOs (not enough memory?) (cloud '%1')").arg(pc.getName()));
					managerState = ccAbstractVBOManager::FAILED;
					m_vbos.resize(0);
					return false;
				}
				else
				{
					// shouldn't be better for the next VBOs!
					break;
				}
			}
		}
	}
#ifdef _DEBUG
	if (totalMemSizeBytes != totalSizeBytesBefore)
		ccLog::Print(QString("[VBO] VBO(s) (re)initialized for cloud '%1' (%2 Mb = %3% of points could be loaded)")
		                 .arg(pc.getName())
		                 .arg(static_cast<double>(totalMemSizeBytes) / (1 << 20), 0, 'f', 2)
		                 .arg(static_cast<double>(pointsInVBOs) / pc.size() * 100.0, 0, 'f', 2));
#endif

	managerState = ccAbstractVBOManager::INITIALIZED;
	updateFlags  = 0;

	return true;
}

bool ccPointCloudVBOManager::renderVBOs(const ccPointCloud& pc, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams)
{
	using ccChunk = ccPointCloud::ccChunk;

	if (managerState == ccAbstractVBOManager::FAILED)
	{
		return false;
	}

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);
	for (size_t i = 0; i < m_vbos.size(); ++i)
	{
		const auto vbo = m_vbos[i];
		if (vbo && vbo->isCreated() && vbo->bind())
		{
			const GLbyte* start = nullptr; // fake pointer used to prevent warnings on Linux
			glFunc->glVertexPointer(3, GL_FLOAT, 3 * sizeof(PointCoordinateType), start);

			if (glParams.showNorms)
			{
#ifndef DONT_LOAD_NORMALS_IN_VBOS
				int normalDataShift = vbo->normalShift;
				glFunc->glNormalPointer(GL_FLOAT, 3 * sizeof(PointCoordinateType), static_cast<const GLvoid*>(start + normalDataShift));
#else
				if (pc.m_normals)
				{
					vbo->release(); // Releae VBO for normals
					// we must decode normals in a dedicated static array
					PointCoordinateType*      _normals        = ccPointCloud::GetNormalsBuffer();
					const CompressedNormType* _normalsIndexes = ccChunk::Start(*(pc.m_normals), i);
					size_t                    chunkSize       = ccChunk::Size(i, pc.m_normals->size());

					// compressed normals set
					const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
					assert(compressedNormals);
					for (size_t j = 0; j < chunkSize; j++, _normalsIndexes++)
					{
						const CCVector3& N = compressedNormals->getNormal(*_normalsIndexes);
						*(_normals)++      = N.x;
						*(_normals)++      = N.y;
						*(_normals)++      = N.z;
					}
					glFunc->glNormalPointer(GL_FLOAT, 0, ccPointCloud::GetNormalsBuffer());
					vbo->bind(); // Re-bind VBO
				}
#endif
			}
			if (glParams.showColors || glParams.showSF)
			{
				int colorDataShift = vbo->rgbShift;
				glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, 4 * sizeof(ColorCompType), static_cast<const GLvoid*>(start + colorDataShift));
			}
			vbo->release();
			// we can use VBOs directly
			glFunc->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vbo->pointCount)); // Could be glMultiDrawArrays with shaders...
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			managerState = ccAbstractVBOManager::FAILED;
			return false;
		}
	}
	return true;
}
