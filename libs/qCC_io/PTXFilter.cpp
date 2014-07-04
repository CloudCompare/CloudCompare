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

#include "PTXFilter.h"
#include "ccCoordinatesShiftManager.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccColorScalesManager.h>
#include <ccProgressDialog.h>

//CCLib
#include <Neighbourhood.h>
#include <GenericIndexedMesh.h>

//Qt
#include <QFile>
#include <QTextStream>

//System
#include <assert.h>
#include <string.h>

const char CC_PTX_INTENSITY_FIELD_NAME[] = "Intensity";

CC_FILE_ERROR PTXFilter::saveToFile(ccHObject* entity, QString filename)
{
	//not supported
	return CC_FERR_WRONG_FILE_TYPE;
}

CC_FILE_ERROR PTXFilter::loadFile(	QString filename,
									ccHObject& container,
									bool alwaysDisplayLoadDialog/*=true*/,
									bool* coordinatesShiftEnabled/*=0*/,
									CCVector3d* coordinatesShift/*=0*/)
{
	//open ASCII file for reading
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		return CC_FERR_READING;

	QTextStream inFile(&file);

	QString line;
	QStringList tokens;
	bool ok;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	ScalarType minIntensity = 0;
	ScalarType maxIntensity = 0;
	while (result == CC_FERR_NO_ERROR)
	{
		unsigned width = 0, height = 0;
		ccGLMatrix sensorTrans, cloudTrans;

		//read header
		{
			line = inFile.readLine();
			if (line.isNull() && container.getChildrenNumber() != 0) //end of file?
				break;
			height = line.toUInt(&ok);
			if (!ok)
				return CC_FERR_MALFORMED_FILE;
			line = inFile.readLine();
			width = line.toUInt(&ok);
			if (!ok)
				return CC_FERR_MALFORMED_FILE;

			ccLog::Print(QString("[PTX] Grid size: %1 x %2").arg(width).arg(height));

			//read sensor transformation matrix
			//DGM: TODO what can we do of this? Create a ccSensor?
			for (int i=0; i<4; ++i)
			{
				line = inFile.readLine();
				tokens = line.split(" ",QString::SkipEmptyParts);
				if (tokens.size() != 3)
					return CC_FERR_MALFORMED_FILE;

				for (int j=0; j<3; ++j)
				{
					float* colDest = 0;
					if (i == 0)
						//Translation
						colDest = sensorTrans.getTranslation();
					else
						//X, Y and Z axis
						colDest = sensorTrans.getColumn(i-1);

					assert(colDest);
					colDest[j] = tokens[j].toFloat(&ok);
					if (!ok)
						return CC_FERR_MALFORMED_FILE;
				}
			}

			//read cloud transformation matrix
			for (int i=0; i<4; ++i)
			{
				line = inFile.readLine();
				tokens = line.split(" ",QString::SkipEmptyParts);
				if (tokens.size() != 4)
					return CC_FERR_MALFORMED_FILE;

				float* col = cloudTrans.getColumn(i);
				for (int j=0; j<4; ++j)
				{
					col[j] = tokens[j].toFloat(&ok);
					if (!ok)
						return CC_FERR_MALFORMED_FILE;
				}
			}
		}

		//now we can read the grid cells
		unsigned size = width * height;
		ccPointCloud* cloud = new ccPointCloud();
		if (container.getChildrenNumber() == 0)
		{
			cloud->setName("unnamed - Cloud");
		}
		else
		{
			if (container.getChildrenNumber() == 1)
				container.getChild(0)->setName("unnamed - Cloud 1"); //update previous cloud name!

			cloud->setName(QString("unnamed - Cloud %1").arg(container.getChildrenNumber()+1));
		}

		if (!cloud->reserve(size))
		{
			delete cloud;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		//intensities
		ccScalarField* intensitySF = new ccScalarField(CC_PTX_INTENSITY_FIELD_NAME);
		if (!intensitySF->reserve(static_cast<unsigned>(size)))
		{
			ccLog::Warning("[PTX] Not enough memory to load intensities!");
			intensitySF->release();
			intensitySF = 0;
		}

		//grid for computing normals
		unsigned* indexGrid = new unsigned[size];
		if (indexGrid)
		{
			memset(indexGrid,0,sizeof(unsigned)*size);
		}
		else
		{
			ccLog::Warning("[PTX] Not enough memory to save grid structure (requied to compute normals!)");
		}

		//read points
		{
			//progress dialog
			ccProgressDialog pdlg(true);
			CCLib::NormalizedProgress nprogress(&pdlg,size);
			pdlg.setMethodTitle("Loading PTX file");
			pdlg.setInfo(qPrintable(QString("Number of cells: %1").arg(size)));
			pdlg.start();

			CCVector3d Pshift(0,0,0);
			bool firstPoint = true;
			bool hasColors = false;
			bool loadColors = false;

			for (unsigned j=0; j<height; ++j)
			{
				for (unsigned i=0; i<width; ++i)
				{
					line = inFile.readLine();
					tokens = line.split(" ",QString::SkipEmptyParts);

					if (firstPoint)
					{
						hasColors = (tokens.size() == 7);
						if (hasColors)
						{
							loadColors = cloud->reserveTheRGBTable();
							if (!loadColors)
								ccLog::Warning("[PTX] Not enough memory to load RGB colors!");
						}
					}
					if ((hasColors && tokens.size() != 7) || (!hasColors && tokens.size() != 4))
					{
						result = CC_FERR_MALFORMED_FILE;
						//early stop
						j = height;
						break;
					}

					double values[4];
					for (int v=0; v<4; ++v)
					{
						values[v] = tokens[v].toDouble(&ok);
						if (!ok)
						{
							result = CC_FERR_MALFORMED_FILE;
							//early stop
							j = height;
							break;
						}
					}

					//we skip "empty" cells
					if (CCVector3d::fromArray(values).norm2() != 0)
					{
						const double* Pd = values;
						//first point: check for 'big' coordinates
						if (firstPoint)
						{
							bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
							if (shiftAlreadyEnabled)
								Pshift = *coordinatesShift;
							bool applyAll = false;
							if (	sizeof(PointCoordinateType) < 8
								&&	ccCoordinatesShiftManager::Handle(Pd,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,&applyAll))
							{
								cloud->setGlobalShift(Pshift);
								ccLog::Warning("[PTXFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift.x,Pshift.y,Pshift.z);

								//we save coordinates shift information
								if (applyAll && coordinatesShiftEnabled && coordinatesShift)
								{
									*coordinatesShiftEnabled = true;
									*coordinatesShift = Pshift;
								}
							}
							firstPoint = false;
						}

						//add point
						cloud->addPoint(CCVector3(	static_cast<PointCoordinateType>(Pd[0] + Pshift.x),
													static_cast<PointCoordinateType>(Pd[1] + Pshift.y),
													static_cast<PointCoordinateType>(Pd[2] + Pshift.z)) );

						//update index grid
						if (indexGrid)
							indexGrid[j*width + i] = cloud->size(); // = index + 1 (as default value = 0)

						//add intensity
						if (intensitySF)
							intensitySF->addElement(static_cast<ScalarType>(values[3]));

						//color
						if (loadColors)
						{
							colorType rgb[3];
							for (int c=0; c<3; ++c)
							{
								unsigned temp = tokens[4+c].toUInt(&ok);
								ok &= (temp <= static_cast<unsigned>(MAX_COLOR_COMP));
								if (ok)
								{
									rgb[c] = static_cast<colorType>(temp);
								}
								else
								{
									result = CC_FERR_MALFORMED_FILE;
									//early stop
									j = height;
									break;
								}
							}
							cloud->addRGBColor(rgb);
						}
					}

					if (!nprogress.oneStep())
					{
						result = CC_FERR_CANCELED_BY_USER;
						break;
					}
				}
			}
		}

		if (cloud->size() == 0)
		{
			delete cloud;
			if (intensitySF)
				intensitySF->release();
			if (result == CC_FERR_NO_ERROR)
				result = CC_FERR_NO_LOAD;
		}
		else if (cloud->size() < cloud->capacity())
		{
			cloud->resize(cloud->size());
			if (intensitySF)
			{
				assert(intensitySF->currentSize() == cloud->size());
				intensitySF->resize(cloud->size());
				intensitySF->computeMinAndMax();
				intensitySF->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::ABS_NORM_GREY));
				int intensitySFIndex = cloud->addScalarField(intensitySF);

				if (container.getChildrenNumber() == 0)
				{
					minIntensity = intensitySF->getMin();
					maxIntensity = intensitySF->getMax();
				}
				else
				{
					minIntensity = std::min(minIntensity,intensitySF->getMin());
					maxIntensity = std::max(maxIntensity,intensitySF->getMax());
				}

				cloud->showSF(true);
				cloud->setCurrentDisplayedScalarField(intensitySFIndex);
			}

			//try to compute normals
			if (indexGrid)
			{
				if (cloud->reserveTheNormsTable())
				{
					//progress dialog
					ccProgressDialog pdlg(true);
					CCLib::NormalizedProgress nprogress(&pdlg,cloud->size());
					pdlg.setMethodTitle("Compute PTX normals");
					pdlg.setInfo(qPrintable(QString("Number of points: %1").arg(cloud->size())));
					pdlg.start();

					const unsigned* _indexGrid = indexGrid;
					CCLib::ReferenceCloud knn(cloud);
					const int nw = 3;
					knn.reserve((1+2*nw)*(1+2*nw));
					bool canceledByUser = false;
					for (int j=0; j<static_cast<int>(height); ++j)
					{
						for (int i=0; i<static_cast<int>(width); ++i, ++_indexGrid)
						{
							if (*_indexGrid != 0)
							{
								//add the point itself
								knn.clear(false);
								knn.addPointIndex(*_indexGrid-1); //warning: indexes are shifted (0 = no point)
								const CCVector3* P = cloud->getPoint(*_indexGrid-1);
							
								//look for neighbors
								for (int v=std::max(0,j-nw); v<std::min<int>(height,j+nw); ++v)
								{
									if (v != j)
									{
										for (int u=std::max(0,i-nw); u<std::min<int>(width,i+nw); ++u)
										{
											if (u != i)
											{
												unsigned indexN = indexGrid[v*width + u];
												if (indexN != 0)
												{
													//we don't consider points with a too much different depth than the central point
													const CCVector3* Pn = cloud->getPoint(indexN-1);
													if (fabs(Pn->z - P->z) <= std::max(fabs(Pn->x - P->x),fabs(Pn->y - P->y)))
														knn.addPointIndex(indexN-1); //warning: indexes are shifted (0 = no point)
												}
											}
										}
									}
								}

								CCVector3 N(0,0,1);
								if (knn.size() >= 3)
								{
									CCLib::Neighbourhood Z(&knn);

									//compute normal with quadratic func. (if we have enough points)
									if (knn.size() >= 6)
									{
										uchar hfDims[3];
										const PointCoordinateType* h = Z.getHeightFunction(hfDims);
										if (h)
										{
											const CCVector3* gv = Z.getGravityCenter();
											assert(gv);

											const uchar& iX = hfDims[0];
											const uchar& iY = hfDims[1];
											const uchar& iZ = hfDims[2];

											PointCoordinateType lX = P->u[iX] - gv->u[iX];
											PointCoordinateType lY = P->u[iY] - gv->u[iY];

											N.u[iX] = h[1] + (2 * h[3] * lX) + (h[4] * lY);
											N.u[iY] = h[2] + (2 * h[5] * lY) + (h[4] * lX);
											N.u[iZ] = -1.0;

											N.normalize();
										}
									}
									//else
									//{
									//	//compute normal with best fit plane
									//	const CCVector3* _N = Z.getLSQPlaneNormal();
									//	if (_N)
									//		N = *_N;
									//}
									else
									{
										//compute normals with 2D1/2 triangulation
										CCLib::GenericIndexedMesh* theMesh = Z.triangulateOnPlane();
										if (theMesh)
										{
											unsigned faceCount = theMesh->size();
											N.z = 0;

											//for all triangles
											theMesh->placeIteratorAtBegining();
											for (unsigned j=0; j<faceCount; ++j)
											{
												const CCLib::TriangleSummitsIndexes* tsi = theMesh->getNextTriangleIndexes();
												//we look if the central point is one of the triangle's vertices
												if (tsi->i1 == 0 || tsi->i2 == 0|| tsi->i3 == 0)
												{
													const CCVector3 *A = knn.getPoint(tsi->i1);
													const CCVector3 *B = knn.getPoint(tsi->i2);
													const CCVector3 *C = knn.getPoint(tsi->i3);

													CCVector3 no = (*B - *A).cross(*C - *A);
													//no.normalize();
													N += no;
												}
											}

											delete theMesh;
											theMesh = 0;

											//normalize the 'mean' vector
											N.normalize();
										}
									}
								}

								//check normal vector sign
								CCVector3 viewVector = *P /*- cloudTrans.getTranslationAsVec3D()*/; //clouds are still in their local coordinate system!
								if (viewVector.dot(N) > 0)
									N *= -1;
								cloud->addNorm(N);

								//progress
								if (!nprogress.oneStep())
								{
									canceledByUser = true;
									//early stop
									j = height;
									break;
								}
							}
						}
					}

					if (canceledByUser)
					{
						ccLog::Warning("[PTX] Normal computation canceled by user!");
						cloud->unallocateNorms();
					}
				}
				else
				{
					ccLog::Warning("[PTX] Not enough memory to compute normals!");
				}
			}

			//test: export as depth map
			//if (true)
			//{
			//	_indexGrid = indexGrid;
			//	for (int j=0; j<static_cast<int>(height); ++j)
			//	{
			//		for (int i=0; i<static_cast<int>(width); ++i, ++_indexGrid)
			//		{
			//			if (*_indexGrid != 0)
			//			{
			//				const CCVector3* P = cloud->getPoint(*_indexGrid-1);
			//				const_cast<CCVector3*>(P)->x = static_cast<PointCoordinateType>(j);
			//				const_cast<CCVector3*>(P)->y = static_cast<PointCoordinateType>(i);
			//			}
			//		}
			//	}
			//}

			//DGM TODO: should we apply the transformation or is it already done?
			cloud->applyGLTransformation_recursive(&cloudTrans);

			cloud->setVisible(true);
			cloud->showColors(cloud->hasColors());
			cloud->showNormals(cloud->hasNormals());

			container.addChild(cloud);
		}

		if (indexGrid)
		{
			delete[] indexGrid;
			indexGrid = 0;
		}
	}

	//update scalar fields saturation (globally!)
	{
		for (unsigned i=0; i<container.getChildrenNumber(); ++i)
		{
			ccHObject* obj = container.getChild(i);
			assert(obj && obj->isA(CC_TYPES::POINT_CLOUD));
			CCLib::ScalarField* sf = static_cast<ccPointCloud*>(obj)->getScalarField(0);
			if (sf)
			{
				static_cast<ccScalarField*>(sf)->setSaturationStart(minIntensity);
				static_cast<ccScalarField*>(sf)->setSaturationStop(maxIntensity);
			}
		}
	}

	return result;
}
