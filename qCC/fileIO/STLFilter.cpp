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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2276                                                              $
//$LastChangedDate:: 2012-10-18 14:58:26 +0200 (jeu., 18 oct. 2012)        $
//**************************************************************************
//
#include "STLFilter.h"
#include "../ccCoordinatesShiftManager.h"

//Qt
#include <QApplication>
#include <QFileInfo>
#include <QStringList>
#include <QString>

//qCC_db
#include <ccMesh.h>
#include <ccMeshGroup.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccNormalVectors.h>
#include <ccOctree.h>

#include "../ccConsole.h"

CC_FILE_ERROR STLFilter::saveToFile(ccHObject* entity, const char* filename)
{
	if (!entity)
		return CC_FERR_BAD_ARGUMENT;

	if (!entity->isKindOf(CC_MESH))
		return CC_FERR_BAD_ENTITY_TYPE;

	ccGenericMesh* mesh = static_cast<ccGenericMesh*>(entity);
	if (mesh->size()==0)
	{
		ccConsole::Warning(QString("[ObjFilter] No facet in mesh '%1'!").arg(mesh->getName()));
		return CC_FERR_NO_ERROR;
	}

	//try to open file for saving
	FILE* theFile = fopen(filename,"wb");
	if (!theFile)
		return CC_FERR_WRITING;

	CC_FILE_ERROR result = saveToFile(mesh, theFile);

	fclose(theFile);

	return result;
}

CC_FILE_ERROR STLFilter::saveToFile(ccGenericMesh* mesh, FILE *theFile)
{
	assert(theFile && mesh && mesh->size()!=0);
	unsigned numberOfTriangles = mesh->size();
	
	//progress
	ccProgressDialog pdlg(true);
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfTriangles);
	pdlg.setMethodTitle(qPrintable(QString("Saving mesh [%1]").arg(mesh->getName())));
	pdlg.setInfo(qPrintable(QString("Number of facets: %1").arg(numberOfTriangles)));
	pdlg.start();

	if (fprintf(theFile,"solid %s\n",qPrintable(mesh->getName())) < 0) //empty names are acceptable!
		return CC_FERR_WRITING;

	//vertices
	ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
	unsigned nbPoints = vertices->size();
	const double* shift = vertices->getOriginalShift();

	mesh->placeIteratorAtBegining();
	for (unsigned i=0;i<numberOfTriangles;++i)
	{
		CCLib::TriangleSummitsIndexes*tsi = mesh->getNextTriangleIndexes();

		const CCVector3* A = vertices->getPointPersistentPtr(tsi->i1);
		const CCVector3* B = vertices->getPointPersistentPtr(tsi->i2);
		const CCVector3* C = vertices->getPointPersistentPtr(tsi->i3);
		//compute face normal (right hand rule)
		CCVector3 N = (*B-*A).cross(*C-*A);

		if (fprintf(theFile,"facet normal %e %e %e\n",N.x,N.y,N.z) < 0) //scientific notation
			return CC_FERR_WRITING;
		if (fprintf(theFile,"outer loop\n") < 0)
			return CC_FERR_WRITING;
		if (fprintf(theFile,"vertex %e %e %e\n",-shift[0]+(double)A->x,-shift[1]+(double)A->y,-shift[2]+(double)A->z) < 0) //scientific notation
			return CC_FERR_WRITING;
		if (fprintf(theFile,"vertex %e %e %e\n",-shift[0]+(double)B->x,-shift[1]+(double)B->y,-shift[2]+(double)B->z) < 0) //scientific notation
			return CC_FERR_WRITING;
		if (fprintf(theFile,"vertex %e %e %e\n",-shift[0]+(double)C->x,-shift[1]+(double)C->y,-shift[2]+(double)C->z) < 0) //scientific notation
			return CC_FERR_WRITING;
		if (fprintf(theFile,"endloop\nendfacet\n") < 0)
			return CC_FERR_WRITING;
	}

	if (fprintf(theFile,"endsolid %s\n",qPrintable(mesh->getName())) < 0) //empty names are acceptable!
		return CC_FERR_WRITING;

	return CC_FERR_NO_ERROR;
}

const double c_defaultSearchRadius = sqrt(ZERO_TOLERANCE);
bool tagDuplicatedVertices(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters)
{
	GenericChunkedArray<1,int>* equivalentIndexes = (GenericChunkedArray<1,int>*)additionalParameters[0];

	//we look for points very near to the others (only if not yet tagged!)
	
	//structure for nearest neighbors search
	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.truncatedCellCode					= cell.truncatedCode;
	nNSS.prepare(c_defaultSearchRadius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);
	//*/

	unsigned n = cell.points->size(); //number of points in the current cell
	
	//we already know some of the neighbours: the points in the current cell!
	try
	{
		nNSS.pointsInNeighbourhood.resize(n);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	//init structure with cell points
	{
		CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned i=0;i<n;++i,++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;
	}

	//for each point in the cell
	for (unsigned i=0;i<n;++i)
	{
		int thisIndex = (int)cell.points->getPointGlobalIndex(i);
		if (equivalentIndexes->getValue(thisIndex)<0) //has no equivalent yet 
		{
			cell.points->getPoint(i,nNSS.queryPoint);

			//look for neighbors in a (very small) sphere
			unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,c_defaultSearchRadius,false);

			//if there are some very close points
			if (k>1)
			{
				for (unsigned j=0;j<k;++j)
				{
					//all the other points are equivalent to the query point
					const unsigned& otherIndex = nNSS.pointsInNeighbourhood[j].pointIndex;
					if (otherIndex != thisIndex)
						equivalentIndexes->setValue(otherIndex,thisIndex);
				}
			}

			//and the query point is always root
			equivalentIndexes->setValue(thisIndex,thisIndex);
		}
	}

	return true;
}

CC_FILE_ERROR STLFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	ccConsole::Print("[STLFilter::Load] %s",filename);

	//ouverture du fichier
	QFile fp(filename);
	if (!fp.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	//ASCII OR BINARY?
	QString name("mesh");
	bool ascii = true;
	{
		//buffer
		char header[80]={0};
		qint64 sz = fp.read(header,80);
		if (sz<80)
		{
			//either ASCII or BINARY STL FILES are always > 80 bytes
			return sz == 0 ? CC_FERR_READING : CC_FERR_MALFORMED_FILE;
		}
		//normally, binary files shouldn't start by 'solid'
		if (!QString(header).trimmed().toUpper().startsWith("SOLID"))
			ascii = false;
		else //... but sadly some BINARY files does start by SOLID?!!!! (wtf)
		{
			//go back to the begining of the file
			fp.seek(0);
			char line[MAX_ASCII_FILE_LINE_LENGTH];
			//skip first line
			fp.readLine(line,MAX_ASCII_FILE_LINE_LENGTH); //should be ok as we already have read some data
			//we look if the second line (if any) starts by 'facet'
			if (fp.readLine(line,MAX_ASCII_FILE_LINE_LENGTH) < 0)
				ascii = false;
			else
				if (!QString(line).trimmed().toUpper().startsWith("FACET"))
					ascii = false;
		}
		//go back to the begining of the file
		fp.seek(0);
	}
	ccLog::Print("[STL] Detected format: %s",ascii ? "ASCII" : "BINARY");

	//vertices
	ccPointCloud* vertices = new ccPointCloud("vertices");
	//mesh
	ccMesh* mesh = new ccMesh(vertices);
	mesh->setName(name);
	//add normals
	mesh->setTriNormsTable(new NormsIndexesTableType());

	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	if (ascii)
		error = loadASCIIFile(fp,mesh,vertices,alwaysDisplayLoadDialog,coordinatesShiftEnabled,coordinatesShift);
	else
		error = loadBinaryFile(fp,mesh,vertices,alwaysDisplayLoadDialog,coordinatesShiftEnabled,coordinatesShift);

	if (error != CC_FERR_NO_ERROR)
	{
		delete vertices;
		delete mesh;
		return CC_FERR_MALFORMED_FILE;
	}

	unsigned vertCount = vertices->size();
	unsigned faceCount = mesh->size();
	ccConsole::Print("[STLFilter::Load] %i points, %i face(s)",vertCount,faceCount);

	//remove duplicated vertices
	{
		GenericChunkedArray<1,int>* equivalentIndexes = new GenericChunkedArray<1,int>;
		const int razValue = -1;
		if (equivalentIndexes->resize(vertCount,true,razValue))
		{
			ccOctree* octree = vertices->computeOctree();
			if (octree)
			{
				ccProgressDialog progressDlg(true);
				void* additionalParameters[1] = {(void*)equivalentIndexes};
				unsigned result = octree->executeFunctionForAllCellsAtLevel(ccOctree::MAX_OCTREE_LEVEL,tagDuplicatedVertices,additionalParameters,&progressDlg,"Tag duplicated vertices");
				vertices->deleteOctree();
				octree=0;

				if (result>0)
				{
					unsigned remainingCount = 0;
					for (unsigned i=0;i<vertCount;++i)
					{
						int eqIndex = equivalentIndexes->getValue(i);
						assert(eqIndex >= 0);
						if (eqIndex==(int)i) //root point
						{
							int newIndex = (int)(vertCount+remainingCount); //We replace the root index by its 'new' index (+ vertCount, to differentiate it later)
							equivalentIndexes->setValue(i,newIndex);
							++remainingCount;
						}
					}

					ccPointCloud* newVertices = new ccPointCloud("vertices");
					if (newVertices->reserve(remainingCount))
					{
						//copy root points in a new cloud
						{
							for (unsigned i=0;i<vertCount;++i)
							{
								int eqIndex = equivalentIndexes->getValue(i);
								if (eqIndex>=(int)vertCount) //root point
									newVertices->addPoint(*vertices->getPoint(i));
								else
									equivalentIndexes->setValue(i,equivalentIndexes->getValue(eqIndex)); //and update the other indexes
							}
						}

						//update face indexes
						{
							for (unsigned i=0;i<faceCount;++i)
							{
								CCLib::TriangleSummitsIndexes* tri = mesh->getTriangleIndexes(i);
								tri->i1 = (unsigned)equivalentIndexes->getValue(tri->i1)-vertCount;
								tri->i2 = (unsigned)equivalentIndexes->getValue(tri->i2)-vertCount;
								tri->i3 = (unsigned)equivalentIndexes->getValue(tri->i3)-vertCount;
							}
						}
						
						mesh->setAssociatedCloud(newVertices);
						delete vertices;
						vertices = newVertices;
						vertCount = vertices->size();
	
						ccConsole::Print("[STLFilter::Load] Remaining vertices after auto-removal of duplicate ones: %i",vertCount);
					}
					else
					{
						ccLog::Warning("[STL] Not enough memory: couldn't removed duplicated vertices!");
					}
				}
				else
				{
					ccLog::Warning("[STL] Duplicated vertices removal algorithm failed?!");
				}
			}
			else
			{
				ccLog::Warning("[STL] Not enough memory: couldn't removed duplicated vertices!");
			}
		}
		else
		{
			ccLog::Warning("[STL] Not enough memory: couldn't removed duplicated vertices!");
		}

		if (equivalentIndexes)
			equivalentIndexes->release();
		equivalentIndexes=0;
	}

	NormsIndexesTableType* normals = mesh->getTriNormsTable();
	if (normals)
	{
		normals->link();
		mesh->addChild(normals);
		mesh->showNormals(true);
	}
	else
	{
		if (mesh->computeNormals())
			mesh->showNormals(true);
		else
			ccLog::Warning("[STL] Failed to compute per-vertex normals...");
	}
	vertices->setEnabled(false);
	vertices->setLocked(false); //DGM: no need to lock it as it is only used by one mesh!
	mesh->addChild(vertices);

	container.addChild(mesh);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR STLFilter::loadASCIIFile(QFile& fp,
									ccMesh* mesh,
									ccPointCloud* vertices,
									bool alwaysDisplayLoadDialog,
									bool* coordinatesShiftEnabled/*=0*/,
									double* coordinatesShift/*=0*/)
{
	assert(fp.isOpen() && mesh && vertices);

	//buffer
	char currentLine[MAX_ASCII_FILE_LINE_LENGTH];

	//1st line: 'solid name'
	QString name("mesh");
	{
		if (fp.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH) < 0)
		{
			return CC_FERR_READING;
		}
		QStringList tokens = QString(currentLine).split(QRegExp("\\s+"),QString::SkipEmptyParts);
		if (tokens.empty() || tokens[0].toUpper() != "SOLID")
		{
			ccLog::Error("[STL] File should begin by 'solid [name]'!");
			return CC_FERR_MALFORMED_FILE;
		}
		//Extract name
		if (tokens.size()>1)
		{
			tokens.removeAt(0);
			name = tokens.join(" ");
		}
	}
	mesh->setName(name);

	//progress dialog
	ccProgressDialog progressDlg(true);
	progressDlg.setMethodTitle("Loading ASCII STL file");
	progressDlg.setInfo("Loading in progress...");
	progressDlg.setRange(0,0);
	progressDlg.show();
	QApplication::processEvents();

	//current vertex shift
	double Pshift[3]={0.0,0.0,0.0};

	unsigned pointCount = 0;
	unsigned faceCount = 0;
	bool normalWarningAlreadyDisplayed = true;
	NormsIndexesTableType* normals = mesh->getTriNormsTable();

	unsigned lineCount=1;
	while (true)
	{
		CCVector3 N;
		bool normalIsOk=false;

		//1st line of a 'facet': "facet normal ni nj nk" / or 'endsolid' (i.e. end of file)
		{
			if (fp.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH) < 0)
				break;
			++lineCount;

			QStringList tokens = QString(currentLine).split(QRegExp("\\s+"),QString::SkipEmptyParts);
			if (tokens.empty() || tokens[0].toUpper() != "FACET")
			{
				if (tokens[0].toUpper() != "ENDSOLID")
				{
					ccLog::Error("[STL] Error on line #%i: line should start by 'facet'!",lineCount);
					return CC_FERR_MALFORMED_FILE;
				}
				break;
			}

			if (normals && tokens.size()>=5)
			{
				//let's try to read normal
				if (tokens[1].toUpper() == "NORMAL")
				{
					N.x = tokens[2].toDouble(&normalIsOk);
					if (normalIsOk)
					{
						N.y = tokens[3].toDouble(&normalIsOk);
						if (normalIsOk)
							N.z = tokens[4].toDouble(&normalIsOk);
					}
					if (!normalIsOk && !normalWarningAlreadyDisplayed)
					{
						ccLog::Warning("[STL] Error on line #%i: failed to read 'normal' values!",lineCount);
						normalWarningAlreadyDisplayed=true;
					}
				}
				else if (!normalWarningAlreadyDisplayed)
				{
					ccLog::Warning("[STL] Error on line #%i: expecting 'normal' after 'facet'!",lineCount);
					normalWarningAlreadyDisplayed=true;
				}
			}
			else if (tokens.size() > 1 && !normalWarningAlreadyDisplayed)
			{
				ccLog::Warning("[STL] Error on line #%i: incomplete 'normal' description!",lineCount);
				normalWarningAlreadyDisplayed=true;
			}
		}

		//2nd line: 'outer loop'
		{
			if (fp.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH) <= 0 || !QString(currentLine).trimmed().toUpper().startsWith("OUTER LOOP"))
			{
				ccLog::Error("[STL] Error: expecting 'outer loop' on line #%i",lineCount+1);
				return CC_FERR_MALFORMED_FILE;
			}
			++lineCount;
		}

		//3rd to 5th lines: 'vertex vix viy viz'
		unsigned vertIndexes[3];
		unsigned pointCountBefore = pointCount;
		for (unsigned i=0;i<3;++i)
		{
			if (fp.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH) <= 0 || !QString(currentLine).trimmed().toUpper().startsWith("VERTEX"))
			{
				ccLog::Error("[STL] Error: expecting a line starting by 'vertex' on line #%i",lineCount+1);
				return CC_FERR_MALFORMED_FILE;
			}
			++lineCount;

			QStringList tokens = QString(currentLine).split(QRegExp("\\s+"),QString::SkipEmptyParts);
			if (tokens.size()<4)
			{
				ccLog::Error("[STL] Error on line #%i: incomplete 'vertex' description!",lineCount);
				return CC_FERR_MALFORMED_FILE;
			}

			//read vertex
			double Pd[3];
			{
				bool vertexIsOk=false;
				Pd[0] = tokens[1].toDouble(&vertexIsOk);
				if (vertexIsOk)
				{
					Pd[1] = tokens[2].toDouble(&vertexIsOk);
					if (vertexIsOk)
						Pd[2] = tokens[3].toDouble(&vertexIsOk);
				}
				if (!vertexIsOk)
				{
					ccLog::Error("[STL] Error on line #%i: failed to read 'vertex' coordinates!",lineCount);
					return CC_FERR_MALFORMED_FILE;
				}
			}

			//first point: check for 'big' coordinates
			if (pointCount==0)
			{
				bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
				if (shiftAlreadyEnabled)
					memcpy(Pshift,coordinatesShift,sizeof(double)*3);
				bool applyAll=false;
				if (ccCoordinatesShiftManager::Handle(Pd,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,applyAll))
				{
					vertices->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
					ccConsole::Warning("[STLFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift[0],Pshift[1],Pshift[2]);

					//we save coordinates shift information
					if (applyAll && coordinatesShiftEnabled && coordinatesShift)
					{
						*coordinatesShiftEnabled = true;
						coordinatesShift[0] = Pshift[0];
						coordinatesShift[1] = Pshift[1];
						coordinatesShift[2] = Pshift[2];
					}
				}
			}

			CCVector3 P((PointCoordinateType)(Pd[0]+Pshift[0]),
						(PointCoordinateType)(Pd[1]+Pshift[1]),
						(PointCoordinateType)(Pd[2]+Pshift[2]));

			//look for existing vertices at the same place! (STL format is so dumb...)
			{
				//int equivalentIndex = -1;
				//if (pointCount>2)
				//{
				//	//brute force!
				//	for (int j=(int)pointCountBefore-1; j>=0; j--)
				//	{
				//		const CCVector3* Pj = vertices->getPoint(j);
				//		if (Pj->x == P.x &&
				//			Pj->y == P.y &&
				//			Pj->z == P.z)
				//		{
				//			equivalentIndex = j;
				//			break;
				//		}
				//	}
				//}

				////new point ?
				//if (equivalentIndex < 0)
				{
					//cloud is already full?
					if (vertices->capacity() == pointCount && !vertices->reserve(pointCount+1000))
						return CC_FERR_NOT_ENOUGH_MEMORY;

					//insert new point
					vertIndexes[i] = pointCount++;
					vertices->addPoint(P);
				}
				//else
				//{
				//	vertIndexes[i] = (unsigned)equivalentIndex;
				//}
			}
		}

		//we have successfully read the 3 vertices
		//let's add a new triangle
		{
			//mesh is full?
			if (mesh->maxSize() == faceCount)
			{
				if(!mesh->reserve(faceCount+1000))
					return CC_FERR_NOT_ENOUGH_MEMORY;

				if (normals)
				{
					bool success = normals->reserve(mesh->maxSize());
					if (success && faceCount==0) //specific case: allocate per triangle normal indexes the first time!
						success = mesh->reservePerTriangleNormalIndexes();

					if (!success)
					{
						ccLog::Warning("[STL] Not enough memory: can't store normals!");
						mesh->removePerTriangleNormalIndexes();
						mesh->setTriNormsTable(0);
						normals=0;
					}
				}
			}

			mesh->addTriangle(vertIndexes[0],vertIndexes[1],vertIndexes[2]);
			++faceCount;
		}

		//and a new normal?
		if (normals)
		{
			int index = -1;
			if (normalIsOk)
			{
				//compress normal
				index = (int)normals->currentSize();
				normsType nIndex = ccNormalVectors::GetNormIndex(N.u);
				normals->addElement(nIndex);
			}
			mesh->addTriangleNormalIndexes(index,index,index);
		}

		//6th line: 'endloop'
		{
			if (fp.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH) <= 0 || !QString(currentLine).trimmed().toUpper().startsWith("ENDLOOP"))
			{
				ccLog::Error("[STL] Error: expecting 'endnloop' on line #%i",lineCount+1);
				return CC_FERR_MALFORMED_FILE;
			}
			++lineCount;
		}

		//7th and last line: 'endfacet'
		{
			if (fp.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH) <= 0 || !QString(currentLine).trimmed().toUpper().startsWith("ENDFACET"))
			{
				ccLog::Error("[STL] Error: expecting 'endfacet' on line #%i",lineCount+1);
				return CC_FERR_MALFORMED_FILE;
			}
			++lineCount;
		}

		//progress
		if ((faceCount % 1024) == 0)
		{
			if (progressDlg.isCancelRequested())
				break;
			progressDlg.update(faceCount>>10);
		}
	}

	if (normalWarningAlreadyDisplayed)
		ccLog::Warning("[STL] Failed to read some 'normal' values!");

	progressDlg.stop();

	//do some cleaning
	if (vertices->size() < vertices->capacity())
		vertices->resize(vertices->size());
	if (mesh->size() < mesh->maxSize())
		mesh->resize(mesh->size());
	if (normals && normals->currentSize() < normals->capacity())
		normals->resize(normals->capacity());

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR STLFilter::loadBinaryFile(QFile& fp,
									ccMesh* mesh,
									ccPointCloud* vertices,
									bool alwaysDisplayLoadDialog,
									bool* coordinatesShiftEnabled/*=0*/,
									double* coordinatesShift/*=0*/)
{
	assert(fp.isOpen() && mesh && vertices);

	unsigned pointCount = 0;
	unsigned faceCount = 0;

	//UINT8[80] – Header (we skip it)
	fp.seek(80);
	mesh->setName("Mesh"); //hard to guess solid name with binary files!

	//UINT32 – Number of triangles
	{
		__int32 tmpInt32;
		if (fp.read((char*)&tmpInt32,4)<4)
			return CC_FERR_READING;
		faceCount = tmpInt32;
	}

	if (!mesh->reserve(faceCount))
		return CC_FERR_NOT_ENOUGH_MEMORY;
	NormsIndexesTableType* normals = mesh->getTriNormsTable();
	if (normals && (!normals->reserve(faceCount) || !mesh->reservePerTriangleNormalIndexes()))
	{
		ccLog::Warning("[STL] Not enough memory: can't store normals!");
		mesh->removePerTriangleNormalIndexes();
		mesh->setTriNormsTable(0);
	}

	//progress dialog
	ccProgressDialog progressDlg(true);
	CCLib::NormalizedProgress nProgress(&progressDlg,faceCount);
	progressDlg.setMethodTitle("Loading binary STL file");
	progressDlg.setInfo(qPrintable(QString("Loading %1 faces").arg(faceCount)));
	progressDlg.start();
	QApplication::processEvents();

	//current vertex shift
	double Pshift[3]={0.0,0.0,0.0};

	for (unsigned f=0;f<faceCount;++f)
	{
		//REAL32[3] – Normal vector
		assert(sizeof(float)==4);
		CCVector3 N;
		if (fp.read((char*)N.u,12)<12)
			return CC_FERR_READING;

		//3 vertices
		unsigned vertIndexes[3];
		unsigned pointCountBefore=pointCount;
		for (unsigned i=0;i<3;++i)
		{
			//REAL32[3] – Vertex 1,2 & 3
			float Pf[3];
			if (fp.read((char*)Pf,12)<0)
				return CC_FERR_READING;

			//first point: check for 'big' coordinates
			double Pd[3]={Pf[0],Pf[1],Pf[2]};
			if (pointCount==0)
			{
				bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
				if (shiftAlreadyEnabled)
					memcpy(Pshift,coordinatesShift,sizeof(double)*3);
				bool applyAll=false;
				if (ccCoordinatesShiftManager::Handle(Pd,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,applyAll))
				{
					vertices->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
					ccConsole::Warning("[STLFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift[0],Pshift[1],Pshift[2]);

					//we save coordinates shift information
					if (applyAll && coordinatesShiftEnabled && coordinatesShift)
					{
						*coordinatesShiftEnabled = true;
						coordinatesShift[0] = Pshift[0];
						coordinatesShift[1] = Pshift[1];
						coordinatesShift[2] = Pshift[2];
					}
				}
			}

			CCVector3 P((PointCoordinateType)(Pd[0]+Pshift[0]),
						(PointCoordinateType)(Pd[1]+Pshift[1]),
						(PointCoordinateType)(Pd[2]+Pshift[2]));

			//look for existing vertices at the same place! (STL format is so dumb...)
			{
				//int equivalentIndex = -1;
				//if (pointCount>2)
				//{
				//	//brute force!
				//	for (int j=(int)pointCountBefore-1; j>=0; j--)
				//	{
				//		const CCVector3* Pj = vertices->getPoint(j);
				//		if (Pj->x == P.x &&
				//			Pj->y == P.y &&
				//			Pj->z == P.z)
				//		{
				//			equivalentIndex = j;
				//			break;
				//		}
				//	}
				//}

				////new point ?
				//if (equivalentIndex < 0)
				{
					//cloud is already full?
					if (vertices->capacity() == pointCount && !vertices->reserve(pointCount+1000))
						return CC_FERR_NOT_ENOUGH_MEMORY;

					//insert new point
					vertIndexes[i] = pointCount++;
					vertices->addPoint(P);
				}
				//else
				//{
				//	vertIndexes[i] = (unsigned)equivalentIndex;
				//}
			}
		}

		//UINT16 – Attribute byte count (not used)
		{
			char a[2];
			if (fp.read(a,2)<0)
				return CC_FERR_READING;
		}

		//we have successfully read the 3 vertices
		//let's add a new triangle
		{
			mesh->addTriangle(vertIndexes[0],vertIndexes[1],vertIndexes[2]);
		}

		//and a new normal?
		if (normals)
		{
			//compress normal
			int index = (int)normals->currentSize();
			normsType nIndex = ccNormalVectors::GetNormIndex(N.u);
			normals->addElement(nIndex);
			mesh->addTriangleNormalIndexes(index,index,index);
		}

		//progress
		if (!nProgress.oneStep())
			break;
	}

	progressDlg.stop();

	//do some cleaning
	if (vertices->size() < vertices->capacity())
		vertices->resize(vertices->size());

	return CC_FERR_NO_ERROR;
}

