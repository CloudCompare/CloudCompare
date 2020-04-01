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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ObjFilter.h"
#include "FileIO.h"

//Qt
#include <QApplication>
#include <QFile>
#include <QFileInfo>
#include <QString>
#include <QStringList>
#include <QTextStream>

//qCC_db
#include <ccChunk.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccMaterial.h>
#include <ccMaterialSet.h>
#include <ccMesh.h>
#include <ccNormalVectors.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccSubMesh.h>

//CCLib
#include <Delaunay2dMesh.h>

//System
#include <cstring>


ObjFilter::ObjFilter()
	: FileIOFilter( {
					"_OBJ Filter",
					8.0f,	// priority
					QStringList{ "obj" },
					"obj",
					QStringList{ "OBJ mesh (*.obj)" },
					QStringList{ "OBJ mesh (*.obj)" },
					Import | Export
					} )
{
}

bool ObjFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::MESH)
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR ObjFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity)
		return CC_FERR_BAD_ARGUMENT;

	if (!entity->isKindOf(CC_TYPES::MESH))
	{
		ccLog::Warning("[OBJ] This filter can only save one mesh (optionally with sub-meshes) at a time!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//mesh
	ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
	if (!mesh || mesh->size() == 0)
	{
		ccLog::Warning("[OBJ] Input mesh is empty!");
		return CC_FERR_NO_SAVE;
	}

	//vertices
	ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
	if (!vertices || vertices->size() == 0)
	{
		ccLog::Warning("[OBJ] Input mesh has no vertices?!");
		return CC_FERR_NO_SAVE;
	}
	unsigned nbPoints = vertices->size();

	//try to open file for saving
	QFile file(filename);
	if (!file.open(QFile::Text | QFile::WriteOnly))
		return CC_FERR_WRITING;

	//progress (start with vertices)
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Saving mesh [%1]").arg(mesh->getName()));
		pDlg->setInfo(QObject::tr("Writing %1 vertices").arg(nbPoints));
		pDlg->setAutoClose(false); //don't close dialogue when progress bar is full
		pDlg->start();
	}
	CCLib::NormalizedProgress nprogress(pDlg.data(), nbPoints);

	QTextStream stream(&file);
	stream.setRealNumberNotation(QTextStream::FixedNotation);
	stream.setRealNumberPrecision(sizeof(PointCoordinateType) == 4 && !vertices->isShifted() ? 8 : 12);

	stream << "# " << FileIO::createdBy() << endl;
	stream << "# " << FileIO::createdDateTime() << endl;
	
	if (file.error() != QFile::NoError)
		return CC_FERR_WRITING;

	for (unsigned i = 0; i < nbPoints; ++i)
	{
		const CCVector3* P = vertices->getPoint(i);
		CCVector3d Pglobal = vertices->toGlobal3d<PointCoordinateType>(*P);
		stream << "v " << Pglobal.x << " " << Pglobal.y << " " << Pglobal.z << endl;
		if (file.error() != QFile::NoError)
			return CC_FERR_WRITING;
		if (pDlg && !nprogress.oneStep()) //update progress bar, check cancel requested
			return CC_FERR_CANCELED_BY_USER;
	}

	//normals
	bool withTriNormals = mesh->hasTriNormals();
	bool withVertNormals = vertices->hasNormals();
	bool withNormals = withTriNormals || withVertNormals;
	if (withNormals)
	{
		//per-triangle normals
		if (withTriNormals)
		{
			NormsIndexesTableType* normsTable = mesh->getTriNormsTable();

			//reset save dialog
			unsigned numTriangleNormals = normsTable->currentSize();
			if (pDlg)
				pDlg->setInfo(QObject::tr("Writing $1 triangle normals").arg(numTriangleNormals));
			nprogress.scale(numTriangleNormals);
			nprogress.reset();

			if (normsTable)
			{
				for (unsigned i = 0; i < numTriangleNormals; ++i)
				{
					const CCVector3& normalVec = ccNormalVectors::GetNormal(normsTable->getValue(i));
					stream << "vn " << normalVec.x << " " << normalVec.y << " " << normalVec.z << endl;
					if (file.error() != QFile::NoError)
						return CC_FERR_WRITING;

					//increment progress bar
					if (pDlg && !nprogress.oneStep()) //cancel requested
						return CC_FERR_CANCELED_BY_USER;
				}
			}
			else
			{
				assert(false);
				withTriNormals = false;
			}
		}
		//per-vertices normals
		else //if (withVertNormals)
		{
			//reset save dialog
			if (pDlg)
				pDlg->setInfo(QObject::tr("Writing %1 vertex normals").arg(nbPoints));
			nprogress.scale(nbPoints);
			nprogress.reset();

			for (unsigned i = 0; i < nbPoints; ++i)
			{
				const CCVector3& normalVec = vertices->getPointNormal(i);
				stream << "vn " << normalVec.x << " " << normalVec.y << " " << normalVec.z << endl;
				if (file.error() != QFile::NoError)
					return CC_FERR_WRITING;

				//increment progress bar
				if (pDlg && !nprogress.oneStep()) //cancel requested
				{
					return CC_FERR_CANCELED_BY_USER;
				}
			}
		}
	}

	//materials
	const ccMaterialSet* materials = mesh->getMaterialSet();
	bool withMaterials = (materials && mesh->hasMaterials());
	if (withMaterials)
	{
		//reset save dialog
		if (pDlg)
			pDlg->setInfo(QObject::tr("Writing %1 materials").arg(materials->size()));
		nprogress.scale(1);
		nprogress.reset();

		//save mtl file
		QStringList errors;
		QString baseName = QFileInfo(filename).baseName();
		if (materials->saveAsMTL(QFileInfo(filename).absolutePath(),baseName,errors))
		{
			stream << "mtllib " << baseName << ".mtl" << endl;
			if (file.error() != QFile::NoError)
				return CC_FERR_WRITING;
		}
		else
		{
			materials = nullptr;
			withMaterials = false;
		}

		//display potential 'errors'
		for (int i=0; i<errors.size(); ++i)
		{
			ccLog::Warning(QString("[OBJ][Material file writer] ")+errors[i]);
		}

		//increment progress bar
		if (pDlg && !nprogress.oneStep()) //cancel requested
			return CC_FERR_CANCELED_BY_USER;
	}

	//save texture coordinates
	bool withTexCoordinates = withMaterials && mesh->hasPerTriangleTexCoordIndexes();
	if (withTexCoordinates)
	{
		TextureCoordsContainer* texCoords = mesh->getTexCoordinatesTable();
		if (texCoords)
		{
			//reset save dialog
			unsigned numTexCoords = texCoords->currentSize();
			if (pDlg)
			{
				pDlg->setInfo(QObject::tr("Writing %1 texture coordinates").arg(numTexCoords));
			}
			nprogress.scale(numTexCoords);
			nprogress.reset();

			for (unsigned i=0; i<texCoords->currentSize(); ++i)
			{
				const TexCoords2D& tc = texCoords->getValue(i);
				stream << "vt " << tc.tx << " " << tc.ty << endl;
				if (file.error() != QFile::NoError)
					return CC_FERR_WRITING;

				//increment progress bar
				if (pDlg && !nprogress.oneStep()) //cancel requested
				{
					return CC_FERR_CANCELED_BY_USER;
				}
			}
		}
		else
		{
			withTexCoordinates = false;
		}
	}

	ccHObject::Container subMeshes;
	//look for sub-meshes
	mesh->filterChildren(subMeshes,false,CC_TYPES::SUB_MESH);
	//check that the number of facets is the same as the full mesh!
	{
		unsigned faceCount = 0;
		for (ccHObject::Container::const_iterator it = subMeshes.begin(); it != subMeshes.end(); ++it)
			faceCount += static_cast<ccSubMesh*>(*it)->size();

		//if there's no face (i.e. no sub-mesh) or less face than the total mesh, we save the full mesh!
		if (faceCount < mesh->size())
		{
			subMeshes.clear();
			subMeshes.push_back(mesh);
		}
	}

	//reset save dialog for triangles
	unsigned numTriangles = mesh->size();
	if (pDlg)
	{
		pDlg->setInfo(QObject::tr("Writing %1 triangles").arg(numTriangles));
		pDlg->setAutoClose(true); //(re-enable) close dialogue when progress bar is full
	}
	nprogress.scale(numTriangles);
	nprogress.reset();

	//mesh or sub-meshes
	unsigned indexShift = 0;
	for (ccHObject::Container::const_iterator it = subMeshes.begin(); it != subMeshes.end(); ++it)
	{
		ccGenericMesh* st = static_cast<ccGenericMesh*>(*it);

		stream << "g " << (st->getName().isNull() ? "mesh" : st->getName()) << endl;
		if (file.error() != QFile::NoError)
			return CC_FERR_WRITING;

		unsigned triNum = st->size();
		st->placeIteratorAtBeginning();

		int lastMtlIndex = -1;
		int t1 = -1;
		int t2 = -1;
		int t3 = -1;

		for (unsigned i=0; i<triNum; ++i)
		{
			if (withMaterials)
			{
				int mtlIndex = mesh->getTriangleMtlIndex(indexShift + i);
				if (mtlIndex != lastMtlIndex)
				{
					if (mtlIndex >= 0 && mtlIndex < static_cast<int>(materials->size()))
					{
						ccMaterial::CShared mat = materials->at(mtlIndex);
						stream << "usemtl " << mat->getName() << endl;
					}
					else
					{
						stream << "usemtl " << endl;
					}
					if (file.error() != QFile::NoError)
						return CC_FERR_WRITING;
					lastMtlIndex = mtlIndex;
				}

				if (withTexCoordinates)
				{
					mesh->getTriangleTexCoordinatesIndexes(indexShift + i, t1, t2, t3);
					if (t1 >= 0) ++t1;
					if (t2 >= 0) ++t2;
					if (t3 >= 0) ++t3;
				}
			}

			const CCLib::VerticesIndexes* tsi = st->getNextTriangleVertIndexes();
			//for per-triangle normals
			unsigned i1 = tsi->i1 + 1;
			unsigned i2 = tsi->i2 + 1;
			unsigned i3 = tsi->i3 + 1;

			stream << "f";
			if (withNormals)
			{
				int n1 = static_cast<int>(i1);
				int n2 = static_cast<int>(i2);
				int n3 = static_cast<int>(i3);
				if (withTriNormals)
				{
					st->getTriangleNormalIndexes(i, n1, n2, n3);
					if (n1 >= 0) ++n1;
					if (n2 >= 0) ++n2;
					if (n3 >= 0) ++n3;
				}

				if (withTexCoordinates)
				{
					stream << " " << i1 << "/" << t1 << "/" << n1;
					stream << " " << i2 << "/" << t2 << "/" << n2;
					stream << " " << i3 << "/" << t3 << "/" << n3;
				}
				else
				{
					stream << " " << i1 << "//" << n1;
					stream << " " << i2 << "//" << n2;
					stream << " " << i3 << "//" << n3;
				}
			}
			else
			{
				if (withTexCoordinates)
				{
					stream << " " << i1 << "/" << t1;
					stream << " " << i2 << "/" << t2;
					stream << " " << i3 << "/" << t3;
				}
				else
				{
					stream << " " << i1;
					stream << " " << i2;
					stream << " " << i3;
				}
			}
			stream << endl;

			if (file.error() != QFile::NoError)
			{
				return CC_FERR_WRITING;
			}

			if (pDlg && !nprogress.oneStep()) //cancel requested
			{
				return CC_FERR_CANCELED_BY_USER;
			}
		}

		stream << "#" << triNum << " faces" << endl;
		if (file.error() != QFile::NoError)
		{
			return CC_FERR_WRITING;
		}

		indexShift += triNum;
	}

	return CC_FERR_NO_ERROR;
}

//! Updates point index to a global index starting from 0!
static bool UpdatePointIndex(int& vIndex, int maxIndex)
{
	if (vIndex == 0 || -vIndex>maxIndex)
		return false;
	vIndex = (vIndex>0 ? vIndex-1 : maxIndex+vIndex);
	return true;
}

//! OBJ facet ('f') element
struct facetElement
{
	//! A set of indexes (vertex, texture coordinates and normal)
	union
	{
		struct
		{
			int vIndex;		//vertex index
			int tcIndex;	//texture coordinate index
			int nIndex;		//normal index
		};
		int indexes[3];
	};

	//! Default constructor
	facetElement()
		: vIndex(0)
		, tcIndex(0)
		, nIndex(0)
	{
	}

	//! Updates point index to a global index starting from 0!
	inline bool updatePointIndex(int maxIndex)
	{
		return UpdatePointIndex(vIndex,maxIndex);
	}

	//! Updates tex coord index to a global index starting from 0!
	inline bool updateTexCoordIndex(int maxIndex)
	{
		if (-tcIndex > maxIndex)
			return false;
		//if tcIndex == 0 then we return '-1'
		tcIndex = (tcIndex >= 0 ? tcIndex-1 : maxIndex+tcIndex);
		return true;
	}

	//! Updates normal index to a global index starting from 0!
	inline bool updateNormalIndex(int maxIndex)
	{
		if (-nIndex > maxIndex)
			return false;
		//if nIndex == 0 then we return '-1'
		nIndex = (nIndex >= 0 ? nIndex-1 : maxIndex+nIndex);
		return true;
	}
};

CC_FILE_ERROR ObjFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	ccLog::Print(QString("[OBJ] ") + filename);

	//open file
	QFile file(filename);
	if (!file.open(QFile::ReadOnly))
		return CC_FERR_READING;
	QTextStream stream(&file);

	//current vertex shift
	CCVector3d Pshift(0, 0, 0);

	//vertices
	ccPointCloud* vertices = new ccPointCloud("vertices");
	int pointsRead = 0;

	//facets
	unsigned int facesRead = 0;
	unsigned int totalFacesRead = 0;
	int maxVertexIndex = -1;

	//base mesh
	ccMesh* baseMesh = new ccMesh(vertices);
	baseMesh->setName(QFileInfo(filename).baseName());
	//we need some space already reserved!
	if (!baseMesh->reserve(128))
	{
		ccLog::Error("Not engouh memory!");
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	//groups (starting index + name)
	std::vector<std::pair<unsigned,QString> > groups;

	//materials
	ccMaterialSet* materials = nullptr;
	bool hasMaterial = false;
	int currentMaterial = -1;
	bool currentMaterialDefined = false;
	bool materialsLoadFailed = true;

	//texture coordinates
	TextureCoordsContainer* texCoords = nullptr;
	bool hasTexCoords = false;
	int texCoordsRead = 0;
	int maxTexCoordIndex = -1;

	//normals
	NormsIndexesTableType* normals = nullptr;
	int normsRead = 0;
	bool normalsPerFacet = false;
	int maxTriNormIndex = -1;

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("OBJ file"));
		pDlg->setInfo(QObject::tr("Loading in progress..."));
		pDlg->setRange(0, static_cast<int>(file.size()));
		pDlg->show();
		QApplication::processEvents();
	}

	//common warnings that can appear multiple time (we avoid to send too many messages to the console!)
	enum OBJ_WARNINGS {	INVALID_NORMALS		= 0,
						INVALID_INDEX		= 1,
						NOT_ENOUGH_MEMORY	= 2,
						INVALID_LINE		= 3,
						CANCELLED_BY_USER	= 4,
	};
	bool objWarnings[5] = { false, false, false, false, false };
	bool error = false;

	try
	{
		unsigned lineCount = 0;
		unsigned polyCount = 0;
		QString currentLine = stream.readLine();
		
		while (!currentLine.isNull())
		{
			++lineCount;
			if (pDlg && ((lineCount % 2048) == 0))
			{
				if (pDlg->wasCanceled())
				{
					error = true;
					objWarnings[CANCELLED_BY_USER] = true;
					break;
				}
				pDlg->setValue(static_cast<int>(file.pos()));
				QApplication::processEvents();
			}

			//specific case for weird files
			while (currentLine.endsWith('\\'))
			{
				currentLine.resize(currentLine.length() - 1);
				currentLine += stream.readLine();
				++lineCount;
				if (pDlg && ((lineCount % 2048) == 0))
				{
					if (pDlg->wasCanceled())
					{
						error = true;
						objWarnings[CANCELLED_BY_USER] = true;
						break;
					}
					pDlg->setValue(static_cast<int>(file.pos()));
					QApplication::processEvents();
				}
			}

			const QStringList tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts );

			//skip comments & empty lines
			if (tokens.empty() || tokens.front().startsWith('/', Qt::CaseInsensitive) || tokens.front().startsWith('#', Qt::CaseInsensitive))
			{
				currentLine = stream.readLine();
				continue;
			}

			/*** new vertex ***/
			if (tokens.front() == "v")
			{
				//reserve more memory if necessary
				if (vertices->size() == vertices->capacity())
				{
					if (!vertices->reserve(vertices->capacity() + ccChunk::SIZE))
					{
						objWarnings[NOT_ENOUGH_MEMORY] = true;
						error = true;
						break;
					}
				}

				//malformed line?
				if (tokens.size() < 4)
				{
					objWarnings[INVALID_LINE] = true;
					error = true;
					break;
				}

				CCVector3d Pd( tokens[1].toDouble(), tokens[2].toDouble(), tokens[3].toDouble() );

				//first point: check for 'big' coordinates
				if (pointsRead == 0)
				{
					bool preserveCoordinateShift = true;
					if (HandleGlobalShift(Pd, Pshift, preserveCoordinateShift, parameters))
					{
						if (preserveCoordinateShift)
						{
							vertices->setGlobalShift(Pshift);
						}
						ccLog::Warning("[OBJ] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
					}
				}

				//shifted point
				CCVector3 P = CCVector3::fromArray((Pd + Pshift).u);
				vertices->addPoint(P);
				++pointsRead;
			}
			/*** new vertex texture coordinates ***/
			else if (tokens.front() == "vt")
			{
				//create and reserve memory for tex. coords container if necessary
				if (!texCoords)
				{
					texCoords = new TextureCoordsContainer();
					texCoords->link();
				}
				if (texCoords->currentSize() == texCoords->capacity())
				{
					if (!texCoords->reserveSafe(texCoords->capacity() + ccChunk::SIZE))
					{
						objWarnings[NOT_ENOUGH_MEMORY] = true;
						error = true;
						break;
					}
				}

				//malformed line?
				if (tokens.size() < 2)
				{
					objWarnings[INVALID_LINE] = true;
					error = true;
					break;
				}

				TexCoords2D T(tokens[1].toFloat(), 0);

				if (tokens.size() > 2) //OBJ specification allows for only one value!!!
				{
					T.ty = tokens[2].toFloat();
				}

				texCoords->addElement(T);
				++texCoordsRead;
			}
			/*** new vertex normal ***/
			else if (tokens.front() == "vn") //--> in fact it can also be a facet normal!!!
			{
				//create and reserve memory for normals container if necessary
				if (!normals)
				{
					normals = new NormsIndexesTableType;
					normals->link();
				}
				if (normals->currentSize() == normals->capacity())
				{
					if (!normals->reserveSafe(normals->capacity() + ccChunk::SIZE))
					{
						objWarnings[NOT_ENOUGH_MEMORY] = true;
						error = true;
						break;
					}
				}

				//malformed line?
				if (tokens.size() < 4)
				{
					objWarnings[INVALID_LINE] = true;
					error = true;
					break;
				}

				CCVector3 N(static_cast<PointCoordinateType>(tokens[1].toDouble()),
							static_cast<PointCoordinateType>(tokens[2].toDouble()),
							static_cast<PointCoordinateType>(tokens[3].toDouble()));

				if (fabs(N.norm2() - 1.0) > 0.005)
				{
					objWarnings[INVALID_NORMALS] = true;
					N.normalize();
				}
				CompressedNormType nIndex = ccNormalVectors::GetNormIndex(N.u);

				normals->addElement(nIndex); //we don't know yet if it's per-vertex or per-triangle normal...
				++normsRead;
			}
			/*** new group ***/
			else if (tokens.front() == "g" || tokens.front() == "o")
			{
				//update new group index
				facesRead = 0;
				//get the group name
				QString groupName = (tokens.size() > 1 && !tokens[1].isEmpty() ? tokens[1] : "default");
				for (int i = 2; i < tokens.size(); ++i) //multiple parts?
					groupName.append(QString(" ") + tokens[i]);
				//push previous group descriptor (if none was pushed)
				if (groups.empty() && totalFacesRead > 0)
					groups.emplace_back(0, "default");
				//push new group descriptor
				if (!groups.empty() && groups.back().first == totalFacesRead)
					groups.back().second = groupName; //simply replace the group name if the previous group was empty!
				else
					groups.emplace_back(totalFacesRead, groupName);
				polyCount = 0; //restart polyline count at 0!
			}
			/*** new face ***/
			else if (tokens.front().startsWith('f'))
			{
				//malformed line?
				if (tokens.size() < 4)
				{
					objWarnings[INVALID_LINE] = true;
					currentLine = stream.readLine();
					continue;
					//error = true;
					//break;
				}

				//read the face elements (singleton, pair or triplet)
				std::vector<facetElement> currentFace;
				{
					currentFace.reserve(tokens.size() - 1);
					for (int i = 1; i < tokens.size(); ++i)
					{
						QStringList vertexTokens = tokens[i].split('/');
						if (vertexTokens.empty() || vertexTokens[0].isEmpty())
						{
							objWarnings[INVALID_LINE] = true;
							error = true;
							break;
						}
						else
						{
							//new vertex
							facetElement fe; //(0,0,0) by default

							fe.vIndex = vertexTokens[0].toInt();
							if (vertexTokens.size() > 1 && !vertexTokens[1].isEmpty())
								fe.tcIndex = vertexTokens[1].toInt();
							if (vertexTokens.size() > 2 && !vertexTokens[2].isEmpty())
								fe.nIndex = vertexTokens[2].toInt();

							currentFace.push_back(fe);
						}
					}
				}

				if (error)
					break;

				if (currentFace.size() < 3)
				{
					ccLog::Warning("[OBJ] Malformed file: polygon on line %1 has less than 3 vertices!",lineCount);
					error = true;
					break;
				}

				//first vertex
				std::vector<facetElement>::iterator A = currentFace.begin();

				//the very first vertex of the group tells us about the whole sequence
				if (facesRead == 0)
				{
					//we have a tex. coord index as second vertex element!
					if (!hasTexCoords && A->tcIndex != 0 && !materialsLoadFailed)
					{
						if (!baseMesh->reservePerTriangleTexCoordIndexes())
						{
							objWarnings[NOT_ENOUGH_MEMORY] = true;
							error = true;
							break;
						}
						for (unsigned int i = 0; i < totalFacesRead; ++i)
							baseMesh->addTriangleTexCoordIndexes(-1, -1, -1);

						hasTexCoords = true;
					}

					//we have a normal index as third vertex element!
					if (!normalsPerFacet && A->nIndex != 0)
					{
						//so the normals are 'per-facet'
						if (!baseMesh->reservePerTriangleNormalIndexes())
						{
							objWarnings[NOT_ENOUGH_MEMORY] = true;
							error = true;
							break;
						}
						for (unsigned int i = 0; i < totalFacesRead; ++i)
							baseMesh->addTriangleNormalIndexes(-1, -1, -1);
						normalsPerFacet = true;
					}
				}

				//we process all vertices accordingly
				for (facetElement& vertex : currentFace)
				{
					//vertex index
					{
						if (!vertex.updatePointIndex(pointsRead))
						{
							objWarnings[INVALID_INDEX] = true;
							error = true;
							break;
						}
						if (vertex.vIndex > maxVertexIndex)
							maxVertexIndex = vertex.vIndex;
					}
					//should we have a tex. coord index as second vertex element?
					if (hasTexCoords && currentMaterialDefined)
					{
						if (!vertex.updateTexCoordIndex(texCoordsRead))
						{
							objWarnings[INVALID_INDEX] = true;
							error = true;
							break;
						}
						if (vertex.tcIndex > maxTexCoordIndex)
							maxTexCoordIndex = vertex.tcIndex;
					}

					//should we have a normal index as third vertex element?
					if (normalsPerFacet)
					{
						if (!vertex.updateNormalIndex(normsRead))
						{
							objWarnings[INVALID_INDEX] = true;
							error = true;
							break;
						}
						if (vertex.nIndex > maxTriNormIndex)
							maxTriNormIndex = vertex.nIndex;
					}
				}

				//don't forget material (common for all vertices)
				if (currentMaterialDefined && !materialsLoadFailed)
				{
					if (!hasMaterial)
					{
						if (!baseMesh->reservePerTriangleMtlIndexes())
						{
							objWarnings[NOT_ENOUGH_MEMORY] = true;
							error = true;
							break;
						}
						for (unsigned int i = 0; i < totalFacesRead; ++i)
							baseMesh->addTriangleMtlIndex(-1);

						hasMaterial = true;
					}
				}

				if (error)
					break;

				//Now, let's tesselate the whole polygon
				bool shouldTesselate = (currentFace.size() > 4 && vertices);
				if (shouldTesselate)
				{
					for (const facetElement& fe : currentFace)
					{
						if (fe.vIndex < 0 || vertices->size() <= static_cast<unsigned>(fe.vIndex))
						{
							//we haven't loaded all the vertices?! Too bad, we can't tesselate properly :(
							ccLog::Warning("[OBJ] Failed to tesselate face");
							shouldTesselate = false;
							break;
						}
					}
				}
				if (shouldTesselate)
				{
					try
					{
						CCLib::PointCloud contour;
						contour.reserve(static_cast<unsigned>(currentFace.size()));

						for (const facetElement& fe : currentFace)
						{
							contour.addPoint(*vertices->getPoint(fe.vIndex));
						}
						CCLib::Delaunay2dMesh* dMesh = CCLib::Delaunay2dMesh::TesselateContour(&contour);
						if (dMesh)
						{
							//need more space?
							unsigned triCount = dMesh->size();
							if (baseMesh->size() + triCount >= baseMesh->capacity())
							{
								if (!baseMesh->reserve(baseMesh->size() + std::max(triCount, 4096u)))
								{
									objWarnings[NOT_ENOUGH_MEMORY] = true;
									error = true;
									break;
								}
							}

							//push new triangle
							const int* _triIndexes = dMesh->getTriangleVertIndexesArray();
							//determine if the triangles must be flipped or not
							bool flip = false;
							{
								for (unsigned i = 0; i < triCount; ++i, _triIndexes += 3)
								{
									int i1 = _triIndexes[0];
									int i2 = _triIndexes[1];
									int i3 = _triIndexes[2];
									//by definition the first edge of the original polygon
									//should be in the same 'direction' of the triangle that uses it
									if (	(i1 == 0 || i2 == 0 || i3 == 0)
										&&	(i1 == 1 || i2 == 1 || i3 == 1) )
									{
										if (	(i1 == 1 && i2 == 0)
											||	(i2 == 1 && i3 == 0)
											||	(i3 == 1 && i1 == 0) )
										{
											flip = true;
										}
										break;
									}
								}
							}

							_triIndexes = dMesh->getTriangleVertIndexesArray();
							for (unsigned i = 0; i < triCount; ++i, _triIndexes += 3)
							{
								const facetElement& f1 = currentFace[_triIndexes[0]];
								facetElement f2 = currentFace[_triIndexes[1]];
								facetElement f3 = currentFace[_triIndexes[2]];

								if (flip)
									std::swap(f2, f3);

								baseMesh->addTriangle(f1.vIndex, f2.vIndex, f3.vIndex);

								if (hasMaterial)
									baseMesh->addTriangleMtlIndex(currentMaterial);

								if (hasTexCoords)
									baseMesh->addTriangleTexCoordIndexes(f1.tcIndex, f2.tcIndex, f3.tcIndex);

								if (normalsPerFacet)
									baseMesh->addTriangleNormalIndexes(f1.nIndex, f2.nIndex, f3.nIndex);

								++facesRead;
								++totalFacesRead;
							}

							delete dMesh;
							dMesh = nullptr;
						}
						else
						{
							ccLog::Warning("[OBJ] Failed to tesselate face");
							shouldTesselate = false;
						}
					}
					catch (const std::bad_alloc&)
					{
						//not enough memory to tesselate!
						shouldTesselate = false;
					}
				}

				if (!shouldTesselate)
				{
					std::vector<facetElement>::const_iterator B = A + 1;
					std::vector<facetElement>::const_iterator C = B + 1;
					for (; C != currentFace.end(); ++B, ++C)
					{
						//need more space?
						if (baseMesh->size() == baseMesh->capacity())
						{
							if (!baseMesh->reserve(baseMesh->size() + 4096))
							{
								objWarnings[NOT_ENOUGH_MEMORY] = true;
								error = true;
								break;
							}
						}

						//push new triangle
						baseMesh->addTriangle(A->vIndex, B->vIndex, C->vIndex);
						++facesRead;
						++totalFacesRead;

						if (hasMaterial)
							baseMesh->addTriangleMtlIndex(currentMaterial);

						if (hasTexCoords)
							baseMesh->addTriangleTexCoordIndexes(A->tcIndex, B->tcIndex, C->tcIndex);

						if (normalsPerFacet)
							baseMesh->addTriangleNormalIndexes(A->nIndex, B->nIndex, C->nIndex);
					}
				}
			}
			/*** polyline ***/
			else if (tokens.front().startsWith('l'))
			{
				//malformed line?
				if (tokens.size() < 3)
				{
					objWarnings[INVALID_LINE] = true;
					currentLine = stream.readLine();
					continue;
				}

				//read the face elements (singleton, pair or triplet)
				ccPolyline* polyline = new ccPolyline(vertices);
				if (!polyline->reserve(static_cast<unsigned>(tokens.size() - 1)))
				{
					//not enough memory
					objWarnings[NOT_ENOUGH_MEMORY] = true;
					delete polyline;
					polyline = nullptr;
					currentLine = stream.readLine();
					continue;
				}

				for (int i = 1; i < tokens.size(); ++i)
				{
					//get next polyline's vertex index
					QStringList vertexTokens = tokens[i].split('/');
					if (vertexTokens.empty() || vertexTokens[0].isEmpty())
					{
						objWarnings[INVALID_LINE] = true;
						error = true;
						break;
					}
					else
					{
						int index = vertexTokens[0].toInt(); //we ignore normal index (if any!)
						if (!UpdatePointIndex(index, pointsRead))
						{
							objWarnings[INVALID_INDEX] = true;
							error = true;
							break;
						}

						polyline->addPointIndex(index);
					}
				}

				if (error)
				{
					delete polyline;
					polyline = nullptr;
					break;
				}

				polyline->setVisible(true);
				QString name = groups.empty() ? QString("Line") : groups.back().second + QString(".line");
				polyline->setName(QString("%1 %2").arg(name).arg(++polyCount));
				vertices->addChild(polyline);

			}
			/*** material ***/
			else if (tokens.front() == "usemtl") //see 'MTL file' below
			{
				if (materials) //otherwise we have failed to load MTL file!!!
				{
					QString mtlName = currentLine.mid(7).trimmed();
					//DGM: in case there's space characters in the material name, we must read it again from the original line buffer
					//QString mtlName = (tokens.size() > 1 && !tokens[1].isEmpty() ? tokens[1] : "");
					currentMaterial = (!mtlName.isEmpty() ? materials->findMaterialByName(mtlName) : -1);
					currentMaterialDefined = true;
				}
			}
			/*** material file (MTL) ***/
			else if (tokens.front() == "mtllib")
			{
				//malformed line?
				if (tokens.size() < 2 || tokens[1].isEmpty())
				{
					objWarnings[INVALID_LINE] = true;
				}
				else
				{
					//we build the whole MTL filename + path
					//DGM: in case there's space characters in the filename, we must read it again from the original line buffer
					//QString mtlFilename = tokens[1];
					QString mtlFilename = currentLine.mid(7).trimmed();
					//remove any quotes around the filename (Photoscan 1.4 bug)
					if (mtlFilename.startsWith("\""))
					{
						mtlFilename = mtlFilename.right(mtlFilename.size() - 1);
					}
					if (mtlFilename.endsWith("\""))
					{
						mtlFilename = mtlFilename.left(mtlFilename.size() - 1);
					}
					ccLog::Print(QString("[OBJ] Material file: ") + mtlFilename);
					QString mtlPath = QFileInfo(filename).canonicalPath();
					//we try to load it
					if (!materials)
					{
						materials = new ccMaterialSet("materials");
						materials->link();
					}

					size_t oldSize = materials->size();
					QStringList errors;
					if (ccMaterialSet::ParseMTL(mtlPath, mtlFilename, *materials, errors))
					{
						ccLog::Print("[OBJ] %i materials loaded", materials->size() - oldSize);
						materialsLoadFailed = false;
					}
					else
					{
						ccLog::Error(QString("[OBJ] Failed to load material file! (should be in '%1')").arg(mtlPath + '/' + QString(mtlFilename)));
						materialsLoadFailed = true;
					}

					if (!errors.empty())
					{
						for (int i = 0; i < errors.size(); ++i)
							ccLog::Warning(QString("[OBJ::Load::MTL parser] ") + errors[i]);
					}
					if (materials->empty())
					{
						materials->release();
						materials = nullptr;
						materialsLoadFailed = true;
					}
				}
			}
			///*** shading group ***/
			//else if (tokens.front() == "s")
			//{
			//	//ignored!
			//}

			if (error)
				break;

			currentLine = stream.readLine();
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		objWarnings[NOT_ENOUGH_MEMORY] = true;
		error = true;
	}

	file.close();

	//1st check
	if (!error && pointsRead == 0)
	{
		//of course if there's no vertex, that's the end of the story ...
		ccLog::Warning("[OBJ] Malformed file: no vertex in file!");
		error = true;
	}

	if (!error)
	{
		ccLog::Print("[OBJ] %i points, %u faces",pointsRead,totalFacesRead);
		if (texCoordsRead > 0 || normsRead > 0)
			ccLog::Print("[OBJ] %i tex. coords, %i normals",texCoordsRead,normsRead);

		//do some cleaning
		vertices->shrinkToFit();
		if (normals)
			normals->shrink_to_fit();
		if (texCoords)
			texCoords->shrink_to_fit();
		if (baseMesh->size() == 0)
		{
			delete baseMesh;
			baseMesh = nullptr;
		}
		else
		{
			baseMesh->shrinkToFit();
		}

		if (	maxVertexIndex >= pointsRead
			||	maxTexCoordIndex >= texCoordsRead
			||	maxTriNormIndex >= normsRead)
		{
			//hum, we've got a problem here
			ccLog::Warning("[OBJ] Malformed file: indexes go higher than the number of elements! (v=%i/tc=%i/n=%i)", maxVertexIndex, maxTexCoordIndex, maxTriNormIndex);
			if (maxVertexIndex >= pointsRead)
			{
				error = true;
			}
			else
			{
				objWarnings[INVALID_INDEX] = true;
				if (maxTexCoordIndex >= texCoordsRead)
				{
					if (texCoords)
					{
						texCoords->release();
						texCoords = nullptr;
					}
					if (materials)
					{
						materials->release();
						materials = nullptr;
					}
				}
				if (maxTriNormIndex >= normsRead)
				{
					if (normals)
					{
						normals->release();
						normals = nullptr;
					}
				}
			}
		}
		
		if (!error && baseMesh)
		{
			if (normals && normalsPerFacet)
			{
				baseMesh->setTriNormsTable(normals);
				baseMesh->showTriNorms(true);
			}
			if (materials)
			{
				baseMesh->setMaterialSet(materials);
				baseMesh->showMaterials(true);
			}
			if (texCoords)
			{
				if (materials)
				{
					baseMesh->setTexCoordinatesTable(texCoords);
				}
				else
				{
					ccLog::Warning("[OBJ] Texture coordinates were defined but no material could be loaded!");
				}
			}

			//normals: if the obj file doesn't provide any, let the user know
			if (!normals)
			{
				ccLog::Warning("[OBJ] Mesh has no normals! You can manually compute them (select it then call \"Edit > Normals > Compute\")");
			}

			//create sub-meshes if necessary
			ccLog::Print("[OBJ] 1 mesh loaded - %i group(s)", groups.size());
			if (groups.size() > 1)
			{
				for (size_t i = 0; i < groups.size(); ++i)
				{
					const QString& groupName = groups[i].second;
					unsigned startIndex = groups[i].first;
					unsigned endIndex = (i + 1 == groups.size() ? baseMesh->size() : groups[i + 1].first);

					if (startIndex == endIndex)
					{
						continue;
					}

					ccSubMesh* subTri = new ccSubMesh(baseMesh);
					if (subTri->reserve(endIndex-startIndex))
					{
						subTri->addTriangleIndex(startIndex, endIndex);
						subTri->setName(groupName);
						subTri->showMaterials(baseMesh->materialsShown());
						subTri->showNormals(baseMesh->normalsShown());
						subTri->showTriNorms(baseMesh->triNormsShown());
						//subTri->showColors(baseMesh->colorsShown());
						//subTri->showWired(baseMesh->isShownAsWire());
						baseMesh->addChild(subTri);
					}
					else
					{
						delete subTri;
						subTri = nullptr;
						objWarnings[NOT_ENOUGH_MEMORY] = true;
					}
				}
				baseMesh->setVisible(false);
				vertices->setLocked(true);
			}

			baseMesh->addChild(vertices);
			//DGM: we can't deactive the vertices if it has children! (such as polyline)
			if (vertices->getChildrenNumber() != 0)
				vertices->setVisible(false);
			else
				vertices->setEnabled(false);

			container.addChild(baseMesh);
		}

		if (!baseMesh && vertices->size() != 0)
		{
			//no (valid) mesh!
			container.addChild(vertices);
			//we hide the vertices if the entity has children (probably polylines!)
			if (vertices->getChildrenNumber() != 0)
			{
				vertices->setVisible(false);
			}
		}

		//special case: normals held by cloud!
		if (normals && !normalsPerFacet)
		{
			if (normsRead == pointsRead) //must be 'per-vertex' normals
			{
				vertices->setNormsTable(normals);
				if (baseMesh)
					baseMesh->showNormals(true);
			}
			else
			{
				ccLog::Warning("File contains normals which seem to be neither per-vertex nor per-face!!! We had to ignore them...");
			}
		}
	}

	if (error)
	{
		delete baseMesh;
		delete vertices;
	}

	//release shared structures
	if (normals)
	{
		normals->release();
		normals = nullptr;
	}
	if (texCoords)
	{
		texCoords->release();
		texCoords = nullptr;
	}
	if (materials)
	{
		materials->release();
		materials = nullptr;
	}

	if (pDlg)
	{
		pDlg->close();
	}

	//potential warnings
	if (objWarnings[INVALID_NORMALS])
		ccLog::Warning("[OBJ] Some normals in file were invalid. You should re-compute them (select entity, then \"Edit > Normals > Compute\")");
	if (objWarnings[INVALID_INDEX])
		ccLog::Warning("[OBJ] File is malformed! Check indexes...");
	if (objWarnings[NOT_ENOUGH_MEMORY])
		ccLog::Warning("[OBJ] Not enough memory!");
	if (objWarnings[INVALID_LINE])
		ccLog::Warning("[OBJ] File is malformed! Missing data.");

	if (error)
	{
		if (objWarnings[NOT_ENOUGH_MEMORY])
			return CC_FERR_NOT_ENOUGH_MEMORY;
		else if (objWarnings[CANCELLED_BY_USER])
			return CC_FERR_CANCELED_BY_USER;
		else 
			return CC_FERR_MALFORMED_FILE;
	}
	else
	{
		return CC_FERR_NO_ERROR;
	}
}
