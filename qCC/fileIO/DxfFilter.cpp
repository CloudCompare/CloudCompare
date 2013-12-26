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

#include "DxfFilter.h"

//Qt
#include <QApplication>
#include <QFile>

//CCLib
#include <ScalarField.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccMesh.h>
#include <ccLog.h>
#include <ccNormalVectors.h>

//DXF lib
#ifdef CC_DXF_SUPPORT
#include <dl_dxf.h>
#include <dl_creationadapter.h>
#endif

//system
#include <assert.h>

#ifdef CC_DXF_SUPPORT

//! dxflib-to-CC custom mapper
class DxfImporter : public DL_CreationAdapter
{
public:
	//! Default constructor
	DxfImporter(ccHObject* root)
		: m_root(root)
		, m_points(0)
		, m_faces(0)
		, m_poly(0)
		, m_polyVertices(0)
	{
		assert(m_root);
	}
   
	virtual void addLayer(const DL_LayerData& data)
	{
		// store our layer colours
		m_layerColourMap[data.name.c_str()] = getAttributes().getColor();
	}
   
	virtual void addPoint(const DL_PointData& P)
	{
		//create the 'points' point cloud if necessary
		if (!m_points)
		{
			m_points = new ccPointCloud("Points");
			m_root->addChild(m_points);
		}
		if (!m_points->reserve(m_points->size()+1))
		{
			ccLog::Error("[DxfImporter] Not enough memory!");
			return;
		}

		m_points->addPoint(CCVector3(	static_cast<PointCoordinateType>(P.x),
										static_cast<PointCoordinateType>(P.y),
										static_cast<PointCoordinateType>(P.z) ));

		colorType col[3];
		if (getCurrentColour(col))
		{
			//RGB field already instantiated?
			if (m_points->hasColors())
			{
				m_points->addRGBColor(col);
			}
			//otherwise, reserve memory and set all previous points to white by default
			else if (m_points->setRGBColor(ccColor::white))
			{
				//then replace the last color by the current one
				m_points->setPointColor(m_points->size()-1,col);
				m_points->showColors(true);
			}
		}
		else if (m_points->hasColors())
		{
			//add default color if none is defined!
			m_points->addRGBColor(ccColor::white);
		}
	}

	virtual void addPolyline(const DL_PolylineData& poly)
	{
		//create a new polyline if necessary
		if (m_poly && !m_poly->size())
			delete m_poly;
		m_polyVertices = new ccPointCloud("vertices");
		m_poly = new ccPolyline(m_polyVertices);
		m_poly->addChild(m_polyVertices);
		if (!m_polyVertices->reserve(poly.number) || !m_poly->reserve(poly.number))
		{
			ccLog::Error("[DxfImporter] Not enough memory!");
			delete m_poly;
			m_polyVertices = 0;
			m_poly = 0;
			return;
		}
		m_polyVertices->setVisible(false);
		m_poly->setVisible(true);
		m_poly->setName("Polyline");

		//flags
		m_poly->setClosed(poly.flags & 1);
		//m_poly->set2DMode(poly.flags & 8); //DGM: "2D" polylines in CC doesn't mean the same thing ;)

		//color
		colorType col[3];
		if (getCurrentColour(col))
		{
			m_poly->setColor(col);
			m_poly->showColors(true);
		}
	}

	virtual void addVertex(const DL_VertexData& vertex)
	{
		//we assume it's a polyline vertex!
		if (	m_poly
			&&	m_polyVertices
			&&	m_polyVertices->size() < m_polyVertices->capacity() )
		{
			m_poly->addPointIndex(m_polyVertices->size());
			m_polyVertices->addPoint(CCVector3(	static_cast<PointCoordinateType>(vertex.x),
												static_cast<PointCoordinateType>(vertex.y),
												static_cast<PointCoordinateType>(vertex.z) ));

			if (m_poly->size() == 1)
				m_root->addChild(m_poly);
		}
	}

	virtual void add3dFace(const DL_3dFaceData& face)
	{
		//TODO: understand what this really is?!
		CCVector3 P[4];
		for (unsigned i=0; i<4; ++i)
		{
			P[i] = CCVector3(	static_cast<PointCoordinateType>(face.x[i]),
								static_cast<PointCoordinateType>(face.y[i]),
								static_cast<PointCoordinateType>(face.z[i]) );
		}
		
		//create the 'faces' mesh if necessary
		if (!m_faces)
		{
			ccPointCloud* vertices = new ccPointCloud("vertices");
			m_faces = new ccMesh(vertices);
			m_faces->setName("Faces");
			m_faces->addChild(vertices);
			m_faces->setVisible(true);
			vertices->setEnabled(false);
			vertices->setLocked(true);
			
			m_root->addChild(m_faces);
		}
		
		ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_faces->getAssociatedCloud());
		if (!vertices)
		{
			assert(false);
			return;
		}
		
		int vertIndexes[4] = {-1, -1, -1, -1};
		unsigned addedVertCount = 4;
		//check if the two last vertices are the same
		if (P[2].x == P[3].x && P[2].y == P[3].y && P[2].z == P[3].z)
			addedVertCount = 3;

		//current face color
		colorType col[3];
		colorType* faceCol = 0;
		if (getCurrentColour(col))
			faceCol = col;


		//look for already defined vertices
		unsigned vertCount = vertices->size();
		if (vertCount)
		{
			//DGM TODO: could we be smarter?
			for (unsigned i=0; i<addedVertCount; ++i)
			{
				for (unsigned j=0; j<vertCount; ++j)
				{
					const CCVector3* Pj = vertices->getPoint(j);
					if (P[i].x == Pj->x && P[i].y == Pj->y && P[i].z == Pj->z)
					{
						bool useCurrentVertex = true;

						//We must also check that the color is the same (if any)
						if (faceCol || vertices->hasColors())
						{
							const colorType* _faceCol = faceCol ? faceCol : ccColor::white;
							const colorType* _vertCol = vertices->hasColors() ? vertices->getPointColor(j) : ccColor::white;
							useCurrentVertex = (_faceCol[0] == _vertCol[0] && _faceCol[1] == _vertCol[1] && _faceCol[2] == _vertCol[2]);
						}

						if (useCurrentVertex)
						{
							vertIndexes[i] = static_cast<int>(j);
							break;
						}
					}
				}
			}
		}

		//now create new vertices
		unsigned createdVertCount = 0;
		{
			for (unsigned i=0; i<addedVertCount; ++i)
				if (vertIndexes[i] < 0)
					++createdVertCount;
		}

		if (createdVertCount != 0)
		{
			//reserve memory for the new vertices
			if (!vertices->reserve(vertCount+createdVertCount))
			{
				ccLog::Error("[DxfImporter] Not enough memory!");
				return;
			}

			for (unsigned i=0; i<addedVertCount; ++i)
			{
				if (vertIndexes[i] < 0)
				{
					vertIndexes[i] = static_cast<int>(vertCount++);
					vertices->addPoint(P[i]);
				}
			}
		}

		//number of triangles to add
		unsigned addTriCount = (addedVertCount == 3 ? 1 : 2);

		//now add the corresponding face(s)
		if (!m_faces->reserve(m_faces->size() + addTriCount))
		{
			ccLog::Error("[DxfImporter] Not enough memory!");
			return;
		}
		m_faces->addTriangle(vertIndexes[0], vertIndexes[1], vertIndexes[2]);
		if (addedVertCount == 4)
			m_faces->addTriangle(vertIndexes[0], vertIndexes[2], vertIndexes[3]);

		//add per-triangle normals
		{
			//normals table
			NormsIndexesTableType* triNormsTable = m_faces->getTriNormsTable();
			bool firstTime = false;
			if (!triNormsTable)
			{
				triNormsTable = new NormsIndexesTableType(); 
				m_faces->setTriNormsTable(triNormsTable);
				m_faces->addChild(triNormsTable);
				firstTime = true;
			}

			//add 1 or 2 new entries
			unsigned triNormCount = triNormsTable->currentSize();
			if (!triNormsTable->reserve(triNormsTable->currentSize() + addTriCount))
			{
				ccLog::Error("[DxfImporter] Not enough memory!");
				return;
			}
			
			CCVector3 N = (P[1]-P[0]).cross(P[2]-P[0]);
			N.normalize();
			triNormsTable->addElement(ccNormalVectors::GetNormIndex(N.u));
			if (addTriCount == 2)
			{
				N = (P[2]-P[0]).cross(P[3]-P[0]);
				N.normalize();
				triNormsTable->addElement(ccNormalVectors::GetNormIndex(N.u));
			}

			//per-triangle normals indexes
			if (firstTime)
			{
				if (!m_faces->reservePerTriangleNormalIndexes())
				{
					ccLog::Error("[DxfImporter] Not enough memory!");
					return;
				}
				m_faces->showNormals(true);
			}
			int n1 = static_cast<int>(triNormCount);
			m_faces->addTriangleNormalIndexes(n1, n1, n1);
			if (addTriCount == 2)
			{
				int n2 = static_cast<int>(triNormCount+1);
				m_faces->addTriangleNormalIndexes(n2, n2, n2);
			}
		}

		//and now for the color
		if (faceCol)
		{
			//RGB field already instantiated?
			if (vertices->hasColors())
			{
				for (unsigned i=0; i<createdVertCount; ++i)
					vertices->addRGBColor(faceCol);
			}
			//otherwise, reserve memory and set all previous points to white by default
			else if (vertices->setRGBColor(ccColor::white))
			{
				//then replace the last color(s) by the current one
				for (unsigned i=0; i<createdVertCount; ++i)
					vertices->setPointColor(vertCount-1-i,faceCol);
				m_faces->showColors(true);
			}
		}
		else if (vertices->hasColors())
		{
			//add default color if none is defined!
			for (unsigned i=0; i<createdVertCount; ++i)
				vertices->addRGBColor(ccColor::white);
		}
	}

	virtual void addLine(const DL_LineData& line)
	{
		//we open lines as simple polylines!
		ccPointCloud* polyVertices = new ccPointCloud("vertices");
		ccPolyline* poly = new ccPolyline(polyVertices);
		poly->addChild(polyVertices);
		if (!polyVertices->reserve(2) || !poly->reserve(2))
		{
			ccLog::Error("[DxfImporter] Not enough memory!");
			delete poly;
			return;
		}
		polyVertices->setVisible(false);
		poly->setVisible(true);
		poly->setName("Line");
		poly->addPointIndex(0,2);
		//add first point
		polyVertices->addPoint(CCVector3(	static_cast<PointCoordinateType>(line.x1),
											static_cast<PointCoordinateType>(line.y1),
											static_cast<PointCoordinateType>(line.z1) ));
		//add second point
		polyVertices->addPoint(CCVector3(	static_cast<PointCoordinateType>(line.x2),
											static_cast<PointCoordinateType>(line.y2),
											static_cast<PointCoordinateType>(line.z2) ));

		//flags
		poly->setClosed(false);

		//color
		colorType col[3];
		if (getCurrentColour(col))
		{
			poly->setColor(col);
			poly->showColors(true);
		}
      
		m_root->addChild(poly);
	}

protected:

	//! Root object (new objects will be added as its children)
	ccHObject* m_root;

	//! Points
	ccPointCloud* m_points;
	//! Faces
	ccMesh* m_faces;
	//! Current polyline (if any)
	ccPolyline* m_poly;
	//! Current polyline vertices
	ccPointCloud* m_polyVertices;

private:

	//!Returns current colour (either the current data's colour or the current layer's one)
	bool getCurrentColour( colorType ccColour[/*3*/] )
	{
		const DL_Attributes attributes = getAttributes();

		int colourIndex = attributes.getColor();

		if ( colourIndex == 0 )
		{
			// TODO Colours BYBLOCK not handled
			return false;
		}
		else if ( colourIndex == 256 )
		{
			// an attribute of 256 means the colours are BYLAYER, so grab it from our map instead
			const int defaultIndex = -1;
			colourIndex = m_layerColourMap.value( attributes.getLayer().c_str(), defaultIndex );

			//if we don't have any information on the current layer
			if (colourIndex == defaultIndex)
				return false;
		}

		ccColour[0] = static_cast<colorType>( dxfColors[colourIndex][0] * MAX_COLOR_COMP );
		ccColour[1] = static_cast<colorType>( dxfColors[colourIndex][1] * MAX_COLOR_COMP );
		ccColour[2] = static_cast<colorType>( dxfColors[colourIndex][2] * MAX_COLOR_COMP );

		return true;
	}

	//! Keep track of the colour of each layer in case the colour attribute is set to BYLAYER
	QMap<QString,int> m_layerColourMap;

};

#endif //CC_DXF_SUPPORT

CC_FILE_ERROR DxfFilter::saveToFile(ccHObject* root, const char* filename)
{
#ifndef CC_DXF_SUPPORT

	ccLog::Error("[DXF] DXF format not supported! Check compilation parameters!");
	return CC_FERR_CONSOLE_ERROR;

#else

	if (!root || !filename)
		return CC_FERR_BAD_ARGUMENT;

	ccHObject::Container polylines;
	root->filterChildren(polylines,true,CC_POLY_LINE);

	//only polylines are handled for now
	size_t polyCount = polylines.size();
	if (!polyCount)
		return CC_FERR_NO_SAVE;

	//get global bounding box
	ccBBox box;
	for (size_t i=0; i<polyCount; ++i)
	{
		ccBBox polyBox = polylines[i]->getBB();
		if (i)
		{
			box += polyBox;
		}
		else
		{
			box = polyBox;
		}
	}

	CCVector3 diag = box.getDiagVec();
	double baseSize = static_cast<double>(std::max(diag.x,diag.y));
	double lineWidth = baseSize / 40.0;
	double pageMargin = baseSize / 20.0;

	DL_Dxf dxf;
	DL_WriterA* dw = dxf.out(filename, DL_VERSION_R12);
	if (!dw)
	{
		return CC_FERR_WRITING;
	}

	//write header
	dxf.writeHeader(*dw);

	//add dimensions
	dw->dxfString(9, "$INSBASE");
	dw->dxfReal(10,0.0);
	dw->dxfReal(20,0.0);
	dw->dxfReal(30,0.0);
	dw->dxfString(9, "$EXTMIN");
	dw->dxfReal(10,static_cast<double>(box.minCorner().x)-pageMargin);
	dw->dxfReal(20,static_cast<double>(box.minCorner().y)-pageMargin);
	dw->dxfReal(30,static_cast<double>(box.minCorner().z)-pageMargin);
	dw->dxfString(9, "$EXTMAX");
	dw->dxfReal(10,static_cast<double>(box.maxCorner().x)+pageMargin);
	dw->dxfReal(20,static_cast<double>(box.maxCorner().y)+pageMargin);
	dw->dxfReal(30,static_cast<double>(box.maxCorner().z)+pageMargin);
	dw->dxfString(9, "$LIMMIN");
	dw->dxfReal(10,static_cast<double>(box.minCorner().x)-pageMargin);
	dw->dxfReal(20,static_cast<double>(box.minCorner().y)-pageMargin);
	dw->dxfString(9, "$LIMMAX");
	dw->dxfReal(10,static_cast<double>(box.maxCorner().x)+pageMargin);
	dw->dxfReal(20,static_cast<double>(box.maxCorner().y)+pageMargin);

	//close header
	dw->sectionEnd();

	//Opening the Tables Section
	dw->sectionTables();
	//Writing the Viewports
	dxf.writeVPort(*dw);

	//Writing the Linetypes (all by default)
	{
		dw->tableLineTypes(25);
		dxf.writeLineType(*dw, DL_LineTypeData("BYBLOCK", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("BYLAYER", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("CONTINUOUS", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("ACAD_ISO02W100", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("ACAD_ISO03W100", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("ACAD_ISO04W100", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("ACAD_ISO05W100", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("BORDER", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("BORDER2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("BORDERX2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("CENTER", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("CENTER2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("CENTERX2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DASHDOT", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DASHDOT2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DASHDOTX2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DASHED", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DASHED2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DASHEDX2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DIVIDE", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DIVIDE2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DIVIDEX2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DOT", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DOT2", 0));
		dxf.writeLineType(*dw, DL_LineTypeData("DOTX2", 0));
		dw->tableEnd();
	}

	//Writing the Layers
	dw->tableLayers(static_cast<int>(polyCount)+1);
	QStringList layerNames;
	{
		//default layer
		dxf.writeLayer(*dw, 
			DL_LayerData("0", 0), 
			DL_Attributes(
			std::string(""),      // leave empty
			DL_Codes::black,      // default color
			100,                  // default width (in 1/100 mm)
			"CONTINUOUS"));       // default line style

		//polylines layers
		for (unsigned i=0; i<polyCount; ++i)
		{
			//default layer name
			//TODO: would be better to use the polyline name!
			//but it can't be longer than 31 characters (R14 limit)
			QString layerName = QString("POLYLINE_%1").arg(i+1,3,10,QChar('0'));

			layerNames << layerName;
			dxf.writeLayer(*dw, 
				DL_LayerData(layerName.toStdString(), 0), 
				DL_Attributes(
				std::string(""),
				i == 0 ? DL_Codes::green : /*-*/DL_Codes::green, //invisible if negative!
				static_cast<int>(lineWidth),
				"CONTINUOUS"));
		}
	}
	dw->tableEnd();

	//Writing Various Other Tables
	//dxf.writeStyle(*dw); //DXFLIB V2.5
	dxf.writeStyle(*dw,DL_StyleData("Standard",0,0.0,0.75,0.0,0,2.5,"txt","")); //DXFLIB V3.3
	dxf.writeView(*dw);
	dxf.writeUcs(*dw);

	dw->tableAppid(1);
	dw->tableAppidEntry(0x12);
	dw->dxfString(2, "ACAD");
	dw->dxfInt(70, 0);
	dw->tableEnd();

	//Writing Dimension Styles
	dxf.writeDimStyle(	*dw, 
						/*arrowSize*/1, 
						/*extensionLineExtension*/1,
						/*extensionLineOffset*/1,
						/*dimensionGap*/1,
						/*dimensionTextSize*/1);
	
	//Writing Block Records
	dxf.writeBlockRecord(*dw);
	dw->tableEnd();

	//Ending the Tables Section
	dw->sectionEnd();

	//Writing the Blocks Section
	{
		dw->sectionBlocks();

		dxf.writeBlock(*dw,  DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Model_Space");

		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space");

		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space0");

		dw->sectionEnd();
	}

	//Writing the Entities Section
	{
		dw->sectionEntities();

		//write polylines
		for (unsigned i=0; i<polyCount; ++i)
		{
			const ccPolyline* poly = static_cast<ccPolyline*>(polylines[i]);
			unsigned vertexCount = poly->size();
			int flags = poly->isClosed() ? 1 : 0;
			if (!poly->is2DMode())
				flags |= 8; //3D polyline
			dxf.writePolyline(	*dw,
								DL_PolylineData(static_cast<int>(vertexCount),0,0,flags),
								DL_Attributes(layerNames[i].toStdString(), DL_Codes::bylayer, -1, "BYLAYER") );

			for (unsigned i=0; i<vertexCount; ++i)
			{
				CCVector3 P;
				poly->getPoint(i,P);
				dxf.writeVertex(*dw, DL_VertexData(	P.x, P.y, P.y ) );
			}

			dxf.writePolylineEnd(*dw);
		}

		dw->sectionEnd();
	}

	//Writing the Objects Section
	dxf.writeObjects(*dw);
	dxf.writeObjectsEnd(*dw);

	//Ending and Closing the File
	dw->dxfEOF();
	dw->close();
	delete dw;
	dw = 0;

	return CC_FERR_NO_ERROR;

#endif
}

CC_FILE_ERROR DxfFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
#ifdef CC_DXF_SUPPORT
	DxfImporter importer(&container);
	if (!DL_Dxf().in(filename, &importer))
	{
		return CC_FERR_READING;
	}
#else
	ccLog::Error("[DXF] Not supported in this version!");
#endif

	return container.getChildrenNumber() == 0 ? CC_FERR_NO_LOAD : CC_FERR_NO_ERROR;
}
