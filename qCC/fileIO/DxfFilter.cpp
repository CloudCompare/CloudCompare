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
#include <ccLog.h>

//DXF lib
#ifdef CC_DXF_SUPPORT
#include <dl_dxf.h>
#include <dl_creationadapter.h>
#endif

//system
#include <assert.h>

#ifdef CC_DXF_SUPPORT
class DxfImporter : public DL_CreationAdapter
{
public:
	//! Default constructor
	DxfImporter(ccHObject* root)
		: m_root(root)
		, m_points(0)
		, m_poly(0)
		, m_polyVertices(0)
	{
		assert(m_root);
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

	//virtual void add3dFace(const DL_3dFaceData&)
	//{
	//	//TODO: understand what this really is?!
	//}

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
		m_root->addChild(poly);
	}

protected:

	//! Root object (new objects will be added as its children)
	ccHObject* m_root;

	//! Points
	ccPointCloud* m_points;
	//! Current polyline (if any)
	ccPolyline* m_poly;
	//! Current polyline vertices
	ccPointCloud* m_polyVertices;

};
#endif

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
	DL_WriterA* dw = dxf.out(qPrintable(filename), DL_VERSION_R12);
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
				lineWidth,
				"CONTINUOUS"));
		}
	}
	dw->tableEnd();

	//Writing Various Other Tables
	dxf.writeStyle(*dw);
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

	ccLog::Print("[DXF] File %s saved successfully",filename);

	return CC_FERR_NO_ERROR;

#endif
}

CC_FILE_ERROR DxfFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
#ifdef CC_DXF_SUPPORT
	DxfImporter importer(&container);
	if (!DL_Dxf().in(qPrintable(filename), &importer))
	{
		return CC_FERR_READING;
	}
#else
	ccLog::Error("[DXF] Not supported in this version!");
#endif

	return CC_FERR_NO_ERROR;
}
