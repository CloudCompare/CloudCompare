//##########################################################################
//#                                                                        #
//#                 CLOUDCOMPARE PLUGIN: qSTEPCADImport                    #
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
//#                          COPYRIGHT: EDF R&D                            #
//#                                                                        #
//##########################################################################

#include "../include/STEPFilter.h"

//Qt
#include <QFile>
#include <QFileInfo>
#include <QInputDialog>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccLog.h>

// Include OpenCascade :
#include "STEPControl_Reader.hxx"
#include "Interface_Static.hxx"
#include "IFSelect_ReturnStatus.hxx"
#include "IFSelect_PrintCount.hxx"

#include "BRepTools.hxx"
#include "BRepBuilderAPI_MakePolygon.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"
#include "BRepBuilderAPI_MakeSolid.hxx"
#include "BRep_Builder.hxx"
#include "Poly_Triangulation.hxx"

#include "TopoDS.hxx"
#include "TopoDS_Compound.hxx"
#include "TopoDS_Solid.hxx"
#include "TopoDS_Shell.hxx"
#include "TopAbs_ShapeEnum.hxx"

#include "TopTools.hxx"
#include "TColgp_Array1OfPnt.hxx"
#include "TopExp_Explorer.hxx"
#include "TopExp.hxx"
#include "BRepBuilderAPI_Sewing.hxx"

#include "TopTools_ListOfShape.hxx"
#include "NCollection_List.hxx"

#include "BRepMesh_FastDiscret.hxx"
#include "BRepMesh_IncrementalMesh.hxx"

using namespace std;

static std::map<TopAbs_ShapeEnum, QString> ShapeTypes
{
	{TopAbs_COMPOUND,  "TopAbs_COMPOUND"},
	{TopAbs_COMPSOLID, "TopAbs_COMPSOLID"},
	{TopAbs_SOLID,     "TopAbs_SOLID"},
	{TopAbs_SHELL,     "TopAbs_SHELL"},
	{TopAbs_FACE,      "TopAbs_FACE"},
	{TopAbs_WIRE,      "TopAbs_WIRE"},
	{TopAbs_EDGE,      "TopAbs_EDGE"},
	{TopAbs_VERTEX,    "TopAbs_VERTEX"},
	{TopAbs_SHAPE,     "TopAbs_SHAPE"}
};

STEPFilter::STEPFilter()
	: FileIOFilter( {
					"_STEP OpenCascade Filter",
					DEFAULT_PRIORITY,	//priority
					QStringList{ "step", "stp" },
					"step",
					QStringList{ "STEP CAD file (*.step *.stp)"},
					QStringList(),
					Import
					} )
{
}

static double s_defaultLinearDeflection = 1.0e-3;

void STEPFilter::SetDefaultLinearDeflection(double value)
{
	if (value < 1.0e-6)
	{
		ccLog::Warning("[STEP] Input linear deflection is too small");
		return;
	}
	if (value > 1.0e-2)
	{
		ccLog::Warning("[STEP] Input linear deflection is too big");
		return;
	}
	s_defaultLinearDeflection = value;
}

CC_FILE_ERROR STEPFilter::loadFile( const QString& fullFilename,
									ccHObject& container, 
									LoadParameters& parameters )
{
	// check for the file existence
	QFileInfo fi(fullFilename);
	if (!fi.exists())
	{
		return CC_FERR_UNKNOWN_FILE;
	}
	
	if (parameters.sessionStart && parameters.parentWidget)
	{
		bool ok = false;
		double linearDeflection = QInputDialog::getDouble(parameters.parentWidget, "Linear deflection", "Linear deflection", s_defaultLinearDeflection, 1.0e-6, 1.0e-2, 6, &ok);
		if (!ok)
		{
			return CC_FERR_CANCELED_BY_USER;
		}
		SetDefaultLinearDeflection(linearDeflection);
	}
	
	CC_FILE_ERROR error;
	try
	{
		error = importStepFile(container, fullFilename, s_defaultLinearDeflection, parameters);
	}
	catch (Standard_Failure e)
	{
		ccLog::Warning(QString("OpenCascade error: ") + e.GetMessageString());
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}

	return error;
}

CC_FILE_ERROR STEPFilter::importStepFile(	ccHObject& container,
											const QString& fullFileName,
											double linearDeflection,
											LoadParameters& parameters )
{
	//Interface_Static::SetCVal("xstep.cascade.unit", "M"); // DGM: seems to mess things completely, having various impacts during the next run (smaller scale, etc.)

	STEPControl_Reader aReader;
	IFSelect_ReturnStatus aStatus = aReader.ReadFile(qUtf8Printable(fullFileName));
	if (aStatus != IFSelect_ReturnStatus::IFSelect_RetDone)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	// Collecting entities inside the STEP file
	int rootCount = aReader.NbRootsForTransfer();
	if (rootCount < 1)
	{
		ccLog::Warning("No root found in the STEP file.");
		return CC_FERR_NO_LOAD;
	}
	ccLog::Print(QString("[STEP] Number of root(s): %1").arg(rootCount));

	for (Standard_Integer n = 1; n <= rootCount; n++)
	{
		if (!aReader.TransferRoot(n))
		{
			ccLog::Warning(QString("[STEP] Failed to transfer root #%1").arg(n));
		}
	}
	// Root transfers
	int transferredRootCount = aReader.TransferRoots();
	if (transferredRootCount < 1)
	{
		ccLog::Warning("No root could be transferred from the STEP file.");
		return CC_FERR_NO_LOAD;
	}

	Standard_Integer shapeCount = aReader.NbShapes();
	if (shapeCount == 0)
	{
		ccLog::Warning("No shape found in the STEP file.");
		return CC_FERR_NO_LOAD;
	}
	ccLog::Print(QString("[STEP] Number of shapes: %1").arg(shapeCount));

	TopoDS_Shape aShape = aReader.OneShape();
	ccLog::Print("[STEP] Shape type: " + ShapeTypes[aShape.ShapeType()]);
	
	BRepMesh_IncrementalMesh incrementalMesh(aShape, linearDeflection, Standard_True); //not explicitly used, but still needs to be instantiated

	// Creation of the CC vertices and mesh
	ccPointCloud* vertices = new ccPointCloud("vertices");
	vertices->setEnabled(false);
	ccMesh* mesh = new ccMesh(vertices);
	mesh->addChild(vertices);
	mesh->setName("unnamed - tesselated");

	// Notice that the nodes are duplicated during CAD tesslation : if a node is
	// belonging to N triangles, it's duplicated N times.
	unsigned faceCount = 0;
	unsigned triCount = 0; // Number of triangles of the tesselated CAD shape imported
	unsigned vertCount = 0;
	TopExp_Explorer expFaces;
	for (expFaces.Init(aShape, TopAbs_FACE); expFaces.More(); expFaces.Next())
	{
		const TopoDS_Face& face = TopoDS::Face(expFaces.Current());
		++faceCount;

		TopLoc_Location location;
		const auto& facing = BRep_Tool::Triangulation(face, location);
		if (!facing)
		{
			delete mesh;
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}

		vertCount += static_cast<unsigned>(facing->NbNodes());
		triCount += static_cast<unsigned>(facing->NbTriangles());

		if (triCount > mesh->capacity() && !mesh->reserve(triCount + 65536))
		{
			delete mesh;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		if (vertCount > vertices->capacity() && !vertices->reserve(vertCount + 65536))
		{
			delete mesh;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		gp_Trsf nodeTransformation = location;
		const Poly_Array1OfTriangle& triangles = facing->Triangles();
		TopAbs_Orientation orientation = face.Orientation();

		for (int j = 1; j <= facing->NbTriangles(); j++)
		{
			Standard_Integer index[3] { 0 };
			triangles.Value(j).Get(index[0], index[1], index[2]);
			
			unsigned vertIndexes[3] { 0 };
			for (unsigned k = 0; k < 3; ++k)
			{
				gp_Pnt pk = facing->Node(index[k]).Transformed(nodeTransformation);

				vertIndexes[k] = vertices->size();

				vertices->addPoint(CCVector3(pk.X(), pk.Y(), pk.Z()));
			}
			if (orientation == TopAbs_REVERSED)
				mesh->addTriangle(vertIndexes[0], vertIndexes[2], vertIndexes[1]);
			else
				mesh->addTriangle(vertIndexes[0], vertIndexes[1], vertIndexes[2]);
		}
	}

	mesh->shrinkToFit();
	vertices->shrinkToFit();

	ccLog::Print("[STEP] Number of CAD faces  = " + QString::number(faceCount));
	ccLog::Print("[STEP] Number of triangles (after tesselation) = " + QString::number(triCount));
	ccLog::Print("[STEP] Number of vertices (after tesselation)  = " + QString::number(vertCount));

	mesh->mergeDuplicatedVertices(ccMesh::DefaultMergeDuplicateVerticesLevel, parameters.parentWidget);
	vertices = nullptr; //warning, after this point, 'vertices' is not valid anymore

	if (mesh->computePerTriangleNormals())
	{
		mesh->showNormals(true);
	}

	container.addChild(mesh);

	return CC_FERR_NO_ERROR;
}
