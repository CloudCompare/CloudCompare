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

#include "MAFilter.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccProgressDialog.h>

//Qt
#include <QFileInfo>

//System
#include <string.h>
#include <assert.h>

struct edge
{
    int edgeIndex;
	bool positif;
	unsigned theOtherPoint;
	edge* nextEdge;
};

struct faceIndexes
{
    int faceIndex;
	faceIndexes* nextFace;
};

CC_FILE_ERROR MAFilter::saveToFile(ccHObject* entity, const char* filename)
{
	if (!entity || !filename)
        return CC_FERR_BAD_ARGUMENT;

	ccHObject::Container meshes;
	if (entity->isKindOf(CC_MESH))
        meshes.push_back(entity);
    else
        entity->filterChildren(meshes, true, CC_MESH);

    if (meshes.empty())
    {
        ccLog::Error("No mesh in input selection!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }
    else if (meshes.size()>1)
    {
        ccLog::Error("Can't save more than one mesh per MA file!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }

    //we extract the (short) filename from the whole path
	QString baseFilename = QFileInfo(filename).fileName();

    //the mesh to save
    ccGenericMesh* theMesh = ccHObjectCaster::ToGenericMesh(meshes[0]);
    //and its vertices
    ccGenericPointCloud* theCloud = theMesh->getAssociatedCloud();

	unsigned numberOfTriangles = theMesh->size();
	unsigned numberOfVertexes = theCloud->size();

	if (numberOfTriangles==0 || numberOfVertexes==0)
	{
        ccLog::Error("Mesh is empty!");
        return CC_FERR_BAD_ENTITY_TYPE;
	}

	bool hasColors = false;
	if (theCloud->isA(CC_POINT_CLOUD))
	    static_cast<ccPointCloud*>(theCloud)->hasColors();

    //and its scalar field
	/*CCLib::ScalarField* sf = 0;
	if (theCloud->isA(CC_POINT_CLOUD))
	    sf = static_cast<ccPointCloud*>(theCloud)->getCurrentDisplayedScalarField();

    if (!sf)
        ccLog::Warning("No displayed scalar field! Values will all be 0!\n");

	//*/

    //open ASCII file for writing
	FILE* fp = fopen(filename , "wt");

	if (!fp)
        return CC_FERR_WRITING;

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
    unsigned palierModifier = (hasColors ? 1 : 0);
	CCLib::NormalizedProgress nprogress(&pdlg,unsigned(float((2+palierModifier)*numberOfTriangles+(3+palierModifier)*numberOfVertexes)));
	pdlg.setMethodTitle("Save MA file");
	char buffer[256];
    sprintf(buffer,"Triangles = %u",numberOfTriangles);
	pdlg.setInfo(buffer);
	pdlg.start();

	//header
	if (fprintf(fp,"//Maya ASCII 7.0 scene\n") < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (fprintf(fp,"//Name: %s\n",qPrintable(baseFilename)) < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (fprintf(fp,"//Last modified: Sat, Mai 10, 2008 00:00:00 PM\n") < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (fprintf(fp,"requires maya \"4.0\";\n") < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (fprintf(fp,"currentUnit -l %s -a degree -t film;\n","centimeter") < 0)
		{fclose(fp);return CC_FERR_WRITING;}


    //for multiple meshes handling (does not work yet)
    unsigned char currentMesh = 0;

	//NOEUD DE TRANSFORMATION
	if (fprintf(fp,"createNode transform -n \"Mesh%i\";\n",currentMesh+1) < 0)
		{fclose(fp);return CC_FERR_WRITING;}

    //NOEUD "PRINCIPAL"
	if (fprintf(fp,"createNode mesh -n \"MeshShape%i\" -p \"Mesh%i\";\n",currentMesh+1,currentMesh+1) < 0)
		{fclose(fp);return CC_FERR_WRITING;}

	if (fprintf(fp,"\tsetAttr -k off \".v\";\n") < 0)
		{fclose(fp);return CC_FERR_WRITING;}

	if (fprintf(fp,"\tsetAttr \".uvst[0].uvsn\" -type \"string\" \"map1\";\n") < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (fprintf(fp,"\tsetAttr \".cuvs\" -type \"string\" \"map1\";\n") < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (hasColors)
	{
		if (fprintf(fp,"\tsetAttr \".dcol\" yes;\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
	}
	if (fprintf(fp,"\tsetAttr \".dcc\" -type \"string\" \"Ambient+Diffuse\";\n") < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (fprintf(fp,"\tsetAttr \".ccls\" -type \"string\" \"colorSet%i\";\n",currentMesh+1) < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (fprintf(fp,"\tsetAttr \".clst[0].clsn\" -type \"string\" \"colorSet%i\";\n",currentMesh+1) < 0)
		{fclose(fp);return CC_FERR_WRITING;}
	if (hasColors)
	{
		if (fprintf(fp,"\tsetAttr \".ndt\" 0;\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".tgsp\" 1;\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}

		//ON INSERE UN NOEUD "SECONDAIRE"
		if (fprintf(fp,"createNode mesh -n \"polySurfaceShape%i\" -p \"Mesh%i\";\n",currentMesh+1,currentMesh+1) < 0)
			{fclose(fp);return CC_FERR_WRITING;}

		if (fprintf(fp,"\tsetAttr -k off \".v\";\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".io\" yes;\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".uvst[0].uvsn\" -type \"string\" \"map1\";\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".cuvs\" -type \"string\" \"map1\";\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".dcol\" yes;\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".dcc\" -type \"string\" \"Ambient+Diffuse\";\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".ccls\" -type \"string\" \"colorSet%i\";\n",currentMesh+1) < 0)
			{fclose(fp);return CC_FERR_WRITING;}
		if (fprintf(fp,"\tsetAttr \".clst[0].clsn\" -type \"string\" \"colorSet%i\";\n",currentMesh+1) < 0)
			{fclose(fp);return CC_FERR_WRITING;}
	}

	//ecriture des "vertexes"
	if (fprintf(fp,"\tsetAttr -s %u \".vt[0:%u]\"\n",numberOfVertexes,numberOfVertexes-1) < 0)
		{fclose(fp);return CC_FERR_WRITING;}

	const CCVector3d& shift = theCloud->getGlobalShift();
	double scale = theCloud->getGlobalScale();
	assert(scale != 0);
	{
		for (unsigned i=0; i<numberOfVertexes; ++i)
		{
			const CCVector3* P = theCloud->getPoint(i);
			if (fprintf(fp,(i+1==numberOfVertexes ? "\t\t%f %f %f;\n" : "\t\t%f %f %f\n"),
							-shift.x+static_cast<double>(P->x)/scale,
							-shift.z+static_cast<double>(P->z)/scale,
							 shift.y-static_cast<double>(P->y)/scale) < 0)
			{
				fclose(fp);
				return CC_FERR_WRITING;
			}

			nprogress.oneStep();
		}
	}

	//ectitures des "edges"
	//structure permettant la gestion des indexs uniques des edges
	edge** theEdges = new edge*[numberOfVertexes];
	memset(theEdges,0,sizeof(edge*)*numberOfVertexes);
	unsigned ind[3],a,b;
	int currentEdgeIndex=0,lastEdgeIndexPushed=-1;

	int hard=0; //les arr�tes dans Maya peuvent �tre "hard" ou "soft" ...
	{
		theMesh->placeIteratorAtBegining();
		for (unsigned i=0;i<numberOfTriangles;++i)
		{
			const CCLib::TriangleSummitsIndexes* tsi = theMesh->getNextTriangleIndexes(); //DGM: getNextTriangleIndexes is faster for mesh groups!

			ind[0] = tsi->i1;
			ind[1] = tsi->i2;
			ind[2] = tsi->i3;

			uchar k,l;
			for (k=0;k<3;++k)
			{
				l=(k<2 ? k+1 : 0);
				a = (ind[k]<ind[l] ? ind[k] : ind[l]);
				b = (a==ind[k] ? ind[l] : ind[k]);

				currentEdgeIndex = -1;
				edge* e = theEdges[a];
				while (e)
				{
					if (e->theOtherPoint == b)
					{
						currentEdgeIndex = e->edgeIndex;
						break;
					}
					e = e->nextEdge;
				}

				if (currentEdgeIndex<0) //on cree une nouvelle "edge"
				{
					edge* newEdge = new edge;
					newEdge->nextEdge = NULL;
					newEdge->theOtherPoint = b;
					newEdge->positif = (a==ind[k]);
					//newEdge->edgeIndex = ++lastEdgeIndexPushed; //non ! On n'ecrit pas l'arr�te maintenant, donc ce n'est plus vrai
					newEdge->edgeIndex = 0;
					++lastEdgeIndexPushed;
					//currentEdgeIndex = lastEdgeIndexPushed;

					//on doit rajoute le noeud a la fin !!!
					if (theEdges[a])
					{
						e = theEdges[a];
						while (e->nextEdge) e=e->nextEdge;
						e->nextEdge = newEdge;
					}
					else theEdges[a]=newEdge;

					/*if (fprintf(fp,"\n \t\t%i %i %i",a,b,hard) < 0)
						return CC_FERR_WRITING;*/
				}
			}

			nprogress.oneStep();
		}
	}

	//ecriture effective des edges
	{
		unsigned numberOfEdges = unsigned(lastEdgeIndexPushed+1);
		if (fprintf(fp,"\tsetAttr -s %u \".ed[0:%u]\"",numberOfEdges,numberOfEdges-1) < 0)
		{
			fclose(fp);
			return CC_FERR_WRITING;
		}

		lastEdgeIndexPushed = 0;
		for (unsigned i=0; i<numberOfVertexes; ++i)
		{
			edge* e = theEdges[i];
			while (e)
			{
				e->edgeIndex = lastEdgeIndexPushed++;
				if (fprintf(fp,"\n \t\t%u %u %i",i,e->theOtherPoint,hard) < 0)
				{
					fclose(fp);
					return CC_FERR_WRITING;
				}
				e = e->nextEdge;
			}

			nprogress.oneStep();
		}
	}

	if (fprintf(fp,";\n") < 0)
	{
		fclose(fp);
		return CC_FERR_WRITING;
	}

	//ectitures des "faces"
	if (fprintf(fp,"\tsetAttr -s %u \".fc[0:%u]\" -type \"polyFaces\"\n",numberOfTriangles,numberOfTriangles-1) < 0)
	{
		fclose(fp);
		return CC_FERR_WRITING;
	}

	theMesh->placeIteratorAtBegining();
	{
		for (unsigned i=0; i<numberOfTriangles; ++i)
		{
			if (fprintf(fp,"\t\tf 3") < 0)
			{
				fclose(fp);
				return CC_FERR_WRITING;
			}

			CCLib::TriangleSummitsIndexes* tsi = theMesh->getNextTriangleIndexes(); //DGM: getNextTriangleIndexes is faster for mesh groups!
			ind[0] = tsi->i1;
			ind[1] = tsi->i2;
			ind[2] = tsi->i3;

			for (uchar k=0; k<3; ++k)
			{
				uchar l = (k<2 ? k+1 : 0);
				a = (ind[k]<ind[l] ? ind[k] : ind[l]);
				b = (a==ind[k] ? ind[l] : ind[k]);

				edge* e = theEdges[a];
				while (e->theOtherPoint != b)
					e = e->nextEdge;

				if (fprintf(fp," %i",((e->positif && a==ind[k]) || (!e->positif && a==ind[l]) ? e->edgeIndex : -(e->edgeIndex+1))) < 0)
				{
					fclose(fp);
					return CC_FERR_WRITING;
				}
			}

			if (fprintf(fp,(i+1==numberOfTriangles ? ";\n" : "\n")) < 0)
			{
				fclose(fp);
				return CC_FERR_WRITING;
			}

			nprogress.oneStep();
		}
	}

	//on libere la memoire
	{
		for (unsigned i=0; i<numberOfVertexes; ++i)
		{
			if (theEdges[i])
			{
				edge* e = theEdges[i]->nextEdge;
				while (e)
				{
					edge* nextE = e->nextEdge;
					delete e;
					e = nextE;
				}
				delete theEdges[i];
			}

			nprogress.oneStep();
		}
	}

	//cadeaux bonux 2
	if (	fprintf(fp,"\tsetAttr \".cd\" -type \"dataPolyComponent\" Index_Data Edge 0 ;\n") < 0
		||	fprintf(fp,"\tsetAttr \".ndt\" 0;\n") < 0
		||	fprintf(fp,"\tsetAttr \".tgsp\" 1;\n") < 0 )
	{
		fclose(fp);
		return CC_FERR_WRITING;
	}

	//NOEUD DES VERTEX COLORS
	if (hasColors)
	{
	    assert(theCloud->isA(CC_POINT_CLOUD));
        ccPointCloud* pc = static_cast<ccPointCloud*>(theCloud);

		if (fprintf(fp,"createNode polyColorPerVertex -n \"polyColorPerVertex%i\";\n",currentMesh+1) < 0)
			{fclose(fp);return CC_FERR_WRITING;}

		if (fprintf(fp,"\tsetAttr \".uopa\" yes;\n") < 0)
			{fclose(fp);return CC_FERR_WRITING;}

		if (fprintf(fp,"\tsetAttr -s %u \".vclr\";\n",numberOfVertexes) < 0)
			{fclose(fp);return CC_FERR_WRITING;}

		//on construit une structure qui associe chaque vertex aux faces auxquelles elle appartient
		faceIndexes** theFacesIndexes = new faceIndexes*[numberOfVertexes];
		memset(theFacesIndexes,0,sizeof(faceIndexes*)*numberOfVertexes);
		theMesh->placeIteratorAtBegining();
		{
			for (unsigned i=0; i<numberOfTriangles; ++i)
			{
				CCLib::TriangleSummitsIndexes* tsi = theMesh->getNextTriangleIndexes(); //DGM: getNextTriangleIndexes is faster for mesh groups!
				ind[0] = tsi->i1;
				ind[1] = tsi->i2;
				ind[2] = tsi->i3;

				for (uchar j=0; j<3; ++j)
				{
					if (!theFacesIndexes[ind[j]])
					{
						faceIndexes* f = new faceIndexes;
						f->faceIndex = i;
						f->nextFace = NULL;
						theFacesIndexes[ind[j]] = f;
					}
					else
					{
						faceIndexes* f = theFacesIndexes[ind[j]];
						while (f->nextFace)
							f = f->nextFace;
						f->nextFace = new faceIndexes;
						f->nextFace->faceIndex = i;
						f->nextFace->nextFace = NULL;
					}
				}

				nprogress.oneStep();
			}
		}

		//pour chaque vertex
		{
			float col[3];
			float coef = 1.0f/static_cast<float>(MAX_COLOR_COMP);
			for (unsigned i=0; i<numberOfVertexes; ++i)
			{
				const colorType* c = pc->getPointColor(i);
				col[0] = static_cast<float>(c[0]) * coef;
				col[1] = static_cast<float>(c[1]) * coef;
				col[2] = static_cast<float>(c[2]) * coef;

				//on compte le nombre de faces
				int nf = 0;
				faceIndexes* f = theFacesIndexes[i];
				while (f)
				{
					++nf;
					f = f->nextFace;
				}

				if (nf > 0)
				{
					if (fprintf(fp,"\tsetAttr -s %i \".vclr[%u].vfcl\";\n",nf,i) < 0)
						{fclose(fp);return CC_FERR_WRITING;}

					faceIndexes *oldf, *f = theFacesIndexes[i];
					while (f)
					{
						if (fprintf(fp,"\tsetAttr \".vclr[%u].vfcl[%i].frgb\" -type \"float3\" %f %f %f;\n",i,f->faceIndex,col[0],col[1],col[2]) < 0)
						{
							fclose(fp);
							return CC_FERR_WRITING;
						}

						oldf = f;
						f = f->nextFace;
						delete oldf;
					}
					theFacesIndexes[i] = NULL;
				}

				nprogress.oneStep();
			}
		}
		delete[] theFacesIndexes;

		if (fprintf(fp,"\tsetAttr \".cn\" -type \"string\" \"colorSet%i\";\n",currentMesh+1) < 0)
		{
			fclose(fp);
			return CC_FERR_WRITING;
		}
	}

	//Maya connections
	if (hasColors)
	{
		if (	fprintf(fp,"connectAttr \"polyColorPerVertex%i.out\" \"MeshShape%i.i\";\n",currentMesh+1,currentMesh+1) < 0
			||	fprintf(fp,"connectAttr \"polySurfaceShape%i.o\" \"polyColorPerVertex%i.ip\";\n",currentMesh+1,currentMesh+1) < 0 )
		{
			fclose(fp);
			return CC_FERR_WRITING;
		}
	}
	
	if (fprintf(fp,"connectAttr \"MeshShape%i.iog\" \":initialShadingGroup.dsm\" -na;\n",currentMesh+1) < 0)
	{
		fclose(fp);
		return CC_FERR_WRITING;
	}

	//fin du fichier
	if (fprintf(fp,"//End of %s\n",qPrintable(baseFilename)) < 0)
	{
		fclose(fp);
		return CC_FERR_WRITING;
	}

	fclose(fp);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR MAFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
    ccLog::Error("Not available yet!");

	return CC_FERR_NO_ERROR;
}
