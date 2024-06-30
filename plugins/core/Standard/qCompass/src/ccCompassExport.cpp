//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#include <QBuffer>
#include <QFileInfo>
#include <QXmlStreamWriter>

#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>

#include <ccFacet.h>
#include <ccPointCloud.h>

#include "ccCompassExport.h"
#include "ccFitPlane.h"
#include "ccGeoObject.h"
#include "ccLineation.h"
#include "ccSNECloud.h"
#include "ccThickness.h"
#include "ccTrace.h"

static int WritePlanes(ccHObject* rootObject, ccHObject* object, QTextStream& out, const QString& parentName = QString())
{
	//get object name
	QString name;
	if (parentName.isEmpty())
	{
		name = QStringLiteral("%1").arg(object->getName());
	}
	else
	{
		name = QStringLiteral("%1.%2").arg(parentName, object->getName());
	}

	//find point cloud (biggest in project) to pull global shift & scale from
	//n.b. ccPlanes do not store a global shift/scale like point clouds do, hence this hack.
	//Will only cause issues if CC is being used with multiple point clouds that have different underlying
	//coordinate systems (and hence shift and scales) - which I can't see happening too often.
	//(in any case, 99% of the time the biggest point cloud in the project will be the model being interpreted)
	ccPointCloud* ss = nullptr;
	if (object->isKindOf(CC_TYPES::PLANE) | ccFitPlane::isFitPlane(object))
	{
		std::vector<ccHObject*> clouds;
		rootObject->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, false);
		unsigned int npoints = 0;
		for (ccHObject* o : clouds)
		{
			ccPointCloud* p = static_cast<ccPointCloud*>(o);
			if (npoints <= p->size())
			{
				npoints = p->size();
				ss = p;
			}
		}
	}

	//is object a plane made by ccCompass?
	int n = 0;
	if (ccFitPlane::isFitPlane(object))
	{
		//write global position
		ccPlane* P = static_cast<ccPlane*>(object);

		//Write object as Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Radius,RMS,Gx,Gy,Gz,Length
		out << name << ",";
		out << object->getMetaData("Strike").toString() << "," << object->getMetaData("Dip").toString() << "," << object->getMetaData("DipDir").toString() << ",";
		out << object->getMetaData("Cx").toString() << "," << object->getMetaData("Cy").toString() << "," << object->getMetaData("Cz").toString() << ",";
		out << object->getMetaData("Nx").toString() << "," << object->getMetaData("Ny").toString() << "," << object->getMetaData("Nz").toString() << ",";
		out << object->getMetaData("Radius").toString() << "," << object->getMetaData("RMS").toString() << ",";

		if (ss != nullptr)
		{
			CCVector3 L = P->getTransformation().getTranslationAsVec3D();
			CCVector3d G = ss->toGlobal3d(L);

			out << G.x << "," << G.y << "," << G.z << ",";
		}

		//write length of trace associated with this plane
		out << std::max(P->getXWidth(), P->getYWidth()) << endl;
		++n;
	}
	else if (object->isKindOf(CC_TYPES::PLANE)) //not one of our planes, but a plane anyway (so we'll export it)
	{
		//calculate plane orientation
		//get plane normal vector
		ccPlane* P = static_cast<ccPlane*>(object);
		CCVector3 N(P->getNormal());
		CCVector3 L = P->getTransformation().getTranslationAsVec3D();

		//We always consider the normal with a positive 'Z' by default!
		if (N.z < 0.0)
			N *= -1.0;

		//calculate strike/dip/dip direction
		float strike = 0.0f;
		float dip = 0.0f;
		float dipdir = 0.0f;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipdir);
		ccNormalVectors::ConvertNormalToStrikeAndDip(N, strike, dip);

		//export
		out << name << ",";
		out << strike << "," << dip << "," << dipdir << ","; //write orientation
		out << L.x << "," << L.y << "," << L.z << ","; //write location
		out << N.x << "," << N.y << "," << N.z << ","; //write normal
		out << "NA" << "," << "UNK" << ","; //the "radius" and "RMS" are unknown

		//write global position
		if (ss != nullptr)
		{
			CCVector3d G = ss->toGlobal3d(L);

			out << G.x << "," << G.y << "," << G.z;
		}

		out << endl;

		++n;
	}

	//write all children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n += WritePlanes(rootObject, o, out, name);
	}

	return n;
}

static int WriteTraces(ccHObject* object, QTextStream& out, const QString& parentName = QString())
{
	//get object name
	QString name;
	if (parentName.isEmpty())
	{
		name = QStringLiteral("%1").arg(object->getName());
	}
	else
	{
		name = QStringLiteral("%1.%2").arg(parentName, object->getName());
	}

	//is object a polyline
	int n = 0;
	if (ccTrace::isTrace(object)) //ensure this is a trace
	{
		ccTrace* p = static_cast<ccTrace*>(object);

		//loop through points
		CCVector3 start;
		CCVector3 end;
		int cost;
		int tID = object->getUniqueID();
		if (p->size() >= 2)
		{
			//set cost function
			ccTrace::COST_MODE = p->getMetaData("cost_function").toInt();

			//loop through segments
			for (unsigned i = 1; i < p->size(); i++)
			{
				//get points
				p->getPoint(i - 1, start);
				p->getPoint(i, end);

				//calculate segment cost
				cost = p->getSegmentCost(p->getPointGlobalIndex(i - 1), p->getPointGlobalIndex(i));

				//write data
				//n.b. csv columns are name,trace_id,seg_id,start_x,start_y,start_z,end_x,end_y,end_z, cost, cost_mode
				out << name << ","; //name
				out << tID << ",";
				out << i - 1 << ",";

				CCVector3d startGlobal = p->toGlobal3d(start);
				out << startGlobal.x << ",";
				out << startGlobal.y << ",";
				out << startGlobal.z << ",";
				CCVector3d endGlobal = p->toGlobal3d(end);
				out << endGlobal.x << ",";
				out << endGlobal.y << ",";
				out << endGlobal.z << ",";

				out << cost << ",";
				out << ccTrace::COST_MODE << endl;
			}
		}
		++n;
	}

	//write all children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n += WriteTraces(o, out, name);
	}

	return n;
}

static int WriteLineations(ccHObject* object, QTextStream& out, const QString& parentName = QString(), bool thicknesses = false)
{
	//get object name
	QString name;
	if (parentName.isEmpty())
	{
		name = QStringLiteral("%1").arg(object->getName());
	}
	else
	{
		name = QStringLiteral("%1.%2").arg(parentName, object->getName());
	}

	//is object a lineation made by ccCompass?
	int n = 0;
	if (((thicknesses == false) && ccLineation::isLineation(object)) | //lineation measurement
		((thicknesses == true) && ccThickness::isThickness(object)))    //or thickness measurement
	{
		//Write object as Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge
		out << name << ",";
		out << object->getMetaData("Sx").toString() << "," << object->getMetaData("Sy").toString() << "," << object->getMetaData("Sz").toString() << ",";
		out << object->getMetaData("Ex").toString() << "," << object->getMetaData("Ey").toString() << "," << object->getMetaData("Ez").toString() << ",";
		out << object->getMetaData("Trend").toString() << "," << object->getMetaData("Plunge").toString() << "," << object->getMetaData("Length").toString() << endl;
		++n;
	}

	//write all children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n += WriteLineations(o, out, name, thicknesses);
	}

	return n;
}
	
int WriteTracesSVG(const ccGLCameraParameters& cameraParams, ccHObject* object, QTextStream& out, int height, float zoom)
{
	int n = 0;

	//is this a drawable polyline?
	if (object->isA(CC_TYPES::POLY_LINE) || ccTrace::isTrace(object))
	{
		//get polyline object
		ccPolyline* line = static_cast<ccPolyline*>(object);

		if (!line->isVisible())
		{
			return 0; //as soon as something is not visible we bail
		}

		//write polyline header
		out << "<polyline fill=\"none\" stroke=\"black\" points=\"";

		//write point string
		for (unsigned i = 0; i < line->size(); i++)
		{
			//get point in world coordinates
			CCVector3 P = *line->getPoint(i);

			//project 3D point into 2D
			CCVector3d coords2D;
			cameraParams.project(P, coords2D);

			//write point
			out << QString::asprintf("%.3f,%.3f ", coords2D.x*zoom, height - (coords2D.y*zoom)); //n.b. we need to flip y-axis
		}

		//end polyline
		out << "\"/>" << endl;

		++n; //a polyline has been written
	}

	//recurse on children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		n += WriteTracesSVG(cameraParams, object->getChild(i), out, height, zoom);
	}

	return n;
}

//recursively write the provided ccHObject and its children
int WriteObjectXML(ccHObject* object, QXmlStreamWriter& out)
{
	int n = 1;
	//write object header based on type
	if (ccGeoObject::isGeoObject(object))
	{
		//write GeoObject
		out.writeStartElement("GEO_OBJECT");
	}
	else if (object->isA(CC_TYPES::PLANE))
	{
		//write fitPlane
		out.writeStartElement("PLANE");
	}
	else if (ccTrace::isTrace(object))
	{
		//write trace
		out.writeStartElement("TRACE");
	}
	else if (ccThickness::isThickness(object))
	{
		//write thickness
		out.writeStartElement("THICKNESS");
	}
	else if (ccSNECloud::isSNECloud(object))
	{
		out.writeStartElement("SNE");
	}
	else if (ccLineation::isLineation(object))
	{
		//write lineation
		out.writeStartElement("LINEATION");
	}
	else if (object->isA(CC_TYPES::POINT_CLOUD))
	{
		out.writeStartElement("CLOUD");
	}
	else if (object->isA(CC_TYPES::POLY_LINE))
	{
		//write polyline (note that this will ignore "trace" polylines as they have been grabbed earlier)
		out.writeStartElement("POLYLINE");
	}
	else if (object->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//write container
		out.writeStartElement("CONTAINER"); //QString::asprintf("CONTAINER name = '%s' id = %d", object->getName(), object->getUniqueID())
	}
	else if (object->isA(CC_TYPES::FACET))
	{
		//write container
		out.writeStartElement("FACET");
	}
	else //just write name,id and metadata for unknown objects
	{
		out.writeStartElement("OBJECT");
	}

	//write name and oid attributes
	out.writeAttribute("name", object->getName());
	out.writeAttribute("id", QString::asprintf("%d", object->getUniqueID()));

	//write metadata tags (these contain the data)
	const auto &metaData = object->metaData();

	for (const auto &key : metaData.keys())
	{
		out.writeTextElement(key, metaData.value(key).toString());
	}

	//special case - we can calculate all metadata from a plane
	if (object->isA(CC_TYPES::PLANE))
	{
		ccPlane* P = static_cast<ccPlane*> (object);

		//write length
		out.writeTextElement("Length", QString::asprintf("%f", std::max(P->getXWidth(), P->getYWidth())));

		//if this is just an ordinary plane, make a corresponding fitplane object and then steal metadata
		if (!ccFitPlane::isFitPlane(P))
		{
			//build fitplane object
			ccFitPlane* temp = new ccFitPlane(P);

			//write metadata
			const auto &tempMetaData = temp->metaData();

			for (const auto &key : tempMetaData.keys())
			{
				out.writeTextElement(key, tempMetaData.value(key).toString());
			}

			//cleanup
			delete temp;
		}
	}

	//if object is a polyline object (or a trace) write trace points and normals
	if (object->isA(CC_TYPES::POLY_LINE))
	{
		ccPolyline* poly = static_cast<ccPolyline*>(object);
		ccTrace* trace = nullptr;
		if (ccTrace::isTrace(object))
		{
			trace = static_cast<ccTrace*>(object);
		}

		QString x;
		QString y;
		QString z;
		QString nx;
		QString ny;
		QString nz;
		QString cost;
		QString wIDs;
		QString w_local_ids;

		//loop through points
		//position
		CCVector3 p1;
		CCVector3 p2;

		//normal vector (if defined)
		CCVector3 n1;
		CCVector3 n2;

		//becomes true if any valid normals are recieved
		bool hasNormals = false;

		if (poly->size() >= 2)
		{
			//loop through segments
			for (unsigned i = 1; i < poly->size(); i++)
			{
				//get points
				poly->getPoint(i - 1, p1); //segment start point
				poly->getPoint(i, p2); //segment end point

				//store data to buffers
				x += QString::asprintf("%f,", p1.x);
				y += QString::asprintf("%f,", p1.y);
				z += QString::asprintf("%f,", p1.z);

				//write data specific to traces
				if (trace)
				{
					int c = trace->getSegmentCost(trace->getPointGlobalIndex(i - 1), trace->getPointGlobalIndex(i));
					cost += QString::asprintf("%d,", c);

					//write point normals (if this is a trace)
					n2 = trace->getPointNormal(i);
					nx += QString::asprintf("%f,", n1.x);
					ny += QString::asprintf("%f,", n1.y);
					nz += QString::asprintf("%f,", n1.z);
					if (!hasNormals && !(n1.x == 0 && n1.y == 0 && n1.z == 0))
					{
						hasNormals = true; //this was a non-null normal estimate - we will write normals now
					}
				}

			}

			//store last point
			x += QString::asprintf("%f", p2.x);
			y += QString::asprintf("%f", p2.y);
			z += QString::asprintf("%f", p2.z);
			if (hasNormals) //normal
			{
				nx += QString::asprintf("%f", n2.x);
				ny += QString::asprintf("%f", n2.y);
				nz += QString::asprintf("%f", n2.z);
			}
			if (trace) //cost
			{
				cost += QStringLiteral("0");
			}

			//if this is a trace also write the waypoints
			if (trace)
			{
				//get ids (on the cloud) for waypoints
				for (int w = 0; w < trace->waypoint_count(); w++)
				{
					wIDs += QString::asprintf("%d,", trace->getWaypoint(w));
				}

				//get ids (vertex # in polyline) for waypoints
				for (int w = 0; w < trace->waypoint_count(); w++)
				{
					//get id of waypoint in cloud
					int globalID = trace->getWaypoint(w);

					//find corresponding point in trace
					unsigned i = 0;
					for (; i < trace->size(); i++)
					{
						if (trace->getPointGlobalIndex(i) == globalID)
						{
							break; //found it!;
						}
					}

					//write this points local index
					w_local_ids += QString::asprintf("%d,", i);
				}
			}

			//write points
			out.writeStartElement("POINTS");
			out.writeAttribute("count", QString::asprintf("%d", poly->size()));

			if (hasNormals)
			{
				out.writeAttribute("normals", "True");
			}
			else
			{
				out.writeAttribute("normals", "False");
			}

			out.writeTextElement("x", x);
			out.writeTextElement("y", y);
			out.writeTextElement("z", z);

			if (hasNormals)
			{
				out.writeTextElement("nx", nx);
				out.writeTextElement("ny", ny);
				out.writeTextElement("nz", nz);
			}

			if (trace)
			{
				//write waypoints
				out.writeTextElement("cost", cost);
				out.writeTextElement("control_point_cloud_ids", wIDs);
				out.writeTextElement("control_point_local_ids", w_local_ids);
			}

			//fin!
			out.writeEndElement();
		}
	}

	//if object is a point cloud write global shift and scale
	if (object->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* cloud = static_cast<ccPointCloud*>(object);
		out.writeTextElement("GLOBAL_SCALE", QString::asprintf("%f", cloud->getGlobalScale()));
		out.writeTextElement("GLOBAL_X", QString::asprintf("%f", cloud->getGlobalShift().x));
		out.writeTextElement("GLOBAL_Y", QString::asprintf("%f", cloud->getGlobalShift().y));
		out.writeTextElement("GLOBAL_Z", QString::asprintf("%f", cloud->getGlobalShift().z));

		//for SNE clouds write all points, point normals and scalar fields
		if (ccSNECloud::isSNECloud(object))
		{
			//write header for point data
			out.writeStartElement("POINTS");
			out.writeAttribute("count", QString::asprintf("%d", cloud->size()));

			//gather data strings
			QString x;
			QString y;
			QString z;
			QString nx;
			QString ny;
			QString nz;
			QString thickness;
			QString weight;
			QString trend;
			QString plunge;
			CCCoreLib::ScalarField* wSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Weight"));
			CCCoreLib::ScalarField* trendSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Trend"));
			CCCoreLib::ScalarField* plungeSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Plunge"));

			CCCoreLib::ScalarField* tSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Thickness"));
			for (unsigned p = 0; p < cloud->size(); p++)
			{
				x += QString::asprintf("%f,", cloud->getPoint(p)->x);
				y += QString::asprintf("%f,", cloud->getPoint(p)->y);
				z += QString::asprintf("%f,", cloud->getPoint(p)->z);
				nx += QString::asprintf("%f,", cloud->getPointNormal(p).x);
				ny += QString::asprintf("%f,", cloud->getPointNormal(p).y);
				nz += QString::asprintf("%f,", cloud->getPointNormal(p).z);
				weight += QString::asprintf("%f,", wSF->getValue(p));
				trend += QString::asprintf("%f,", trendSF->getValue(p));
				plunge += QString::asprintf("%f,", plungeSF->getValue(p));

				if (tSF != nullptr) //can be null if no thickness was estimated!
				{
					thickness += QString::asprintf("%f,", tSF->getValue(p));
				}
			}

			//write
			out.writeTextElement("x", x);
			out.writeTextElement("y", y);
			out.writeTextElement("z", z);
			out.writeTextElement("nx", nx);
			out.writeTextElement("ny", ny);
			out.writeTextElement("nz", nz);
			out.writeTextElement("weight", weight);
			out.writeTextElement("trend", trend);
			out.writeTextElement("plunge", plunge);
			if (tSF != nullptr)
			{
				out.writeTextElement("thickness", thickness);
			}

			//fin
			out.writeEndElement();
		}
	}

	//write facet data
	if (object->isA(CC_TYPES::FACET))
	{
		//write orientation
		ccFacet* f = static_cast<ccFacet*>(object);
		out.writeTextElement("Nx", QString::asprintf("%f", f->getNormal().x));
		out.writeTextElement("Ny", QString::asprintf("%f", f->getNormal().y));
		out.writeTextElement("Nz", QString::asprintf("%f", f->getNormal().z));
		out.writeTextElement("Cx", QString::asprintf("%f", f->getCenter().x));
		out.writeTextElement("Cy", QString::asprintf("%f", f->getCenter().y));
		out.writeTextElement("Cz", QString::asprintf("%f", f->getCenter().z));
		out.writeTextElement("rms", QString::asprintf("%f", f->getRMS()));
		out.writeTextElement("surface", QString::asprintf("%f", f->getSurface()));
	}

	//write children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		n += WriteObjectXML(object->getChild(i), out);
	}

	//close this object
	out.writeEndElement();

	return n;
}

void ccCompassExport::SaveCSV(ccMainAppInterface* app, const QString& filename)
{
	// write a whole bunch of .csv files

	int planes = 0; //keep track of how many objects are being written (used to delete empty files)
	int traces = 0;
	int lineations = 0;
	int thicknesses = 0;

	//build filenames
	QFileInfo fi(filename);

	QString baseName = fi.absolutePath() + "/" + fi.completeBaseName();
	QString ext = fi.suffix();
	if (!ext.isEmpty())
	{
		ext.prepend('.');
	}
	QString plane_fn = baseName + QStringLiteral("_planes") + ext;
	QString trace_fn = baseName + QStringLiteral("_traces") + ext;
	QString lineation_fn = baseName + QStringLiteral("_lineations") + ext;
	QString thickness_fn = baseName + QStringLiteral("_thickness") + ext;

	//create files
	QFile plane_file(plane_fn);
	QFile trace_file(trace_fn);
	QFile lineation_file(lineation_fn);
	QFile thickness_file(thickness_fn);

	//open files
	if (plane_file.open(QIODevice::WriteOnly) && trace_file.open(QIODevice::WriteOnly) && lineation_file.open(QIODevice::WriteOnly) && thickness_file.open(QIODevice::WriteOnly))
	{
		//create text streams for each file
		QTextStream plane_stream(&plane_file);
		QTextStream trace_stream(&trace_file);
		QTextStream lineation_stream(&lineation_file);
		QTextStream thickness_stream(&thickness_file);

		plane_stream.setRealNumberNotation(QTextStream::FixedNotation);
		plane_stream.setRealNumberPrecision(12);
		trace_stream.setRealNumberNotation(QTextStream::FixedNotation);
		trace_stream.setRealNumberPrecision(12);
		lineation_stream.setRealNumberNotation(QTextStream::FixedNotation);
		lineation_stream.setRealNumberPrecision(12);
		thickness_stream.setRealNumberNotation(QTextStream::FixedNotation);
		thickness_stream.setRealNumberPrecision(12);

		//write headers
		plane_stream << "Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Sample_Radius,RMS,Gx,Gy,Gz,Length" << endl;
		trace_stream << "Name,Trace_id,Point_id,Start_x,Start_y,Start_z,End_x,End_y,End_z,Cost,Cost_Mode" << endl;
		lineation_stream << "Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge,Length" << endl;
		thickness_stream << "Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge,Thickness" << endl;

		//write data for all objects in the db tree (n.b. we loop through the dbRoots children rathern than just passing db_root so the naming is correct)
		for (unsigned i = 0; i < app->dbRootObject()->getChildrenNumber(); i++)
		{
			ccHObject* rootObject = app->dbRootObject();
			ccHObject* o = rootObject->getChild(i);

			planes += WritePlanes(rootObject, o, plane_stream);
			traces += WriteTraces(o, trace_stream);
			lineations += WriteLineations(o, lineation_stream, QString(), false);
			thicknesses += WriteLineations(o, thickness_stream, QString(), true);
		}

		//cleanup
		plane_stream.flush();
		plane_file.close();
		trace_stream.flush();
		trace_file.close();
		lineation_stream.flush();
		lineation_file.close();
		thickness_stream.flush();
		thickness_file.close();

		//ensure data has been written (and if not, delete the file)
		if (planes)
		{
			app->dispToConsole("[ccCompass] Successfully exported plane data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			app->dispToConsole("[ccCompass] No plane data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			plane_file.remove();
		}
		if (traces)
		{
			app->dispToConsole("[ccCompass] Successfully exported trace data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			app->dispToConsole("[ccCompass] No trace data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			trace_file.remove();
		}
		if (lineations)
		{
			app->dispToConsole("[ccCompass] Successfully exported lineation data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			app->dispToConsole("[ccCompass] No lineation data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			lineation_file.remove();
		}
		if (thicknesses)
		{
			app->dispToConsole("[ccCompass] Successfully exported thickness data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			app->dispToConsole("[ccCompass] No thickness data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			thickness_file.remove();
		}
	}
	else
	{
		app->dispToConsole("[ccCompass] Could not open output files... ensure CC has write access to this location.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
}
	
void ccCompassExport::SaveSVG(ccMainAppInterface *app, const QString &filename, float zoom)
{
	//set all objects except the point clouds invisible
	std::vector<ccHObject*> hidden; //store objects we hide so we can turn them back on after!
	ccHObject::Container objects;
	app->dbRootObject()->filterChildren(objects, true, CC_TYPES::OBJECT, false); //get list of all children!
	for (ccHObject* o : objects)
	{
		if (!o->isA(CC_TYPES::POINT_CLOUD))
		{
			if (o->isVisible())
			{
				hidden.push_back(o);
				o->setVisible(false);
			}
		}
	}

	//render the scene
	QImage img = app->getActiveGLWindow()->renderToImage(zoom);

	//restore visibility
	for (ccHObject* o : hidden)
	{
		o->setVisible(true);
	}

	//convert image to base64 (png format) to write in svg file
	QByteArray ba;
	QBuffer bu(&ba);
	bu.open(QIODevice::WriteOnly);
	img.save(&bu, "PNG");
	bu.close();

	//create .svg file
	QFile svg_file(filename);
	//open file & create text stream
	if (svg_file.open(QIODevice::WriteOnly))
	{
		QTextStream svg_stream(&svg_file);
		svg_stream.setRealNumberNotation(QTextStream::FixedNotation);
		svg_stream.setRealNumberPrecision(12);

		int width = std::abs(static_cast<int>(app->getActiveGLWindow()->glWidth()  * zoom)); //glWidth and glHeight are negative on some machines??
		int height = std::abs(static_cast<int>(app->getActiveGLWindow()->glHeight() * zoom));

		//write svg header
		svg_stream << QString::asprintf("<svg width=\"%d\" height=\"%d\">", width, height) << endl;

		//write the image
		svg_stream << QString::asprintf("<image height = \"%d\" width = \"%d\" xlink:href = \"data:image/png;base64,", height, width) << ba.toBase64() << "\"/>" << endl;

		//recursively write traces
		ccGLCameraParameters params;
		app->getActiveGLWindow()->getGLCameraParameters(params);
		if (params.perspective)
		{
			app->getActiveGLWindow()->setPerspectiveState(false, true);
			//m_app->getActiveGLWindow()->redraw(false, false); //not sure if this is needed or not?
			app->getActiveGLWindow()->getGLCameraParameters(params); //get updated params
		}

		int count = WriteTracesSVG(params, app->dbRootObject(), svg_stream, height, zoom);

		//TODO: write scale bar

		//write end tag for svg file
		svg_stream << "</svg>" << endl;

		//close file
		svg_stream.flush();
		svg_file.close();

		if (count > 0)
		{
			app->dispToConsole(QString::asprintf("[ccCompass] Successfully saved %d polylines to .svg file.", count));
		}
		else
		{
			//remove file
			svg_file.remove();
			app->dispToConsole("[ccCompass] Could not write polylines to .svg - no polylines found!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}
}
	
void ccCompassExport::SaveXML(ccMainAppInterface *app, const QString &filename)
{
	//find root node
	ccHObject* rootObject = app->dbRootObject();
	if (rootObject && rootObject->getChildrenNumber() == 1) //HACK - often the root only has one child (a .bin file); if so, move down a level
	{
		rootObject = rootObject->getChild(0);
	}
	if (!rootObject)
	{
		assert(false);
		app->dispToConsole("[ccCompass] Internal error: failed to find the root object", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return;
	}

	//open output stream
	QFile file(filename);

	if (file.open(QIODevice::WriteOnly)) //open the file
	{
		QXmlStreamWriter xmlWriter(&file); //open xml stream;

		xmlWriter.setAutoFormatting(true);
		xmlWriter.writeStartDocument();

		/*ccHObject::Container pointClouds;
		rootObject->filterChildren(&pointClouds, true, CC_TYPES::POINT_CLOUD, true);*/

		//write data tree
		WriteObjectXML(rootObject, xmlWriter);

		//write end of document
		xmlWriter.writeEndDocument();

		//close
		file.flush();
		file.close();

		app->dispToConsole("[ccCompass] Successfully exported data-tree to xml.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
	else
	{
		app->dispToConsole("[ccCompass] Could not open output files... ensure CC has write access to this location.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
}
