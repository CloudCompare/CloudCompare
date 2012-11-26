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
//$Rev:: 2059                                                              $
//$LastChangedDate:: 2012-03-29 19:11:56 +0200 (jeu., 29 mars 2012)        $
//**************************************************************************
//
#ifdef CC_ULT_SUPPORT

#include "UltFilter.h"

#include "ccConsole.h"

//Qt
#include <QFileInfo>
#include <QApplication>
#include <QMessageBox>

//CCLib
#include <ScalarField.h>
#include <CCMiscTools.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccGLMatrix.h>

#include <assert.h>

//! Marker roles
enum MARKER_ROLE {	MARKER_PROBE			= 0,	//don't mess with the order!
					MARKER_FEMUR			= 1,
					MARKER_TIBIA			= 2,
					MARKER_POINTER			= 3,
					MARKER_PLANE_PHANTOM	= 4,
					MARKER_CYLINDER_PHANTOM	= 5,
					MARKER_CAMERA			= 6,
					MARKER_CHESSBOARD		= 7,
					MARKER_LOCALIZER		= 8,	//should always be the last "real" marker
					MARKER_UNDEFINED		= 9,	//should always be just after MARKER_LOCALIZER
					MARKER_DEFAULT_REF		= 10,	//should always be just after MARKER_DEFAULT_REF
};

//! Marker count (should be equal to 'MARKER_LOCALIZER')
#define MARKER_ROLES_COUNT MARKER_LOCALIZER

//! Fake UltrasonixLocalizer Trans3D
class Trans3D : public ccGLMatrix
{
public:

	Trans3D(double time=0) : timestamp(time) {}

	//! timestamp (µs)
	double timestamp;
};

//! Marker UltrasonixLocalizer state (position + visibility)
struct MarkerState
{
public:
	//! position
	Trans3D pos;

	//! visibility
	bool visible;

	//! Default constructor
	MarkerState()
		: pos(0.0)
		, visible(false)
	{
	}
};

//! Markers frame
/** All markers positions and states at the same time.
**/
struct MarkersFrame
{
    MarkerState states[MARKER_ROLES_COUNT];
};


CC_FILE_ERROR UltFilter::saveToFile(ccHObject* entity, const char* filename)
{
	if (!entity || !filename)
        return CC_FERR_BAD_ARGUMENT;

	ccHObject::Container clouds;
	if (entity->isKindOf(CC_POINT_CLOUD))
        clouds.push_back(entity);
    else
        entity->filterChildren(clouds, true, CC_POINT_CLOUD);

    if (clouds.empty())
    {
        ccConsole::Error("No point cloud in input selection!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }
    else if (clouds.size()>1)
    {
        ccConsole::Error("Can't save more than one cloud per ULD file!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }

	//which marker is the reference?
	QMessageBox::StandardButton tibiaIsRef = QMessageBox::question(0, "Choose reference", "Tibia as reference (yes)? Or femur (no)? Or none (no to all)", QMessageBox::Yes | QMessageBox::No | QMessageBox::NoToAll, QMessageBox::Yes );
	MARKER_ROLE referenceRole = MARKER_LOCALIZER;
	if (tibiaIsRef == QMessageBox::Yes)
		referenceRole = MARKER_TIBIA;
	else if (tibiaIsRef == QMessageBox::No)
		referenceRole = MARKER_FEMUR;

	//the cloud to save
    ccGenericPointCloud* theCloud = static_cast<ccGenericPointCloud*>(clouds[0]);

    //open binary file for writing
	FILE* fp = fopen(qPrintable(QString(filename).replace(".ult",".uld",Qt::CaseInsensitive)), "wb");
	if (!fp)
        return CC_FERR_WRITING;

	//TYPE
	unsigned type = 64; //DATA_POINT_CLOUD
	fwrite(&type, sizeof(unsigned), 1, fp);

	//Data version
	short dataVersion = 44;
	fwrite(&dataVersion, sizeof(short), 1, fp);

	//Cloud size
	unsigned count = theCloud->size();
	fwrite(&count, sizeof(unsigned), 1, fp);

	//Localization context
	bool hasLocContext = true;
	fwrite(&hasLocContext, sizeof(bool), 1, fp);

	if (hasLocContext)
	{
		//Producer role
		MARKER_ROLE producerRole = MARKER_UNDEFINED;
		fwrite(&producerRole, sizeof(MARKER_ROLE), 1, fp);

		//Producer transformation
		bool prodHasTrans = true;
		fwrite(&prodHasTrans, sizeof(bool), 1, fp);
		if (prodHasTrans)
		{
			//4x4 float Matrix
			ccGLMatrix identity;
			identity.toIdentity();
			fwrite(identity.data(), sizeof(float)*16, 1, fp);

			//Timestamp
			double t=0;
			fwrite(&t, sizeof(double), 1, fp);
		}

		//Reference role
		fwrite(&referenceRole, sizeof(MARKER_ROLE), 1, fp);

		//Producer transformation
		bool refHasTrans = false;
		fwrite(&refHasTrans, sizeof(bool), 1, fp);
	}

	//MemBlock
	bool hasMemBlock = false;
	fwrite(&hasMemBlock, sizeof(bool), 1, fp);

	//Associated probe calibration type
	unsigned probeCalibrationType = 0;
	fwrite(&probeCalibrationType, sizeof(unsigned), 1, fp);

	//Timestamp
	double t=0;
	fwrite(&t, sizeof(double), 1, fp);

	//Vertices
	const double* shift = theCloud->getOriginalShift();
	if (fabs(shift[0])+fabs(shift[0])+fabs(shift[0])>0.0)
        ccConsole::Warning(QString("[ULTFilter::save] Can't recenter cloud %1 on ULT/ULD file save!").arg(theCloud->getName()));

	for (unsigned i=0;i<count;++i)
	{
		const CCVector3* P = theCloud->getPoint(i);
		fwrite(P->u,sizeof(float)*3,1,fp);
	}

	//Other features
	bool hasColors = false;
	fwrite(&hasColors, sizeof(bool), 1, fp);
	bool hasSF = false;
	fwrite(&hasSF, sizeof(bool), 1, fp);
	bool hasNormals = false;
	fwrite(&hasNormals, sizeof(bool), 1, fp);
	bool hasSF2 = false;
	fwrite(&hasSF2, sizeof(bool), 1, fp);

	fclose(fp);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR UltFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	//ccConsole::Print("[BinFilter::loadFile] Opening binary file '%s'...\n",filename);

	assert(filename);

    //file size
    long size = QFileInfo(filename).size();

    if ( size == 0 || ((size % sizeof(MarkersFrame)) != 0))
		return CC_FERR_MALFORMED_FILE;

    //number of transformations in file
    long count = size / sizeof(MarkersFrame);
	ccConsole::Print("[TransBuffer] Found %i trans. in file '%s'",count,filename);
	if (count<1)
		return CC_FERR_NO_LOAD;

	ccPointCloud* cloud = new ccPointCloud();
	if (!cloud->reserve(count) || !cloud->enableScalarField())
	{
		delete cloud;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

    ccProgressDialog pdlg(true);
    pdlg.setMethodTitle("Open Ult File");
	CCLib::NormalizedProgress nprogress(&pdlg,count);
	pdlg.reset();
	pdlg.setInfo(qPrintable(QString("Transformations: %1").arg(count)));
	pdlg.start();
	QApplication::processEvents();

    FILE* fp = fopen(filename,"rb");
    if (!fp)
	{
		delete cloud;
        return CC_FERR_READING;
	}

	//which marker is the reference?
	QMessageBox::StandardButton tibiaIsRef = QMessageBox::question(0, "Choose reference", "Tibia as reference (yes)? Or femur (no)? Or none (no to all)", QMessageBox::Yes | QMessageBox::No | QMessageBox::NoToAll, QMessageBox::Yes );
	MARKER_ROLE referenceRole = MARKER_LOCALIZER;
	if (tibiaIsRef == QMessageBox::Yes)
		referenceRole = MARKER_TIBIA;
	else if (tibiaIsRef == QMessageBox::No)
		referenceRole = MARKER_FEMUR;

	//To apply a predefined pointer tip
	//CCVector3 tip(0,0,0);
	CCVector3 tip(-90.07f, -17.68f, 18.29f);

	MarkersFrame currentframe;
	MarkerState& currentMarker = currentframe.states[MARKER_POINTER];
	MarkerState* referenceMarker = 0;
	if (referenceRole != MARKER_LOCALIZER)
		referenceMarker = currentframe.states+referenceRole;

	unsigned MarkersFrameSize = sizeof(MarkersFrame);
	unsigned realCount=0;
	for (long i=0;i<count;++i)
	{
		if (fread(&currentframe,MarkersFrameSize,1,fp)==0)
		{
			fclose(fp);
			delete cloud;
			return CC_FERR_READING;
		}

		if (currentMarker.visible && (!referenceMarker || referenceMarker->visible))
		{
			CCVector3 P(tip);
			ccGLMatrix trans = currentMarker.pos;
			if (referenceMarker)
				trans = referenceMarker->pos.inverse() * trans;
			trans.apply(P);

			cloud->addPoint(P);
			cloud->setPointScalarValue(realCount,currentMarker.pos.timestamp);
			++realCount;
		}

		if (!nprogress.oneStep())
			break;
	}

	fclose(fp);

	if (realCount==0)
	{
		delete cloud;
		return CC_FERR_NO_LOAD;
	}

	cloud->resize(realCount);
    //we update scalar field
	CCLib::ScalarField* sf = cloud->getCurrentInScalarField();
    if (sf)
    {
        sf->setPositive(true);
        sf->computeMinAndMax();
        cloud->setCurrentDisplayedScalarField(cloud->getCurrentInScalarFieldIndex());
    }

	container.addChild(cloud);

	return CC_FERR_NO_ERROR;
}

#endif
