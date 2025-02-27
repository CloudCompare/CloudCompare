#pragma once

//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qCloudLayers                    #
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
//#                     COPYRIGHT: WigginsTech 2022                        #
//#                                                                        #
//##########################################################################

#include "ccAsprsModel.h"

//CC
#include <CCTypes.h>
#include <CCGeom.h>
#include <ccColorTypes.h>
#include <ccGenericGLDisplay.h>

//QT
#include <QColor>

//std
#include <vector>

class ccPointCloud;
class QStringList;
class ccMainAppInterface;
class RGBAColorsTableType;

class ccCloudLayersHelper
{
public:
	ccCloudLayersHelper(ccMainAppInterface* app);
	~ccCloudLayersHelper();

	bool setCloud(ccPointCloud* cloud);
	void restoreCloud(bool restoreSFValues);

	QStringList getScalarFields();
	bool setScalarFieldIndexAndStoreValues(int index);
	int getCurrentScalarFieldIndex() const { return m_scalarFieldIndex; }

	// set colors alpha to MAX
	void setVisible(bool value);

	// apply visibility and colors
	void applyClassColors(QList<ccAsprsModel::AsprsItem>& items);

	// apply visibility and color return affected count
	int applyClassColor(ccAsprsModel::AsprsItem& item, bool redrawDisplay = false);

	// asprs item code changed
	void changeCode(const ccAsprsModel::AsprsItem& item, ScalarType oldCode);

	// set scalar code to zero return affected count
	int moveItem(const ccAsprsModel::AsprsItem& from, const ccAsprsModel::AsprsItem* to, bool redrawDisplay = false);

	//! Restore original scalar values
	void restoreCurrentSFValues();

	void mouseMove(const CCVector2& center, PointCoordinateType squareDist, std::map<ScalarType, int>& affected);
	bool projectCloud(const ccGLCameraParameters& camera);

	//! Whether the scalar field values (and currently displayed colors) have been modified
	bool modified() const { return m_modified; }

	struct Parameters
	{
		bool anyPoints = false;
		bool visiblePoints = false;
		ccAsprsModel::AsprsItem* input = nullptr;
		ccAsprsModel::AsprsItem* output = nullptr;
	};

	Parameters& getParameters();

	inline ccPointCloud* cloud() { return m_cloud; }

	void keepCurrentSFVisible();

private: // methods
	void project(const ccGLCameraParameters& camera, unsigned start, unsigned end);

	//! Save current scalar field values
	bool saveCurrentSFValues(int sfIndex);

private: // variables
	ccMainAppInterface* m_app;
	ccPointCloud* m_cloud;
	int m_scalarFieldIndex;
	bool m_modified;

	Parameters m_parameters;
	ccGLCameraParameters m_cameraParameters;

	struct BackupData
	{
		bool sfWasShown = false;
		int displayedSFIndex = -1;
		bool colorsWereShown = false;
		bool hadColors = false;
		QSharedPointer<RGBAColorsTableType> colors;
		std::vector<double> scalarValues;
	};
	BackupData m_originalCloudState;

	struct ProjectedPoint
	{
		CCVector2 pos2D;
		bool inFrustum = false;
	};
	std::vector<ProjectedPoint> m_projectedPoints;
};

