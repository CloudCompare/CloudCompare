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
	ccCloudLayersHelper(ccMainAppInterface* app, ccPointCloud* cloud);
	~ccCloudLayersHelper();

	QStringList getScalarFields();
	void setScalarFieldIndex(int index);

	// set colors alpha to MAX
	void setVisible(bool value);

	// apply visibility and colors
	void apply(QList<ccAsprsModel::AsprsItem>& items);

	// apply visibility and color return affected count
	int apply(ccAsprsModel::AsprsItem& item, bool redrawDisplay = false);

	// asprs item code changed
	void changeCode(const ccAsprsModel::AsprsItem& item, ScalarType oldCode);

	// set scalar code to zero return affected count
	int moveItem(const ccAsprsModel::AsprsItem& from, const ccAsprsModel::AsprsItem* to, bool redrawDisplay = false);

	// save color and codes
	void saveState();

	// restore initial colors and codes
	void restoreState();

	void mouseMove(const CCVector2& center, float squareDist, std::map<ScalarType, int>& affected);
	void projectCloud(const ccGLCameraParameters& camera);
	bool hasChanges() const { return m_modified; }

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
	void project(ccGLCameraParameters camera, unsigned start, unsigned end);
	static PointCoordinateType ComputeSquaredEuclideanDistance(const CCVector2& a, const CCVector2& b);

private: // variables
	ccMainAppInterface* m_app;
	ccPointCloud* m_cloud;
	RGBAColorsTableType* m_formerCloudColors;
	bool m_formerCloudColorsWereShown;
	bool m_formerCloudSFWasShown;
	Parameters m_parameters;

	unsigned m_scalarFieldIndex;
	bool m_modified;

	ccGLCameraParameters m_cameraParameters;
	std::vector<CCVector2> m_projectedPoints;
	std::vector<bool> m_pointInFrustum;

	struct CloudState
	{
	public:
		CloudState() {}

		void update(ScalarType code, ccColor::Rgb color)
		{
			this->code = code;
			this->color = color;
		}

		ScalarType code;
		ccColor::Rgb color;
	};

	std::vector<CloudState> m_cloudState;

};

