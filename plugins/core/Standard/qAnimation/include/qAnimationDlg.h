#pragma once

//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#             COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015              #
//#                                                                        #
//##########################################################################

//Local
#include "ExtendedViewport.h"

//qCC_db
#include <ccViewportParameters.h>

//Qt
#include <QDialog>

//System
#include <vector>

#include "ui_animationDlg.h"

class ccGLWindowInterface;
class ccPolyline;
class cc2DViewportObject;
class QListWidgetItem;

//! Dialog for qAnimation plugin
class qAnimationDlg : public QDialog, public Ui::AnimationDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qAnimationDlg(ccGLWindowInterface* view3d,  QWidget* parent = nullptr);

	//! Destrcuctor
	virtual ~qAnimationDlg();

	//! Initialize the dialog with a set of viewports
	bool init(const std::vector<ExtendedViewport>& viewports);

	ccPolyline* getTrajectory();
	bool exportTrajectoryOnExit();

protected:

	void onFPSChanged(int);
	void onTotalTimeChanged(double);
	void onStepTimeChanged(double);
	void onLoopToggled(bool);
	void onCurrentStepChanged(int);
	void onBrowseButtonClicked();
	void onAutoStepsDurationToggled(bool);
	void onSmoothTrajectoryToggled(bool);
	void onSmoothRatioChanged(double);
	void onCodecChanged(int);

	void preview();
	void renderAnimation() { render(false); }
	void renderFrames() { render(true); }
	void onAccept();
	void onReject();

	void onItemChanged(QListWidgetItem*);

protected: //methods

	int getCurrentStepIndex();
	size_t countEnabledSteps() const;

	bool smoothModeEnabled() const;

	int countFrames(size_t startIndex = 0);

	void applyViewport(const ExtendedViewportParameters& viewportParameters);

	double computeTotalTime();

	void updateCurrentStepDuration();
	void updateTotalDuration();
	bool updateCameraTrajectory();
	bool updateSmoothCameraTrajectory();

	bool getNextSegment(size_t& vp1, size_t& vp2) const;

	void render(bool asSeparateFrames);

	bool smoothTrajectory(double ratio, unsigned iterationCount);

	//! Simple step (viewport + time)
	struct Step : public ExtendedViewportParameters
	{
		cc2DViewportObject* viewport = nullptr;

		int indexInOriginalTrajectory = -1;
		CCVector3d cameraCenter;

		double duration_sec = 0.0;
		double length = 0.0;
		int indexInSmoothTrajectory = -1;
	};

	typedef std::vector<Step> Trajectory;

	bool getCompressedTrajectory(Trajectory& compressedTrajectory) const;

	void updateSmoothTrajectoryDurations();

protected: //members

	//! Animation
	Trajectory m_videoSteps;
	//! Smoothed animation
	Trajectory m_smoothVideoSteps;

	//! Associated 3D view
	ccGLWindowInterface* m_view3d;

	//! Map of the codecs short names Vs extensions
	QMap<QString, QString> m_codecNamesAndExtensions;
};
