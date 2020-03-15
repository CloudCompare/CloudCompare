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

#ifndef CC_ANIMATION_DLG_HEADER
#define CC_ANIMATION_DLG_HEADER

//Qt
#include <QDialog>

//System
#include <vector>

#include "ui_animationDlg.h"

class ccGLWindow;
class ccPolyline;
class cc2DViewportObject;
class QListWidgetItem;

//! Dialog for qAnimation plugin
class qAnimationDlg : public QDialog, public Ui::AnimationDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qAnimationDlg(ccGLWindow* view3d,  QWidget* parent = nullptr);

	//! Destrcuctor
	virtual ~qAnimationDlg();

	//! Initialize the dialog with a set of viewports
	bool init(const std::vector<cc2DViewportObject*>& viewports);

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

	void preview();
	void renderAnimation() { render(false); }
	void renderFrames() { render(true); }
	void onAccept();
	void onReject();

	void onItemChanged(QListWidgetItem*);

protected: //methods

	int getCurrentStepIndex();

	int countFrames(size_t startIndex = 0);

	void applyViewport( const cc2DViewportObject* viewport );

	double computeTotalTime();

	void updateCurrentStepDuration();
	void updateTotalDuration();
	bool updateCameraTrajectory();
	bool updateSmoothCameraTrajectory();

	bool getNextSegment(size_t& vp1, size_t& vp2) const;

	void render(bool asSeparateFrames);

protected: //members

	//! Simple step (viewport + time)
	struct Step
	{
		cc2DViewportObject* viewport = nullptr;
		double duration_sec = 0.0;
		double distance = 0.0;
		int indexInSmoothTrajectory = -1;
	};

	//! Animation
	std::vector<Step> m_videoSteps;

	//! Associated 3D view
	ccGLWindow* m_view3d;

	//! Trajectory polyline
	ccPolyline* m_trajectory;
	//! Smooth trajectory polyline
	ccPolyline* m_smoothTrajectory;
	//! Smooth trajectory polyline (reversed)
	ccPolyline* m_smoothTrajectoryReversed;

};

#endif
