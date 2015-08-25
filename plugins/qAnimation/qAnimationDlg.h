//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#         COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015				   #
//#                                                                        #
//##########################################################################

#ifndef CC_ANIMATION_DLG_HEADER
#define CC_ANIMATION_DLG_HEADER

#include "ui_animationDlg.h"
#include "VideoStepItem.h"

//forward declare
class ccGLWindow;
class cc2DViewportObject;


//! Dialog for qAnimation plugin
class qAnimationDlg : public QDialog, public Ui::AnimationDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qAnimationDlg( std::vector<VideoStepItem>& video_steps, ccGLWindow* view3d,  QWidget* parent = 0 );

protected slots:

	void onFPSChanged(double);

	void onTotalTimeChanged(double);
	void onStepTimeChanged(double);

	void onCurrentStepChanged(int);

	void onBrowseButtonClicked();

	void preview();
	void render();
	void onAccept();

protected:

	std::vector<VideoStepItem>& m_videoSteps;

	ccGLWindow* m_view3d;

	int getCurrentStepIndex();

	int countFrameAndResetInterpolators();

	void applyViewport( const cc2DViewportObject* viewport );

	double computeTotalTime();

	void updateCurrentStepDuration();
	void updateTotalDuration();
};

#endif
