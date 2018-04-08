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
class cc2DViewportObject;
class QListWidgetItem;

//! Dialog for qAnimation plugin
class qAnimationDlg : public QDialog, public Ui::AnimationDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qAnimationDlg(ccGLWindow* view3d,  QWidget* parent = 0);

	//! Initialize the dialog with a set of viewports
	bool init(const std::vector<cc2DViewportObject*>& viewports);

protected slots:

	void onFPSChanged(int);

	void onTotalTimeChanged(double);
	void onStepTimeChanged(double);
	void onLoopToggled(bool);
	void onCurrentStepChanged(int);
	void onBrowseButtonClicked();

	void preview();
	void renderAnimation() { render(false); }
	void renderFrames() { render(true); }
	void onAccept();

	void onItemChanged(QListWidgetItem*);

protected: //methods

	int getCurrentStepIndex();

	int countFrames(size_t startIndex = 0);

	void applyViewport( const cc2DViewportObject* viewport );

	double computeTotalTime();

	void updateCurrentStepDuration();
	void updateTotalDuration();

	bool getNextSegment(size_t& vp1, size_t& vp2) const;

	void render(bool asSeparateFrames);

protected: //members

	//! Simple step (viewport + time)
	struct Step
	{
		cc2DViewportObject* viewport;
		double duration_sec;

		Step() : viewport(0), duration_sec(0) {}
	};

	std::vector<Step> m_videoSteps;

	ccGLWindow* m_view3d;
};

#endif
