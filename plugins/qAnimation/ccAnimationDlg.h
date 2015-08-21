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
class ccAnimationDlg : public QDialog, public Ui::AnimationDialog
{
	Q_OBJECT

public:

	//! Default constructor
        ccAnimationDlg( std::vector < VideoStepItem > & video_steps, ccGLWindow * main_window,  QWidget* parent = 0  );

protected slots:


private slots:


        void on_fpsSpinBox_valueChanged(double arg1);

        void on_timeForStepBox_valueChanged(double arg1);

        void on_directoryButton_clicked();

        void on_previewButton_clicked();

        void on_renderButton_clicked();

        void on_stepSelectionList_itemSelectionChanged();

private:

        std::vector < VideoStepItem > m_video_steps;

        ccGLWindow * m_main_window;

        int GetCurrentListIndex();

        void SetView ( cc2DViewportObject * current_params );

        void Preview(   );

        void RenderToFile ( const QString & filename );

        void RunRender (  );

};

#endif
