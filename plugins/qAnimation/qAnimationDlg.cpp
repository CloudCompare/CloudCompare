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

#include "qAnimationDlg.h"

#include <cc2DViewportObject.h>
#include <ccGLWindow.h>

//Qt
#include <QtGui>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QFileDialog>

//standard includes
#include <vector>
#include <sstream>
#include <iomanip>

//System
#include <algorithm>
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <unistd.h>
#endif

qAnimationDlg::qAnimationDlg( std::vector < VideoStepItem > & video_steps, ccGLWindow * main_window, QWidget* parent) :
	QDialog(parent),
	Ui::AnimationDialog(),
	m_video_steps ( video_steps ),
	m_main_window ( main_window )
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	for ( unsigned int i = 0; i < m_video_steps.size(); ++i )
	{
		stepSelectionList->addItem( (m_video_steps[i]).interpolator.view1()->getName() );
	}

	directoryLabel->setText( QDir::homePath() );

	stepSelectionList->setSelectionMode(QAbstractItemView::SingleSelection);

	QModelIndex modelIndex = stepSelectionList->rootIndex(); // u have to find the model index of the first item here
	stepSelectionList->setCurrentIndex(modelIndex);

	int current_item = GetCurrentListIndex();

	VideoStepItem first_video_step = m_video_steps[current_item];


	on_fpsSpinBox_valueChanged( first_video_step.fps );
	on_timeForStepBox_valueChanged( first_video_step.time_to_run );

	//    connect (fpsSpinBox, SIGNAL( valueChanged( double ) ), this, SLOT( on_fpsSpinBox_valueChanged (double) ) );
	//    connect (timeForStepBox , SIGNAL ( valueChanged ( double ) ), this, SLOT ( on_timeForStepBox_valueChanged( int ) ) );
	//    connect (stepSelectionList , SIGNAL ( itemChanged ( QListWidgetItem * ) ), this, SLOT ( on_stepSelectionList_itemChanged( QListWidgetItem *  ) ) );


}

int qAnimationDlg::GetCurrentListIndex()
{
	QModelIndex index ( stepSelectionList->currentIndex() );
	if ( index.isValid() )
	{
		return index.row();
	}
	return 0;
}

void qAnimationDlg::SetView ( cc2DViewportObject * current_params )
{
	m_main_window->setViewportParameters( current_params->getParameters() );

	m_main_window->redraw();
	QApplication::processEvents();
}

void qAnimationDlg::Preview(  )
{
	for ( int i = 0 ; i < stepSelectionList->count() ; ++i )
	{

		//stepSelectionList->setCurrentIndex( i );

		VideoStepItem current_video_step = m_video_steps[i];

		cc2DViewportObject current_params;

		current_video_step.interpolator.setMaxStep( static_cast < unsigned int > ( current_video_step.fps * current_video_step.time_to_run ) );

		while ( (current_video_step).interpolator.nextView( current_params ) )
		{
			SetView ( &current_params );

			int fps ( current_video_step.fps );
#if defined(CC_WINDOWS)
			::Sleep( 1000/fps );
#else
			usleep(1000/fps * 1000);
#endif
		}
	}

}

void qAnimationDlg::RenderToFile ( const QString & filename )
{
	QApplication::processEvents();
	m_main_window->renderToFile(qPrintable( filename ), 1.0 , true, false );
}

void qAnimationDlg::RunRender ()
{
	double frame_time = 0.0;
	for ( int i = 0 ; i < stepSelectionList->count() ; ++i )
	{
		VideoStepItem current_video_step = m_video_steps[i];

		cc2DViewportObject current_params;

		current_video_step.interpolator.setMaxStep( static_cast < unsigned int > ( current_video_step.fps * current_video_step.time_to_run ) );

		while ( (current_video_step).interpolator.nextView( current_params ) )
		{
			frame_time += 1.0 / current_video_step.fps;

			std::stringstream filename ;
			filename << std::fixed ;
			filename << std::setprecision (6);
			filename << std::setfill ('0');
			filename << std::setw(20);
			filename << std::right;
			filename << frame_time << ".png";
			QString filename_q = QString::fromStdString( filename.str() );
			QString dirname ( directoryLabel->text() );

			QString full_path ( QDir ( dirname ).filePath( filename_q ) );

			SetView ( &current_params );

			RenderToFile( full_path );
		}
	}
}

void qAnimationDlg::on_fpsSpinBox_valueChanged(double arg1)
{

	m_video_steps[ GetCurrentListIndex() ].fps = arg1 ;
}

void qAnimationDlg::on_timeForStepBox_valueChanged(double arg1)
{
	m_video_steps[ GetCurrentListIndex() ].time_to_run = arg1 ;
}

void qAnimationDlg::on_directoryButton_clicked()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
		directoryLabel->text(),
		QFileDialog::ShowDirsOnly
		| QFileDialog::DontResolveSymlinks);
	if ( dir == QString("") )
	{
		dir = QDir::homePath();
	}

	directoryLabel->setText (dir);
}

void qAnimationDlg::on_previewButton_clicked()
{
	Preview ();
}


void qAnimationDlg::on_renderButton_clicked()
{
	RunRender ();
}

void qAnimationDlg::on_stepSelectionList_itemSelectionChanged()
{
	int index = GetCurrentListIndex();

	fpsSpinBox->setValue ( m_video_steps[index].fps );
	timeForStepBox->setValue ( m_video_steps[index].time_to_run );

	cc2DViewportObject * current_params ( m_video_steps[index].interpolator.view1() );

	SetView ( current_params );

}
