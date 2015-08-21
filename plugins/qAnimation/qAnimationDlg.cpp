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
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QElapsedTimer>
#include <QProgressDialog>

//standard includes
#include <vector>
#include <iomanip>

#ifdef QFFMPEG_SUPPORT
//QTFFmpeg
#include <QVideoEncoder.h>
#endif

//System
#include <algorithm>
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <unistd.h>
#endif

qAnimationDlg::qAnimationDlg( std::vector<VideoStepItem>& videoSteps, ccGLWindow* view3d, QWidget* parent)
	: QDialog(parent)
	, Ui::AnimationDialog()
	, m_videoSteps(videoSteps)
	, m_view3d(view3d)
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	for (size_t i=0; i<m_videoSteps.size(); ++i )
	{
		stepSelectionList->addItem( m_videoSteps[i].interpolator.view1()->getName() );
	}

	//read persistent settings
	{
		QSettings settings;
		settings.beginGroup("qAnimation");
		QString defaultDir;
#ifdef _MSC_VER
		defaultDir = QApplication::applicationDirPath();
#else
		defaultDir = QDir::homePath();
#endif
		QString lastFilename = settings.value("filename", defaultDir + "/animation.mpg" ).toString();
#ifndef QFFMPEG_SUPPORT
		lastFilename = QFileInfo(lastFilename).absolutePath();
#endif
		outputFileLineEdit->setText( lastFilename );
		settings.endGroup();
	}

	connect ( fpsSpinBox,			SIGNAL( valueChanged(double) ),		this, SLOT( onFPSChanged(double) ) );
	connect ( timeForStepBox,		SIGNAL( valueChanged(double) ),		this, SLOT( onTimeForStepChanged(double) ) );
	connect ( stepSelectionList,	SIGNAL( currentRowChanged(int) ),	this, SLOT( onCurrentStepChanged(int) ) );
	connect ( browseButton,			SIGNAL( clicked() ),				this, SLOT( onBrowseButtonClicked() ) );
	connect ( previewButton,		SIGNAL( clicked() ),				this, SLOT( preview() ) );
	connect ( renderButton,			SIGNAL( clicked() ),				this, SLOT( render() ) );

	stepSelectionList->setCurrentRow(0); //select the first one by default

	onCurrentStepChanged(getCurrentListIndex());
}

int qAnimationDlg::getCurrentListIndex()
{
	return stepSelectionList->currentRow();
}

void qAnimationDlg::applyViewport( const cc2DViewportObject* viewport )
{
	m_view3d->setViewportParameters( viewport->getParameters() );
	m_view3d->redraw();

	//QApplication::processEvents();
}

void qAnimationDlg::onFPSChanged(double fps)
{
	m_videoSteps[ getCurrentListIndex() ].fps = fps;
}

void qAnimationDlg::onTimeForStepChanged(double time)
{
	m_videoSteps[ getCurrentListIndex() ].time_to_run = time;
}

void qAnimationDlg::onBrowseButtonClicked()
{
#ifdef QFFMPEG_SUPPORT
	QString filename = QFileDialog::getSaveFileName(	this,
														tr("Output animation file"),
														outputFileLineEdit->text() );
#else
	QString filename = QFileDialog::getExistingDirectory(	this,
															tr("Open Directory"),
															outputFileLineEdit->text(),
															QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks );
#endif

	if (filename.isEmpty())
	{
		//cancelled by user
		return;
	}

	outputFileLineEdit->setText(filename);
}

int qAnimationDlg::countFrameAndResetInterpolators()
{
	//reset the interpolators and count the total number of frames
	int frameCount = 0;
	{
		for ( int i=0; i<stepSelectionList->count(); ++i )
		{
			VideoStepItem& currentVideoStep = m_videoSteps[i];

			cc2DViewportObject currentParams;
			currentVideoStep.interpolator.reset();
			int count = static_cast<int>( currentVideoStep.fps * currentVideoStep.time_to_run );
			currentVideoStep.interpolator.setMaxStep(static_cast<unsigned>(count));

			frameCount += count;
		}
	}

	return frameCount;
}

void qAnimationDlg::preview()
{
	//we'll take the rendering time into account!
	QElapsedTimer timer;
	timer.start();

	setEnabled(false);

	//reset the interpolators and count the total number of frames
	int frameCount = countFrameAndResetInterpolators();

	//show progress dialog
	QProgressDialog progressDialog(QString("Frames: %1").arg(frameCount), "Cancel", 0, frameCount, this);
	progressDialog.setWindowTitle("Preview");
	progressDialog.show();
	QApplication::processEvents();

	int frameIndex = 0;
	for ( int i=0; i<stepSelectionList->count(); ++i )
	{
		VideoStepItem& currentVideoStep = m_videoSteps[i];

		//theoretical waiting time per frame
		qint64 delay_ms = static_cast<int>(1000 * currentVideoStep.time_to_run / currentVideoStep.fps);

		cc2DViewportObject currentParams;
		while ( currentVideoStep.interpolator.nextView( currentParams ) )
		{
			timer.restart();
			applyViewport ( &currentParams );
			qint64 dt_ms = timer.elapsed();

			progressDialog.setValue(++frameIndex);
			QApplication::processEvents();
			if (progressDialog.wasCanceled())
			{
				break;
			}

			//remaining time
			if (dt_ms < delay_ms)
			{
				int wait_ms = static_cast<int>(delay_ms - dt_ms);
#if defined(CC_WINDOWS)
				::Sleep( wait_ms );
#else
				usleep( wait_ms * 1000 );
#endif
			}
		}
	}

	//reset view
	onCurrentStepChanged(getCurrentListIndex());

	setEnabled(true);
}

void qAnimationDlg::render()
{
	QString outputFilename = outputFileLineEdit->text();

	//save to persistent settings
	{
		QSettings settings;
		settings.beginGroup("qAnimation");
		settings.setValue("filename", outputFilename);
		settings.endGroup();
	}

	setEnabled(false);

	//reset the interpolators and count the total number of frames
	int frameCount = countFrameAndResetInterpolators();

	//show progress dialog
	QProgressDialog progressDialog(QString("Frames: %1").arg(frameCount), "Cancel", 0, frameCount, this);
	progressDialog.setWindowTitle("Render");
	progressDialog.show();
	QApplication::processEvents();

#ifdef QFFMPEG_SUPPORT
	//get original viewport size
	QSize originalViewSize = m_view3d->size();

	//hack: as the encoder requires that the video dimensions are multiples of 8, we resize the window a little bit...
	{
		//find the nearest multiples of 8
		QSize customSize = originalViewSize;
		if (originalViewSize.width() % 8 || originalViewSize.height() % 8)
		{
			if (originalViewSize.width() % 8)
				customSize.setWidth((originalViewSize.width() / 8 + 1) * 8);
			if (originalViewSize.height() % 8)
				customSize.setHeight((originalViewSize.height() / 8 + 1) * 8);
			m_view3d->resize(customSize);
			QApplication::processEvents();
		}
	}

	int bitrate = bitrateSpinBox->value();
	int gop = 12;
	QVideoEncoder encoder(outputFilename, m_view3d->width(), m_view3d->height(), bitrate, gop, static_cast<unsigned>(fpsSpinBox->value()));
	QString errorString;
	if (!encoder.open(&errorString))
	{
		QMessageBox::critical(this, "Error", QString("Failed to open file for output: %1").arg(errorString));
		setEnabled(true);
		return;
	}
#endif

	int frameIndex = 0;
	bool success = true;
	for ( int i=0; i<stepSelectionList->count(); ++i )
	{
		VideoStepItem& currentVideoStep = m_videoSteps[i];

		cc2DViewportObject current_params;
		currentVideoStep.interpolator.reset();
		currentVideoStep.interpolator.setMaxStep( static_cast < unsigned int > ( currentVideoStep.fps * currentVideoStep.time_to_run ) );

		while ( currentVideoStep.interpolator.nextView( current_params ) )
		{
			applyViewport ( &current_params );

			//render to image
			QImage image = m_view3d->renderToImage(1.0 , true, false, true );
			++frameIndex;

			if (image.isNull())
			{
				QMessageBox::critical(this, "Error", "Failed to grab the screen!");
				success = false;
				break;
			}

#ifdef QFFMPEG_SUPPORT
			if (!encoder.encodeImage(image, &errorString))
			{
				QMessageBox::critical(this, "Error", QString("Failed to encode frame #%1: %2").arg(frameIndex).arg(errorString));
				success = false;
				break;
			}
#else
			QString filename = QString("frame_%1.png").arg(frameIndex,6,10,QChar('0'));
			QString fullPath = QDir(outputFilename).filePath(filename);
			if (!image.save(fullPath))
			{
				QMessageBox::critical(this, "Error", QString("Failed to save frame #%1").arg(frameIndex));
				success = false;
				break;
			}
#endif
			progressDialog.setValue(frameIndex);
			QApplication::processEvents();
			if (progressDialog.wasCanceled())
			{
				QMessageBox::warning(this, "Warning", QString("Process has been cancelled"));
				success = false;
				break;
			}
		}

		if (!success)
		{
			break;
		}
	}

#ifdef QFFMPEG_SUPPORT
	encoder.close();

	//hack: restore original size
	m_view3d->resize(originalViewSize);
	QApplication::processEvents();
#endif
	
	progressDialog.hide();
	QApplication::processEvents();

	if (success)
	{
		QMessageBox::information(this, "Job done", "The animation has been saved successfully");
	}

	setEnabled(true);
}

void qAnimationDlg::onCurrentStepChanged(int index)
{
	const VideoStepItem& videoStep = m_videoSteps[index];
	
	//update current values
	fpsSpinBox->setValue(videoStep.fps);
	timeForStepBox->setValue(videoStep.time_to_run);

	applyViewport( videoStep.interpolator.view1() );
}
