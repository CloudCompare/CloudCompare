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

static const QString s_stepDurationKey("StepDurationSec");

qAnimationDlg::qAnimationDlg( std::vector<VideoStepItem>& videoSteps, ccGLWindow* view3d, QWidget* parent)
	: QDialog(parent)
	, Ui::AnimationDialog()
	, m_videoSteps(videoSteps)
	, m_view3d(view3d)
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	for ( size_t i=0; i<m_videoSteps.size(); ++i )
	{
		const cc2DViewportObject* viewport1 = m_videoSteps[i].interpolator.view1();
		const cc2DViewportObject* viewport2 = m_videoSteps[i].interpolator.view2();

		stepSelectionList->addItem( QString("Step %1 (%2 - %3)").arg(i+1).arg(viewport1->getName()).arg(viewport2->getName()) );

		//check if the (1st) viewport has a duration in meta data (from a previous run)
		double duration_sec = 2.0;
		if (viewport1->hasMetaData(s_stepDurationKey))
		{
			duration_sec = viewport1->getMetaData(s_stepDurationKey).toDouble();
		}
		m_videoSteps[i].duration_sec = duration_sec;
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

	connect ( fpsSpinBox,				SIGNAL( valueChanged(double) ),		this, SLOT( onFPSChanged(double) ) );
	connect ( totalTimeDoubleSpinBox,	SIGNAL( valueChanged(double) ),		this, SLOT( onTotalTimeChanged(double) ) );
	connect ( stepTimeDoubleSpinBox,	SIGNAL( valueChanged(double) ),		this, SLOT( onStepTimeChanged(double) ) );
	connect ( stepSelectionList,		SIGNAL( currentRowChanged(int) ),	this, SLOT( onCurrentStepChanged(int) ) );
	connect ( browseButton,				SIGNAL( clicked() ),				this, SLOT( onBrowseButtonClicked() ) );
	connect ( previewButton,			SIGNAL( clicked() ),				this, SLOT( preview() ) );
	connect ( renderButton,				SIGNAL( clicked() ),				this, SLOT( render() ) );
	connect ( buttonBox,				SIGNAL( accepted() ),				this, SLOT( onAccept() ) );

	stepSelectionList->setCurrentRow(0); //select the first one by default

	onCurrentStepChanged(getCurrentStepIndex());
	updateTotalDuration();
}

void qAnimationDlg::onAccept()
{
	for ( size_t i=0; i<m_videoSteps.size(); ++i )
	{
		cc2DViewportObject* viewport1 = m_videoSteps[i].interpolator.view1();

		//save the step duration as meta data
		viewport1->setMetaData(s_stepDurationKey, m_videoSteps[i].duration_sec);
	}
}

double qAnimationDlg::computeTotalTime()
{
	double totalDuration_sec = 0;
	for ( size_t i=0; i<m_videoSteps.size(); ++i )
	{
		totalDuration_sec += m_videoSteps[i].duration_sec;
	}

	return totalDuration_sec;
}

int qAnimationDlg::getCurrentStepIndex()
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
	//nothing to do
}

void qAnimationDlg::onTotalTimeChanged(double newTime_sec)
{
	double previousTime_sec = computeTotalTime();
	if (previousTime_sec != newTime_sec)
	{
		assert(previousTime_sec != 0);
		double scale = newTime_sec / previousTime_sec;

		//scale all the steps
		for ( size_t i=0; i<m_videoSteps.size(); ++i )
		{
			m_videoSteps[i].duration_sec *= scale;
		}

		//update current step
		updateCurrentStepDuration();
	}
}

void qAnimationDlg::onStepTimeChanged(double time_sec)
{
	m_videoSteps[ getCurrentStepIndex() ].duration_sec = time_sec;

	//update total duration
	updateTotalDuration();
	//update current step
	updateCurrentStepDuration();
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
	int totalFrameCount = 0;
	{
		double fps = fpsSpinBox->value();

		for ( size_t i=0; i<m_videoSteps.size(); ++i )
		{
			VideoStepItem& currentVideoStep = m_videoSteps[i];

			cc2DViewportObject currentParams;
			currentVideoStep.interpolator.reset();

			int frameCount = static_cast<int>( fps * currentVideoStep.duration_sec );
			currentVideoStep.interpolator.setMaxStep(static_cast<unsigned>(frameCount));

			totalFrameCount += frameCount;
		}
	}

	return totalFrameCount;
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

	double fps = fpsSpinBox->value();
	
	int frameIndex = 0;
	for ( size_t i=0; i<m_videoSteps.size(); ++i )
	{
		VideoStepItem& currentVideoStep = m_videoSteps[i];

		//theoretical waiting time per frame
		qint64 delay_ms = static_cast<int>(1000 * currentVideoStep.duration_sec / fps);

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
	onCurrentStepChanged( getCurrentStepIndex() );

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
	for ( size_t i=0; i<m_videoSteps.size(); ++i )
	{
		VideoStepItem& currentVideoStep = m_videoSteps[i];

		cc2DViewportObject current_params;
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

void qAnimationDlg::updateTotalDuration()
{
	double totalDuration_sec = computeTotalTime();

	totalTimeDoubleSpinBox->blockSignals(true);
	totalTimeDoubleSpinBox->setValue(totalDuration_sec);
	totalTimeDoubleSpinBox->blockSignals(false);
}

void qAnimationDlg::updateCurrentStepDuration()
{
	int index = getCurrentStepIndex();
	const VideoStepItem& currentVideoStep = m_videoSteps[index];

	stepTimeDoubleSpinBox->blockSignals(true);
	stepTimeDoubleSpinBox->setValue(currentVideoStep.duration_sec);
	stepTimeDoubleSpinBox->blockSignals(false);
}

void qAnimationDlg::onCurrentStepChanged(int index)
{
	//update current step descriptor
	stepIndexLabel->setText(QString::number(index+1));

	updateCurrentStepDuration();

	applyViewport( m_videoSteps[index].interpolator.view1() );
}
