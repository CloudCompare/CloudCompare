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

#include "qAnimationDlg.h"

//Local
#include "ViewInterpolate.h"

//qCC_db
#include <cc2DViewportObject.h>
//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QtGui>
#include <QApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QElapsedTimer>
#include <QProgressDialog>
#include <QMessageBox>

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
#include "windows.h"
#else
#include <unistd.h>
#endif

static const QString s_stepDurationKey("StepDurationSec");
static const QString s_stepEnabledKey("StepEnabled");

qAnimationDlg::qAnimationDlg(ccGLWindow* view3d, QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::AnimationDialog()
	, m_view3d(view3d)
{
	setupUi(this);

	//restore previous settings
	{
		QSettings settings;
		settings.beginGroup("qAnimation");
		
		//last filename
		{
			QString defaultDir;
#ifdef _MSC_VER
			defaultDir = QApplication::applicationDirPath();
#else
			defaultDir = QDir::homePath();
#endif
			const QString defaultFileName( defaultDir + "/animation.mp4" );
			QString lastFilename = settings.value("filename", defaultFileName ).toString();
#ifndef QFFMPEG_SUPPORT
			lastFilename = QFileInfo(lastFilename).absolutePath();
#endif
			outputFileLineEdit->setText( lastFilename );
		}

		//other parameters
		{
			bool startPreviewFromSelectedStep = settings.value("previewFromSelected", previewFromSelectedCheckBox->isChecked()).toBool();
			bool loop = settings.value("loop", loopCheckBox->isChecked()).toBool();
			int frameRate = settings.value("frameRate", fpsSpinBox->value()).toInt();
			int superRes = settings.value("superRes", superResolutionSpinBox->value()).toInt();
			int renderingMode = settings.value("renderingMode", renderingModeComboBox->currentIndex()).toInt();
			int bitRate = settings.value("bitRate", bitrateSpinBox->value()).toInt();

			previewFromSelectedCheckBox->setChecked(startPreviewFromSelectedStep);
			loopCheckBox->setChecked(loop);
			fpsSpinBox->setValue(frameRate);
			superResolutionSpinBox->setValue(superRes);
			renderingModeComboBox->setCurrentIndex(renderingMode);
			bitrateSpinBox->setValue(bitRate);
		}
		
		settings.endGroup();
	}

	connect ( fpsSpinBox,				SIGNAL( valueChanged(int) ),		this, SLOT( onFPSChanged(int) ) );
	connect ( totalTimeDoubleSpinBox,	SIGNAL( valueChanged(double) ),		this, SLOT( onTotalTimeChanged(double) ) );
	connect ( stepTimeDoubleSpinBox,	SIGNAL( valueChanged(double) ),		this, SLOT( onStepTimeChanged(double) ) );
	connect ( loopCheckBox,				SIGNAL( toggled(bool) ),			this, SLOT( onLoopToggled(bool) ) );

	connect ( browseButton,				SIGNAL( clicked() ),				this, SLOT( onBrowseButtonClicked() ) );
	connect ( previewButton,			SIGNAL( clicked() ),				this, SLOT( preview() ) );
	connect ( renderButton,				SIGNAL( clicked() ),				this, SLOT( renderAnimation() ) );
	connect ( exportFramesPushButton,	SIGNAL( clicked() ),				this, SLOT( renderFrames() ) );
	connect ( buttonBox,				SIGNAL( accepted() ),				this, SLOT( onAccept() ) );
}

bool qAnimationDlg::init(const std::vector<cc2DViewportObject*>& viewports)
{
	if (viewports.size() < 2)
	{
		assert(false);
		return false;
	}
	
	try
	{
		m_videoSteps.resize(viewports.size());
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	
	for (size_t i = 0; i < viewports.size(); ++i)
	{
		cc2DViewportObject* vp = viewports[i];

		//check if the (1st) viewport has a duration in meta data (from a previous run)
		double duration_sec = 2.0;
		if (vp->hasMetaData(s_stepDurationKey))
		{
			duration_sec = vp->getMetaData(s_stepDurationKey).toDouble();
		}
		bool isChecked = true;
		if (vp->hasMetaData(s_stepEnabledKey))
		{
			isChecked = vp->getMetaData(s_stepEnabledKey).toBool();
		}

		QString itemName = QString("step %1 (%2)").arg(QString::number(i+1), vp->getName());
		QListWidgetItem* item = new QListWidgetItem(itemName, stepSelectionList);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
		item->setCheckState(isChecked ? Qt::Checked : Qt::Unchecked); // initialize check state
		stepSelectionList->addItem(item);

		m_videoSteps[i].viewport = vp;
		m_videoSteps[i].duration_sec = duration_sec;
	}

	connect ( stepSelectionList, SIGNAL( currentRowChanged(int) ),			this, SLOT( onCurrentStepChanged(int) ) );
	connect ( stepSelectionList, SIGNAL( itemChanged(QListWidgetItem*) ),	this, SLOT( onItemChanged(QListWidgetItem*) ) );

	stepSelectionList->setCurrentRow(0); //select the first one by default
	onCurrentStepChanged(getCurrentStepIndex());
	updateTotalDuration();

	return true;
}

void qAnimationDlg::onAccept()
{
	assert(stepSelectionList->count() >= m_videoSteps.size());
	for (size_t i = 0; i < m_videoSteps.size(); ++i)
	{
		cc2DViewportObject* vp = m_videoSteps[i].viewport;

		//save the step duration as meta data
		vp->setMetaData(s_stepDurationKey, m_videoSteps[i].duration_sec);
		vp->setMetaData(s_stepEnabledKey, (stepSelectionList->item(static_cast<int>(i))->checkState() == Qt::Checked));
	}

	//store settings
	{
		QSettings settings;
		settings.beginGroup("qAnimation");
		settings.setValue("previewFromSelected", previewFromSelectedCheckBox->isChecked());
		settings.setValue("loop", loopCheckBox->isChecked());
		settings.setValue("frameRate", fpsSpinBox->value());
		settings.setValue("renderingMode", renderingModeComboBox->currentIndex());
		settings.setValue("superRes", superResolutionSpinBox->value());
		settings.setValue("bitRate", bitrateSpinBox->value());

		settings.endGroup();
	}
}

double qAnimationDlg::computeTotalTime()
{
	double totalDuration_sec = 0;
	size_t vp1 = 0;
	size_t vp2 = 0;
	while (getNextSegment(vp1, vp2))
	{
		assert(vp1 < stepSelectionList->count());
		totalDuration_sec += m_videoSteps[static_cast<int>(vp1)].duration_sec;
		if (vp2 == 0)
		{
			//loop case
			break;
		}
		vp1 = vp2;
	}

	return totalDuration_sec;
}

int qAnimationDlg::getCurrentStepIndex()
{
	return stepSelectionList->currentRow();
}

void qAnimationDlg::applyViewport( const cc2DViewportObject* viewport )
{
	if (m_view3d)
	{
		m_view3d->setViewportParameters( viewport->getParameters() );
		m_view3d->redraw();
	}

	//QApplication::processEvents();
}

void qAnimationDlg::onFPSChanged(int fps)
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

		size_t vp1 = 0;
		size_t vp2 = 0;
		while (getNextSegment(vp1, vp2))
		{
			assert(vp1 < stepSelectionList->count());
			m_videoSteps[vp1].duration_sec *= scale;

			if (vp2 == 0)
			{
				//loop case
				break;
			}
			vp1 = vp2;
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

bool qAnimationDlg::getNextSegment(size_t& vp1, size_t& vp2) const
{
	if( vp1 >= m_videoSteps.size())
	{
		assert(false);
		return false;
	}

	size_t inputVP1 = vp1;
	while (stepSelectionList->item(static_cast<int>(vp1))->checkState() == Qt::Unchecked)
	{
		++vp1;
		if (vp1 == m_videoSteps.size())
		{
			if (loopCheckBox->isChecked())
			{
				vp1 = 0;
			}
			else
			{
				//no more valid start (vp1)
				return false;
			}
		}
		if (vp1 == inputVP1)
		{
			return false;
		}
	}

	//look for the next enabled viewport
	for (vp2 = vp1+1; vp2 <= m_videoSteps.size(); ++vp2)
	{
		if (vp1 == vp2)
		{
			return false;
		}

		if (vp2 == m_videoSteps.size())
		{
			if (loopCheckBox->isChecked())
			{
				vp2 = 0;
			}
			else
			{
				//stop
				break;
			}
		}
		
		if (stepSelectionList->item(static_cast<int>(vp2))->checkState() == Qt::Checked)
		{
			//we have found a valid couple (vp1, vp2)
			return true;
		}
	}

	//no more valid stop (vp2)
	return false;
}

int qAnimationDlg::countFrames(size_t startIndex/*=0*/)
{
	//reset the interpolators and count the total number of frames
	int totalFrameCount = 0;
	{
		double fps = fpsSpinBox->value();

		size_t vp1 = startIndex;
		size_t vp2 = vp1+1;

		while (getNextSegment(vp1, vp2))
		{
			const Step& currentStep = m_videoSteps[vp1];
			int frameCount = static_cast<int>( fps * currentStep.duration_sec );
			totalFrameCount += frameCount;

			//take care of the 'loop' case
			if (vp2 == 0)
			{
				assert(loopCheckBox->isChecked());
				break;
			}
			vp1 = vp2;
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

	size_t vp1 = previewFromSelectedCheckBox->isChecked() ? static_cast<size_t>(getCurrentStepIndex()) : 0;

	//count the total number of frames
	int frameCount = countFrames(loopCheckBox->isChecked() ? 0 : vp1);
	int fps = fpsSpinBox->value();

	//show progress dialog
	QProgressDialog progressDialog(QString("Frames: %1").arg(frameCount), "Cancel", 0, frameCount, this);
	progressDialog.setWindowTitle("Preview");
	progressDialog.show();
	progressDialog.setModal(true);
	progressDialog.setAutoClose(false);
	QApplication::processEvents();

	assert(stepSelectionList->count() >= m_videoSteps.size());

	int frameIndex = 0;
	size_t vp2 = 0;
	while (getNextSegment(vp1, vp2))
	{
		Step& step1 = m_videoSteps[vp1];
		Step& step2 = m_videoSteps[vp2];

		//theoretical waiting time per frame
		qint64 delay_ms = static_cast<int>(1000 * step1.duration_sec / fps);
		int frameCount = static_cast<int>( fps * step1.duration_sec );

		ViewInterpolate interpolator(step1.viewport, step2.viewport);
		interpolator.setMaxStep(frameCount);
		cc2DViewportObject currentParams;
		while ( interpolator.nextView( currentParams ) )
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
		if (progressDialog.wasCanceled())
		{
			break;
		}

		if (vp2 == 0)
		{
			assert(loopCheckBox->isChecked());
			frameIndex = 0;
		}
		vp1 = vp2;
	}

	//reset view
	onCurrentStepChanged(getCurrentStepIndex());

	setEnabled(true);
}

void qAnimationDlg::render(bool asSeparateFrames)
{
	if (!m_view3d)
	{
		assert(false);
		return;
	}
	QString outputFilename = outputFileLineEdit->text();

	//save to persistent settings
	{
		QSettings settings;
		settings.beginGroup("qAnimation");
		settings.setValue("filename", outputFilename);
		settings.endGroup();
	}

	setEnabled(false);

	//count the total number of frames
	int frameCount = countFrames(0);
	int fps = fpsSpinBox->value();

	//super resolution
	int superRes = superResolutionSpinBox->value();
	const int SUPER_RESOLUTION = 0;
	const int ZOOM = 1;
	int renderingMode = renderingModeComboBox->currentIndex();
	assert(renderingMode == SUPER_RESOLUTION || renderingMode == ZOOM);

	//show progress dialog
	QProgressDialog progressDialog(QString("Frames: %1").arg(frameCount), "Cancel", 0, frameCount, this);
	progressDialog.setWindowTitle("Render");
	progressDialog.show();
	QApplication::processEvents();

#ifdef QFFMPEG_SUPPORT
	QScopedPointer<QVideoEncoder> encoder(0);
	QSize originalViewSize;
	if (!asSeparateFrames)
	{
		//get original viewport size
		originalViewSize = m_view3d->qtSize();

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

		int bitrate = bitrateSpinBox->value() * 1024;
		int gop = fps;
		int animScale = 1;
		if (renderingMode == ZOOM)
		{
			animScale = superRes;
		}
		encoder.reset(new QVideoEncoder(outputFilename, m_view3d->glWidth() * animScale, m_view3d->glHeight() * animScale, bitrate, gop, static_cast<unsigned>(fpsSpinBox->value())));
		QString errorString;
		if (!encoder->open(&errorString))
		{
			QMessageBox::critical(this, "Error", QString("Failed to open file for output: %1").arg(errorString));
			setEnabled(true);
			return;
		}
	}
#else
	if (!asSeparateFrames)
	{
		QMessageBox::critical(this, "Error", QString("Animation mode is not supported (no FFMPEG support)"));
		return;
	}
#endif

	bool lodWasEnabled = m_view3d->isLODEnabled();
	m_view3d->setLODEnabled(false);

	QDir outputDir( QFileInfo(outputFilename).absolutePath() );

	int frameIndex = 0;
	bool success = true;
	size_t vp1 = 0;
	size_t vp2 = 0;
	while (getNextSegment(vp1, vp2))
	{
		Step& step1 = m_videoSteps[vp1];
		Step& step2 = m_videoSteps[vp2];

		ViewInterpolate interpolator(step1.viewport, step2.viewport);
		int frameCount = static_cast<int>( fps * step1.duration_sec );
		interpolator.setMaxStep(frameCount);

		cc2DViewportObject current_params;
		while ( interpolator.nextView( current_params ) )
		{
			applyViewport ( &current_params );

			//render to image
			QImage image = m_view3d->renderToImage(superRes, renderingMode == ZOOM, false, true );

			if (image.isNull())
			{
				QMessageBox::critical(this, "Error", "Failed to grab the screen!");
				success = false;
				break;
			}

			if (renderingMode == SUPER_RESOLUTION && superRes > 1)
			{
				image = image.scaled(image.width() / superRes, image.height() / superRes, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
			}

			if (asSeparateFrames)
			{
				QString filename = QString("frame_%1.png").arg(frameIndex, 6, 10, QChar('0'));
				QString fullPath = outputDir.filePath(filename);
				if (!image.save(fullPath))
				{
					QMessageBox::critical(this, "Error", QString("Failed to save frame #%1").arg(frameIndex + 1));
					success = false;
					break;
				}
			}
			else
			{
#ifdef QFFMPEG_SUPPORT
				QString errorString;
				if (!encoder->encodeImage(image, frameIndex, &errorString))
				{
					QMessageBox::critical(this, "Error", QString("Failed to encode frame #%1: %2").arg(frameIndex + 1).arg(errorString));
					success = false;
					break;
				}
#endif
			}
			++frameIndex;
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

		if (vp2 == 0)
		{
			//stop loop here!
			break;
		}
		vp1 = vp2;
	}

	m_view3d->setLODEnabled(lodWasEnabled);

#ifdef QFFMPEG_SUPPORT
	if (encoder)
	{
		encoder->close();

		//hack: restore original size
		m_view3d->resize(originalViewSize);
		QApplication::processEvents();
	}
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

	stepTimeDoubleSpinBox->blockSignals(true);
	stepTimeDoubleSpinBox->setValue(m_videoSteps[index].duration_sec);
	stepTimeDoubleSpinBox->blockSignals(false);
}

void qAnimationDlg::onItemChanged(QListWidgetItem*)
{
	//update total duration
	updateTotalDuration();

	onCurrentStepChanged(stepSelectionList->currentRow());
}

void qAnimationDlg::onCurrentStepChanged(int index)
{
	//update current step descriptor
	stepIndexLabel->setText(QString::number(index+1));

	updateCurrentStepDuration();

	applyViewport( m_videoSteps[index].viewport );

	//check that the step is enabled
	bool isEnabled = (stepSelectionList->item(index)->checkState() == Qt::Checked);
	bool isLoop = loopCheckBox->isChecked();
	currentStepGroupBox->setEnabled(isEnabled && (index+1 < m_videoSteps.size() || isLoop));
}

void qAnimationDlg::onLoopToggled(bool)
{
	updateTotalDuration();

	onCurrentStepChanged(stepSelectionList->currentRow());
}
