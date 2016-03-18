//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qKinect                     #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################

#include "qKinect.h"

//qCC_db
#include <ccColorTypes.h>
#include <ccPointCloud.h>
#include <ccGLMatrix.h>
#include <ccGBLSensor.h>

//dialog
#include "ccKinectDlg.h"

//Qt
#include <QElapsedTimer>
#include <QMainWindow>
#include <QtGui>

//Libfreenect
#include <libfreenect.h>

//System
#include <string.h>

qKinect::qKinect(QObject* parent/*=0*/)
	: QObject(parent)
	, ccStdPluginInterface()
	, m_kDlg(0)
	, m_timer(0)
	, m_action(0)
{
}

qKinect::~qKinect()
{
	if (m_kDlg)
		delete m_kDlg;
}

QIcon qKinect::getIcon() const
{
	return QIcon(QString::fromUtf8(":/CC/plugin/qKinect/libfreenect_small.png"));
}

void qKinect::getActions(QActionGroup& group)
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doStartGrabbing()));
	}

	group.addAction(m_action);
}

//void qKinect::onNewSelection(const ccHObject::Container& selectedEntities)
//{
//	if (m_action)
//		m_action->setEnabled(true);
//}

static uint16_t *s_depth_data = 0;
static unsigned s_wDepth = 0;
static unsigned s_hDepth = 0;
static unsigned s_depth_count = 0;
static unsigned s_max_depth_count = 0;
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	if (s_depth_data && s_depth_count<s_max_depth_count)
	{
		uint16_t *depth = (uint16_t*)v_depth;
		unsigned mapSize = s_wDepth*s_hDepth;
		memcpy(s_depth_data + s_depth_count*mapSize, depth, mapSize*sizeof(uint16_t));
		++s_depth_count;
	}
}

static uint8_t *s_last_rgb_data = 0;
static unsigned s_wRgb = 0;
static unsigned s_hRgb = 0;
static unsigned s_rgb_count = 0; //max=1
void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	if (s_last_rgb_data && s_rgb_count==0)
	{
		uint8_t *rgbMap = (uint8_t*)rgb;
		memcpy(s_last_rgb_data,rgbMap,sizeof(uint8_t)*s_wRgb*s_hRgb*3);
		++s_rgb_count;
	}
}

//helper
bool getResolution(freenect_resolution& resolution, unsigned& w, unsigned &h)
{
	w=h=0;
	switch (resolution)
	{
		case FREENECT_RESOLUTION_LOW:
			w = 320;
			h = 240;
			break;
		case FREENECT_RESOLUTION_MEDIUM:
			w = 640;
			h = 480;
			break;
		case FREENECT_RESOLUTION_HIGH:
			w = 1280;
			h = 1024;
			break;
		default:
			//invalid mode
			return false;
	}
	return true;
}


//FIXME: should be qKinect class' arguments!
static freenect_context* f_ctx = 0;
static freenect_device* f_dev = 0;
static unsigned s_grabIndex = 0;

void qKinect::doStartGrabbing()
{
	assert(m_app);
	if (!m_app)
		return;

	f_ctx = 0;
	f_dev = 0;
	s_grabIndex = 0;

	if (m_kDlg)
		delete m_kDlg;
	m_kDlg = 0;

	s_max_depth_count = 0;
	if (s_depth_data)
		delete[] s_depth_data;
	s_depth_count = 0;
	s_depth_data = 0;
	s_wDepth = s_hDepth = 0;

	if (s_last_rgb_data)
		delete[] s_last_rgb_data;
	s_last_rgb_data = 0;
	s_rgb_count = 0;
	s_wRgb = s_hRgb = 0;

	if (freenect_init(&f_ctx, NULL) < 0)
	{
		m_app->dispToConsole("[qKinect] Failed to initialize kinect driver!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//freenect_set_log_level(f_ctx, FREENECT_LOG_FATAL);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices(f_ctx);
	m_app->dispToConsole(qPrintable(QString("[qKinect] Number of devices found: %1").arg(nr_devices)));

	if (nr_devices < 1)
	{
		m_app->dispToConsole(QString("[qKinect] No device found"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	int returnCode = freenect_open_device(f_ctx, &f_dev, 0);
	if (returnCode < 0)
	{
		m_app->dispToConsole(QString("[qKinect] Failed to initialize kinect device! (error code: %1)").arg(returnCode),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//test: try to init high resolution mode
	//freenect_frame_mode upDepthMode = freenect_find_depth_mode(FREENECT_RESOLUTION_HIGH, FREENECT_DEPTH_11BIT);
	//int success = freenect_set_depth_mode(f_dev,upDepthMode);

	/*** Depth information ***/
	freenect_frame_mode depthMode = freenect_get_current_depth_mode(f_dev);
	if (!depthMode.depth_format == FREENECT_DEPTH_11BIT)
	{
		depthMode = freenect_find_depth_mode(depthMode.resolution, FREENECT_DEPTH_11BIT);
		if (freenect_set_depth_mode(f_dev,depthMode)<0)
		{
			m_app->dispToConsole("[qKinect] Failed to initialiaze depth mode!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}
	if (!getResolution(depthMode.resolution,s_wDepth,s_hDepth))
	{
		m_app->dispToConsole("[qKinect] Failed to read depth resolution!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	m_app->dispToConsole(qPrintable(QString("[qKinect] Depth resolution: %1 x %2").arg(s_wDepth).arg(s_hDepth)));

	s_depth_data = new uint16_t[s_wDepth*s_hDepth];
	if (!s_depth_data)
	{
		m_app->dispToConsole("[qKinect] Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	s_max_depth_count = 1;

	/*** RGB information ***/
	bool grabRGB = true;
	{
		freenect_frame_mode rgbMode = freenect_get_current_video_mode(f_dev);
		if (!rgbMode.video_format == FREENECT_VIDEO_RGB || depthMode.resolution != rgbMode.resolution)
		{
			rgbMode = freenect_find_video_mode(depthMode.resolution, FREENECT_VIDEO_RGB);
			if (freenect_set_video_mode(f_dev,rgbMode)<0)
			{
				m_app->dispToConsole("[qKinect] Can't find a video mode compatible with current depth mode?!");
				grabRGB = false;
			}
		}

		//still want to/can grab RGB info?
		if (grabRGB)
		{
			getResolution(rgbMode.resolution,s_wRgb,s_hRgb);

			s_last_rgb_data = new uint8_t[s_wRgb*s_hRgb*3];
			if (!s_last_rgb_data) //not enough memory for RGB
			{
				m_app->dispToConsole("[qKinect] Not enough memory to grab RGB info!");
				grabRGB = false;
			}
			else
			{
				m_app->dispToConsole(qPrintable(QString("[qKinect] RGB resolution: %1 x %2").arg(s_wRgb).arg(s_hRgb)));
			}
		}
	}

	int freenect_angle = 0;
	freenect_set_tilt_degs(f_dev,freenect_angle);
	freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);
	if (grabRGB)
		freenect_set_video_buffer(f_dev, s_last_rgb_data);

	freenect_start_depth(f_dev);
	if (s_last_rgb_data)
		freenect_start_video(f_dev);

	m_kDlg = new ccKinectDlg(m_app->getMainWindow());
	if (grabRGB)
		m_kDlg->addMode(QString("%1 x %2").arg(s_wDepth).arg(s_hDepth));
	else
		m_kDlg->grabRGBCheckBox->setChecked(false);
	m_kDlg->grabRGBCheckBox->setEnabled(grabRGB);
	m_kDlg->grabPushButton->setEnabled(true);

	connect(m_kDlg->grabPushButton, SIGNAL(clicked()), this, SLOT(grabCloud()));
	connect(m_kDlg, SIGNAL(finished(int)), this, SLOT(dialogClosed(int)));

	//m_kDlg->setModal(false);
	//m_kDlg->setWindowModality(Qt::NonModal);
	m_kDlg->show();

	if (!m_timer)
	{
		m_timer = new QTimer(this);
		connect(m_timer, SIGNAL(timeout()), this, SLOT(updateRTView()));
	}
	m_timer->start(0);
}

void qKinect::updateRTView()
{
	if (!m_kDlg)
		return;

	s_depth_count = 0;
	s_rgb_count = 0;

	freenect_process_events(f_ctx);

	if (s_last_rgb_data && s_rgb_count)
	{
		QPixmap pixmap = QPixmap::fromImage(QImage(s_last_rgb_data,s_wDepth,s_hDepth,QImage::Format_RGB888));
		m_kDlg->view2D->setPixmap(pixmap);
	}

	if (s_depth_data && s_depth_count)
	{
		QImage image(s_wDepth,s_hDepth,QImage::Format_RGB888);

		//convert depth array to image
		const uint16_t* _depth = s_depth_data;
		unsigned char* _bits = image.bits();
		for (unsigned i=0;i<s_hDepth*s_wDepth;++i,++_depth,_bits+=3)
		{
			if (*_depth < FREENECT_DEPTH_RAW_NO_VALUE)
			{
				//see http://openkinect.org/wiki/Imaging_Information
				float z = 12.36f * tanf((float)(*_depth) / 2842.5f + 1.1863f) - 3.7f;
				//HSV --> V=1, S=1, H = cycling
				ccColor::Rgb rgb = ccColor::Convert::hsv2rgb((abs((int)z*10))%360,1.0,1.0);
				_bits[0] = rgb.r;
				_bits[1] = rgb.g;
				_bits[2] = rgb.b;
			}
			else
			{
				_bits[0] = _bits[1] = _bits[2] = 0;
			}
		}

		m_kDlg->viewDepth->setPixmap(QPixmap::fromImage(image));
	}
}

void qKinect::grabCloud()
{
	assert(f_ctx && f_dev);
	assert(m_kDlg);

	if (m_timer)
	{
		m_timer->stop();
		QApplication::processEvents();
	}

	bool grabRGB = m_kDlg->grabRGBInfo();
	unsigned char framesAvgCount = m_kDlg->getFrameAveragingCount();

	uint16_t* old_depth_data = s_depth_data;
	const unsigned mapSize = s_wDepth*s_hDepth;
	
	if (framesAvgCount>1)
	{
		uint16_t* depth_frames = new uint16_t[mapSize*framesAvgCount];
		if (!depth_frames)
		{
			m_app->dispToConsole("[qKinect] Not enough memory to compute frame averaging!");
			framesAvgCount=1;
		}
		else
		{
			s_depth_data = depth_frames;
		}
	}

	//Flush buffers
	s_max_depth_count = 0;
	s_depth_count = 1;
	s_rgb_count = 1;
	freenect_process_events(f_ctx);

	//Start
	QElapsedTimer eTimer;
	eTimer.start();

	s_rgb_count = 0;
	s_depth_count = 0;
	s_max_depth_count = framesAvgCount;

	while (s_depth_count<framesAvgCount || (grabRGB && s_rgb_count==0))
	{
		freenect_process_events(f_ctx);

		if(s_depth_count == 0 && eTimer.elapsed() > 5000 ) //timeout 5s. without any  data
			break;
	}

	//success?
	if (s_depth_count)
	{
		const unsigned mapSize = s_hDepth*s_wDepth;
		
		//first, pocess frame averaging (if any)
		if (s_depth_count>1)
		{
			const unsigned char minFrameCount = (framesAvgCount>>1); //a pixel must be visible in at least half the frames!
			uint16_t *_depth = s_depth_data;
			for (unsigned i = 0; i < mapSize; ++i,++_depth)
			{
				//sum all equivalent pixels
				unsigned pixCount = 0;
				double pixSum = 0.0;
				const uint16_t* kDepth = s_depth_data + i;
				for (unsigned k=0;k<s_depth_count;++k,kDepth+=mapSize)
				{
					if (*kDepth < FREENECT_DEPTH_RAW_NO_VALUE)
					{
						pixSum += (double)s_depth_data[k*mapSize + i];
						++pixCount;
					}
				}

				if (pixCount>=minFrameCount)
				{
					*_depth = (uint16_t)(pixSum/(double)pixCount);
				}
				else
				{
					*_depth = FREENECT_DEPTH_RAW_NO_VALUE;
				}
			}
		}

		/*** Depth calibration info ***/

		//see http://openkinect.org/wiki/Imaging_Information
		/*static const float minDistance = -10.0f;
		static const float scaleFactor = .0021f;
		const float cx = (float)w/2;
		const float cy = (float)h/2;
		const float fx = 1.0f/scaleFactor; //* (480.0/640.0);
		const float fy = 1.0f/scaleFactor; //~476
		//*/

		//see http://nicolas.burrus.name/index.php/Research/KinectCalibration
		static const float minDistance = -10.0f;
		static const float cx = 339.5f;
		static const float cy = 242.7f;
		static const float fx = 594.21f;
		static const float fy = 591.04f;
		//*/

		/*** RGB calibration info ***/
		static const float fx_rgb = 529.21508098293293f;
		static const float fy_rgb = 525.56393630057437f;
		static const float cx_rgb = 328.94272028759258f;
		static const float cy_rgb = 267.48068171871557f;

		float mat[16] = {	 9.9984628826577793e-01f, 1.2635359098409581e-03f, -1.7487233004436643e-02f,  1.9985242312092553e-02f,
							-1.4779096108364480e-03f, 9.9992385683542895e-01f, -1.2251380107679535e-02f, -7.4423738761617583e-04f,
							1.7470421412464927e-02f, 1.2275341476520762e-02f,  9.9977202419716948e-01f, -1.0916736334336222e-02f,
							0.0f, 0.0f, 0.0f, 1.0f};

		ccGLMatrix depth2rgb(mat);
		depth2rgb.transpose();

		ccPointCloud* depthMap = new ccPointCloud();
		bool hasRGB = s_rgb_count && s_last_rgb_data && m_kDlg->grabRGBCheckBox->isChecked();
		if (depthMap->reserve(mapSize))
		{
			if (hasRGB)
			{
				if (depthMap->reserveTheRGBTable())
					depthMap->showColors(true);
				else
				{
					m_app->dispToConsole("[qKinect] Not enough memory to grab colors!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
					hasRGB=false;
				}
			}

			const uint16_t *depth = s_depth_data;
			const uint8_t* col = 0;
			const uint8_t white[3]={255,255,255};
			CCVector3 P,Q;

			bool meanFilter = m_kDlg->meanFilterCheckBox->isChecked();

			for (unsigned j=0; j<s_hDepth; ++j)
			{
				//P.y = (PointCoordinateType)j;
				for (unsigned i=0; i<s_wDepth; ++i,++depth)
				{
					uint16_t d = *depth;

					//mean filter
					if (meanFilter)
					{
						double sum = 0.0;
						unsigned count = 0;
						for (int k=-1; k<=1; ++k)
						{
							int ii = static_cast<int>(i)+k;
							if (ii>=0 && ii<static_cast<int>(s_wDepth))
								for (int l=-1;l<=1;++l)
								{
									int jj = static_cast<int>(j)+l;
									if (jj>=0 && jj<static_cast<int>(s_hDepth))
									{
										const uint16_t& dd = s_depth_data[jj*s_wDepth+ii];
										if (dd < FREENECT_DEPTH_RAW_NO_VALUE)
										{
											sum += static_cast<double>(s_depth_data[jj*s_wDepth+ii]);
											++count;
										}
									}
								}
						}

						if (count > 1)
							d = static_cast<uint16_t>(sum/count);
					}

					if (d < FREENECT_DEPTH_RAW_NO_VALUE)
					{
						//see http://openkinect.org/wiki/Imaging_Information
						P.z = 12.36f * tanf(static_cast<float>(d) / 2842.5f + 1.1863f) - 3.7f;
						//see http://nicolas.burrus.name/index.php/Research/KinectCalibration
						P.x = (static_cast<float>(i) - cx) * (P.z + minDistance) / fx;
						P.y = (static_cast<float>(j) - cy) * (P.z + minDistance) / fy ;

						if (hasRGB)
						{
							assert(s_last_rgb_data);

							Q = depth2rgb * P;

							Q.x = (Q.x * fx_rgb / Q.z) + cx_rgb;
							Q.y = (Q.y * fy_rgb / Q.z) + cy_rgb;
							int i_rgb = (int)Q.x;
							int j_rgb = (int)Q.y;
							if (i_rgb>=0 && i_rgb<(int)s_wDepth && j_rgb>=0 && j_rgb<(int)s_hDepth)
							{
								col = s_last_rgb_data+(i_rgb+j_rgb*s_wRgb)*3;
							}
							else
							{
								col = white;
							}
						}

						P.y = -P.y;
						P.z = -P.z;
						depthMap->addPoint(P);

						if (col)
							depthMap->addRGBColor(col);
					}
				}
			}

			//m_app->dispToConsole(QString("Cloud captured: %1 points").arg(depthMap->size()));

			depthMap->resize(depthMap->size());
			QString cloudName = m_kDlg->getCloudName() + QString::number(++s_grabIndex);
			depthMap->setName(qPrintable(cloudName));
			//associate sensor
			ccGBLSensor* sensor = new ccGBLSensor(ccGBLSensor::YAW_THEN_PITCH);
			ccGLMatrix rot;
			{
				float* mat = rot.data();
				mat[0] =  1.0f;
				mat[1] =  0.0f;
				mat[2] =  0.0f;

				mat[4] =  0.0f;
				mat[5] =  0.0f;
				mat[6] = -1.0f;

				mat[8] =  0.0f;
				mat[9] =  1.0f;
				mat[10] = 0.0f;

				mat[15] = 1.0f;
			}
			sensor->setRigidTransformation(rot);
			sensor->setYawStep(0.0017f);
			sensor->setPitchStep(0.0017f);
			sensor->setUncertainty(1e-3f);
			{
				if (sensor->computeAutoParameters(depthMap))
				{
					sensor->setName("Kinect");
					sensor->setGraphicScale(20.0f);
					sensor->setVisible(true);
					depthMap->addChild(sensor);
				}
				else
				{
					delete sensor;
					sensor = 0;
				}
			}
			//selectedEntities.push_back(depthMap);
			m_app->addToDB(depthMap,false,true,true);
			m_app->refreshAll();
		}
		else
		{
			//not enough memory
			delete depthMap;
			depthMap = 0;
			//result = -5;
		}
	}
	else
	{
		//no data!
		//result = -6;
	}

	//restore 'old' buffer
	s_max_depth_count = 1;
	if (old_depth_data != s_depth_data)
	{
		delete[] s_depth_data;
		s_depth_data = old_depth_data;
	}

	if (m_timer)
		m_timer->start(0);
}

void qKinect::dialogClosed(int result)
{
	assert(f_ctx && f_dev);
	assert(m_kDlg);

	//if (result != 0)
	//	grabCloud();

	if (m_timer)
		m_timer->stop();

	freenect_stop_depth(f_dev);
	if (s_last_rgb_data)
		freenect_stop_video(f_dev);
	freenect_set_led(f_dev,LED_BLINK_GREEN);
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	s_max_depth_count = 0;
	if (s_depth_data)
		delete[] s_depth_data;
	s_depth_data = 0;
	s_depth_count = 0;

	s_rgb_count=1;
	if (s_last_rgb_data)
		delete[] s_last_rgb_data;
	s_last_rgb_data = 0;
	s_rgb_count = 0;
}
