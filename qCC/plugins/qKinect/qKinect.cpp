//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qKinect                        #
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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1773                                                              $
//$LastChangedDate:: 2011-01-31 17:14:06 +0100 (lun., 31 janv. 2011)       $
//**************************************************************************
//

#include <QtGui>

#include "qKinect.h"
//#include "ccHprDlg.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccGLMatrix.h>

//dialog
#include "ccKinectDlg.h"

//Qt
#include <QElapsedTimer>

//Libfreenect
#include <libfreenect.h>

void qKinect::getDescription(ccPluginDescription& desc)
{
    strcpy(desc.name,"Kinect 3D stream capture (with libfreenect)");
    strcpy(desc.menuName,"qKinect");
    desc.hasAnIcon=true;
    desc.version=1;
}

bool qKinect::onNewSelection(const ccHObject::Container& selectedEntities)
{
    return true;
}

uint16_t *depth_mid = 0;
unsigned wDepth = 0;
unsigned hDepth = 0;
unsigned got_depth = 0;
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    if (depth_mid && got_depth==0)
    {
        uint16_t *depth = (uint16_t*)v_depth;
        memcpy(depth_mid,depth,sizeof(uint16_t)*wDepth*hDepth);
        ++got_depth;
    }
}

uint8_t *rgb_mid = 0;
unsigned wRgb = 0;
unsigned hRgb = 0;
unsigned got_rgb = 0;
void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
    if (rgb_mid && got_rgb==0)
    {
        uint8_t *rgbMap = (uint8_t*)rgb;
        memcpy(rgb_mid,rgbMap,sizeof(uint8_t)*wRgb*hRgb*3);
        ++got_rgb;
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


int qKinect::doAction(ccHObject::Container& selectedEntities,
                            unsigned& uiModificationFlags,
                            ccProgressDialog* progressCb/*=NULL*/,
                            QWidget* parent/*=NULL*/)
{
    freenect_context* f_ctx=0;
	if (freenect_init(&f_ctx, NULL) < 0)
		return -1;

    freenect_device *f_dev=0;

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);

	int nr_devices = freenect_num_devices (f_ctx);
    if (m_app)
        m_app->dispToConsole(qPrintable(QString("[qKinect] Number of devices found: %1").arg(nr_devices)));

	if (nr_devices < 1)
		return -2;

	if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
		return -3;

	//test: try to init high resolution mode
	//freenect_frame_mode upDepthMode = freenect_find_depth_mode(FREENECT_RESOLUTION_HIGH, FREENECT_DEPTH_11BIT);
	//int success = freenect_set_depth_mode(f_dev,upDepthMode);

	/*** Depth information ***/

	depth_mid = 0;
	got_depth = 0;
	wDepth = hDepth = 0;

	freenect_frame_mode depthMode = freenect_get_current_depth_mode(f_dev);
	if (!depthMode.depth_format == FREENECT_DEPTH_11BIT)
	{
		depthMode = freenect_find_depth_mode(depthMode.resolution, FREENECT_DEPTH_11BIT);
		if (freenect_set_depth_mode(f_dev,depthMode)<0)
			return -4;
	}
	if (!getResolution(depthMode.resolution,wDepth,hDepth))
		return -4; //invalid mode
    if (m_app)
		m_app->dispToConsole(qPrintable(QString("[qKinect] Depth resolution: %1 x %2").arg(wDepth).arg(hDepth)));

	ccKinectDlg kDlg(parent);
	kDlg.addMode(QString("%1 x %2").arg(wDepth).arg(hDepth));
	if (!kDlg.exec())
		return 0;

	depth_mid = new uint16_t[wDepth*hDepth];
	if (!depth_mid)
		return -5; //not enough memory

	/*** RGB information ***/

	rgb_mid = 0;
	got_rgb = 0;
	wRgb = hRgb = 0;
	bool grabRGB = kDlg.grabRGBInfo();
	if (grabRGB)
	{
		freenect_frame_mode rgbMode = freenect_get_current_video_mode(f_dev);
		if (!rgbMode.video_format == FREENECT_VIDEO_RGB || depthMode.resolution != rgbMode.resolution)
		{
			rgbMode = freenect_find_video_mode(depthMode.resolution, FREENECT_VIDEO_RGB);
			if (freenect_set_video_mode(f_dev,rgbMode)<0)
			{
				if (m_app)
					m_app->dispToConsole("[qKinect] Can't find a video mode compatible with depth mode?!");
				grabRGB = false;
			}
		}

		//still want to/can grab RGB info?
		if (grabRGB)
		{
			getResolution(rgbMode.resolution,wRgb,hRgb);

			rgb_mid = new uint8_t[wRgb*hRgb*3];
			if (!rgb_mid) //not enough memory for RGB
			{
				if (m_app)
					m_app->dispToConsole("[qKinect] Not enough memory to grab RGB info!");
				grabRGB = false;
			}
			else
			{
				if (m_app)
					m_app->dispToConsole(qPrintable(QString("[qKinect] RGB resolution: %1 x %2").arg(wRgb).arg(hRgb)));
			}
		}
	}

    int freenect_angle = 0;
	freenect_set_tilt_degs(f_dev,freenect_angle);
	freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);
	if (grabRGB)
		freenect_set_video_buffer(f_dev, rgb_mid);

	freenect_start_depth(f_dev);
	if (rgb_mid)
		freenect_start_video(f_dev);

	unsigned char framesAvgCount = kDlg.getFrameAveragingCount();
	unsigned char* counters = 0;
	double* avgDepth = 0;
	if (framesAvgCount>1)
	{
		counters = new unsigned char[wDepth*hDepth];
		avgDepth = new double[wDepth*hDepth];
		if (!counters || !avgDepth)
		{
			if (m_app)
				m_app->dispToConsole("[qKinect] Not enough memory to compute frame averaging!");
			framesAvgCount=1;
		}
		else
		{
			memset(counters,0,sizeof(unsigned char)*wDepth*hDepth);
			memset(avgDepth,0,sizeof(double)*wDepth*hDepth);
		}
	}

	QElapsedTimer eTimer;
	eTimer.start();

	unsigned char framesCount = 0;
	while (got_depth==0 || (grabRGB && got_rgb==0))
	{
	    freenect_process_events(f_ctx);

		if (got_depth == 1 && framesAvgCount>1)
		{
			//copy pixels
			const uint16_t *_depth = depth_mid;
			double* _avgDepth = avgDepth;
			unsigned char* _counters = counters;
			for (unsigned j = 0; j < hDepth; ++j)
			{
				for (unsigned i = 0; i < wDepth; ++i,++_depth,++_avgDepth,++_counters)
				{
					if (*_depth < FREENECT_DEPTH_RAW_NO_VALUE)
					{
						*_avgDepth += (double)*_depth;
						++(*_counters);
					}
				}
			}
			
			if (++framesCount<framesAvgCount)
				got_depth = 0;
		}

		if( framesCount == 0 && eTimer.elapsed() > 5000 ) //timeout 5s.
            break;
	}

	freenect_stop_depth(f_dev);
	if (grabRGB)
		freenect_stop_video(f_dev);
	freenect_set_led(f_dev,LED_BLINK_GREEN);
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	//finish frame averaging
	if (framesAvgCount>1)
	{
		unsigned char minFrames = (framesAvgCount>>1); //a pixel must be visible in at least half the frames!
		uint16_t *_depth = depth_mid;
		const double* _avgDepth = avgDepth;
		const unsigned char* _counters = counters;
		for (unsigned j = 0; j < hDepth; ++j)
		{
			for (unsigned i = 0; i < wDepth; ++i,++_depth,++_avgDepth,++_counters)
			{
				if (*_counters >= minFrames)
				{
					*_depth = (uint16_t)(*_avgDepth/(double)*_counters);
				}
				else
				{
					*_depth = FREENECT_DEPTH_RAW_NO_VALUE;
				}
			}
		}
	}

	int result = 1;

	//capture
    if (got_depth)
    {
        /*** Depth calibration info ***/

        //see http://openkinect.org/wiki/Imaging_Information
        /*static const double minDistance = -10.0;
        static const double scaleFactor = .0021;
        const float cx = (float)w/2;
        const float cy = (float)h/2;
        const float fx = 1.0/scaleFactor; //* (480.0/640.0);
        const float fy = 1.0/scaleFactor; //~476
        //*/

        //see http://nicolas.burrus.name/index.php/Research/KinectCalibration
        static const double minDistance = -10.0;
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

        float mat[16]={9.9984628826577793e-01f, 1.2635359098409581e-03f, -1.7487233004436643e-02f, 1.9985242312092553e-02f,
                        -1.4779096108364480e-03f, 9.9992385683542895e-01f, -1.2251380107679535e-02f, -7.4423738761617583e-04f,
                        1.7470421412464927e-02f, 1.2275341476520762e-02f, 9.9977202419716948e-01f, -1.0916736334336222e-02f,
                        0.0f, 0.0f, 0.0f, 1.0f};

        ccGLMatrix depth2rgb(mat);
        depth2rgb.transpose();

        ccPointCloud* depthMap = new ccPointCloud();
        if (depthMap->reserve(wDepth*hDepth))
		{
			if (got_rgb)
			{
				depthMap->reserveTheRGBTable();
				depthMap->showColors(true);
			}

			const uint16_t *depth = depth_mid;
			const uint8_t* col = 0;
			const uint8_t white[3]={255,255,255};
			CCVector3 P,Q;

			for (unsigned j = 0; j < hDepth; ++j)
			{
				//P.y = (PointCoordinateType)j;
				for (unsigned i = 0; i < wDepth; ++i,++depth)
				{
					if (*depth < FREENECT_DEPTH_RAW_NO_VALUE)
					{
						//see http://openkinect.org/wiki/Imaging_Information
						P.z = 12.36f * tanf((float)*depth / 2842.5f + 1.1863f) - 3.7f;
						//see http://nicolas.burrus.name/index.php/Research/KinectCalibration
						P.x = ((float)i - cx) * (P.z + minDistance) / fx;
						P.y = ((float)j - cy) * (P.z + minDistance) / fy ;

						if (got_rgb)
						{
							assert(rgb_mid);

							Q = depth2rgb * P;

							Q.x = (Q.x * fx_rgb / Q.z) + cx_rgb;
							Q.y = (Q.y * fy_rgb / Q.z) + cy_rgb;
							int i_rgb = (int)Q.x;
							int j_rgb = (int)Q.y;
							if (i_rgb>=0 && i_rgb<(int)wDepth && j_rgb>=0 && j_rgb<(int)hDepth)
							{
								col = rgb_mid+(i_rgb+j_rgb*wRgb)*3;
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

			//if (m_app)
			//    m_app->dispToConsole(QString("Cloud captured: %1 points").arg(depthMap->size()));

			depthMap->resize(depthMap->size());
			depthMap->setName(qPrintable(kDlg.getCloudName()));
			selectedEntities.push_back(depthMap);
		}
		else
		{
			//not enough memory
			delete depthMap;
			depthMap=0;
			result = -5;
		}
    }
	else
	{
		//no data!
		result = -6;
	}

	if (depth_mid)
		delete[] depth_mid;
    depth_mid=0;

	if (rgb_mid)
		delete[] rgb_mid;
    rgb_mid=0;

	if (counters)
		delete[] counters;
	counters = 0;

	if (avgDepth)
		delete[] avgDepth;
	avgDepth = 0;

    //currently selected entities appearance may have changed!
    uiModificationFlags |= CC_PLUGIN_REFRESH_GL_WINDOWS;
    //*/

    return result;
}

QString qKinect::getErrorMessage(int errorCode/*, LANGUAGE lang*/)
{
    QString errorMsg;
    switch(errorCode)
    {
        case -1:
            errorMsg=QString("Failed to init driver!");
            break;
        case -2:
            errorMsg=QString("No device found");
            break;
        case -3:
            errorMsg=QString("Could not open device");
            break;
        case -4:
            errorMsg=QString("Unsupported depth/video mode");
            break;
        case -5:
            errorMsg=QString("Not enough memory");
            break;
        case -6:
            errorMsg=QString("No data!");
            break;
        default:
            errorMsg=QString("Undefined error!");
            break;
    }
    return errorMsg;
}

QIcon qKinect::getIcon() const
{
    return QIcon(QString::fromUtf8(":/CC/plugin/qKinect/libfreenect_small.png"));
}

Q_EXPORT_PLUGIN2(qKinect,qKinect);
