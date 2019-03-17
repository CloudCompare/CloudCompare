//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//

//PCL
#include <pcl/keypoints/sift_keypoint.h>

#include <ccIncludeGL.h> //Always first normally, but it imports 'min' and 'max' definitions that clash with PCL ones (DGM: from where?)

#include "ExtractSIFT.h"

//Local
#include "dialogs/SIFTExtractDlg.h"
#include "../utils/PCLConv.h"
#include "../utils/cc2sm.h"
#include "../utils/sm2cc.h"

//qCC_db
#include <ccPointCloud.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QMainWindow>

//System
#include <iostream>
#include <sstream>

//! Extract SIFT keypoints
/** if only the point cloud is given PCL default parameters are used (that are not really good, so please give parameters)
	\note Different types can be passed as input for this function:
		- PointXYZI
		- PointNormal
		- PointXYZRGB
	\note If a PointType with a scale field is passed as output type, scales will be returned together with the return cloud
**/
template <typename PointInT, typename PointOutT>
int estimateSIFT(	const typename pcl::PointCloud<PointInT>::Ptr in_cloud,
					typename pcl::PointCloud<PointOutT>::Ptr out_cloud,
					int nr_octaves = 0,
					float min_scale = 0,
					int nr_scales_per_octave = 0,
					float min_contrast = 0)
{
	pcl::SIFTKeypoint< PointInT, PointOutT > keypoint_detector ;
	keypoint_detector.setInputCloud(in_cloud);

	if ( nr_octaves != 0 && min_scale != 0 && nr_scales_per_octave != 0 )
	{
		keypoint_detector.setScales (min_scale, nr_octaves, nr_scales_per_octave);
	}

	if (min_contrast != 0)
	{
		keypoint_detector.setMinimumContrast(min_contrast);
	}

	keypoint_detector.compute(*out_cloud);
	return 1;
}

ExtractSIFT::ExtractSIFT()
	: BaseFilter(FilterDescription(	"Extract SIFT",
									"Extract SIFT Keypoints",
									"Extract SIFT keypoints for clouds with intensity/RGB or any scalar field",
									":/toolbar/PclUtils/icons/sift.png"))
	, m_dialog(0)
	, m_nr_octaves(0)
	, m_min_scale(0)
	, m_nr_scales_per_octave(0)
	, m_min_contrast(0)
	, m_use_min_contrast(false)
	, m_mode(RGB)
{
}

ExtractSIFT::~ExtractSIFT()
{
	//we must delete parent-less dialogs ourselves!
	if (m_dialog && m_dialog->parent() == 0)
		delete m_dialog;
}

int ExtractSIFT::checkSelected()
{
	//do we have a selected cloud?
	int have_cloud = isFirstSelectedCcPointCloud();
	if (have_cloud != 1)
		return -11;

	//do we have at least a scalar field?
	int have_sf = hasSelectedScalarField();
	if (have_sf == 1)
		return 1;

	//also having rgb data will be enough
	if (hasSelectedRGB() != 0)
		return 1;

	return -51;
}

int ExtractSIFT::openInputDialog()
{
	//do we have scalar fields?
	std::vector<std::string> fields = getSelectedAvailableScalarFields();

	//do we have rgb fields?
	if (hasSelectedRGB() == 1)
	{
		//add rgb field
		fields.push_back("rgb");
	}

	if (fields.empty()) //fields?
		return -51;

	//initialize the dialog object
	if (!m_dialog)
		m_dialog = new SIFTExtractDlg(m_app ? m_app->getMainWindow() : 0);

	//update the combo box
	m_dialog->updateComboBox(fields);

	if (!m_dialog->exec())
		return 0;

	return 1;
}

void ExtractSIFT::getParametersFromDialog()
{
	if (!m_dialog)
		return;

	//get the parameters from the dialog
	m_nr_octaves = m_dialog->nrOctaves->value();
	m_min_scale = static_cast<float>(m_dialog->minScale->value());
	m_nr_scales_per_octave = m_dialog->scalesPerOctave->value();
	m_use_min_contrast = m_dialog->useMinContrast->checkState();
	m_min_contrast = m_use_min_contrast ? static_cast<float>(m_dialog->minContrast->value()) : 0;
	m_field_to_use = m_dialog->intensityCombo->currentText();

	if (m_field_to_use == "rgb")
	{
		m_mode = RGB;
	}
	else
	{
		m_mode = SCALAR_FIELD;
	}

	QString fieldname(m_field_to_use);
	fieldname.replace(' ', '_');
	m_field_to_use_no_space = qPrintable(fieldname); //DGM: warning, toStdString doesn't preserve "local" characters
}

int ExtractSIFT::checkParameters()
{
	if ( (m_nr_octaves > 0) && (m_min_scale > 0) && (m_nr_scales_per_octave > 0))
	{
		if (m_use_min_contrast)
		{
			if (m_min_contrast > 0)
			{
				return 1;
			}
			else
			{
				return -52;
			}
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return -52;
	}
}

int ExtractSIFT::compute()
{
	ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return -1;

	std::list<std::string> req_fields;
	try
	{
		req_fields.push_back("xyz"); // always needed
		switch (m_mode)
		{
		case RGB:
			req_fields.push_back("rgb");
			break;
		case SCALAR_FIELD:
			req_fields.push_back(qPrintable(m_field_to_use)); //DGM: warning, toStdString doesn't preserve "local" characters
			break;
		default:
			assert(false);
			break;
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -1;
	}
	
	PCLCloud::Ptr sm_cloud = cc2smReader(cloud).getAsSM(req_fields);
	if (!sm_cloud)
		return -1;

	//Now change the name of the field to use to a standard name, only if in OTHER_FIELD mode
	if (m_mode == SCALAR_FIELD)
	{
		int field_index = pcl::getFieldIndex(*sm_cloud, m_field_to_use_no_space);
		sm_cloud->fields.at(field_index).name = "intensity"; //we always use intensity as name... even if it is curvature or another field.
	}

	//initialize all possible clouds
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	//Now do the actual computation
	if (m_mode == SCALAR_FIELD)
	{
		FROM_PCL_CLOUD(*sm_cloud, *cloud_i);
		estimateSIFT<pcl::PointXYZI, pcl::PointXYZ>(cloud_i, out_cloud, m_nr_octaves, m_min_scale, m_nr_scales_per_octave, m_min_contrast );
	}
	else if (m_mode == RGB)
	{
		FROM_PCL_CLOUD(*sm_cloud, *cloud_rgb);
		estimateSIFT<pcl::PointXYZRGB, pcl::PointXYZ>(cloud_rgb, out_cloud, m_nr_octaves, m_min_scale, m_nr_scales_per_octave, m_min_contrast );
	}

	PCLCloud::Ptr out_cloud_sm (new PCLCloud);
	TO_PCL_CLOUD(*out_cloud, *out_cloud_sm);

	if ( out_cloud_sm->height * out_cloud_sm->width == 0)
	{
		//cloud is empty
		return -53;
	}

	ccPointCloud* out_cloud_cc = sm2ccConverter(out_cloud_sm).getCloud();
	if (!out_cloud_cc)
	{
		//conversion failed (not enough memory?)
		return -1;
	}

	std::stringstream name;
	if (m_mode == RGB)
		name << "SIFT Keypoints_" << m_nr_octaves << "_" << "rgb" << "_" << m_min_scale << "_" << m_nr_scales_per_octave << "_" << m_min_contrast;
	else
		name << "SIFT Keypoints_" << m_nr_octaves << "_" << m_field_to_use_no_space  << "_" << m_min_scale << "_" << m_nr_scales_per_octave << "_" << m_min_contrast;

	out_cloud_cc->setName(name.str().c_str());
	out_cloud_cc->setDisplay(cloud->getDisplay());
	//copy global shift & scale
	out_cloud_cc->setGlobalScale(cloud->getGlobalScale());
	out_cloud_cc->setGlobalShift(cloud->getGlobalShift());

	if (cloud->getParent())
		cloud->getParent()->addChild(out_cloud_cc);

	emit newEntity(out_cloud_cc);

	return 1;
}

QString ExtractSIFT::getErrorMessage(int errorCode)
{
	switch(errorCode)
	{
		//THESE CASES CAN BE USED TO OVERRIDE OR ADD FILTER-SPECIFIC ERRORS CODES
		//ALSO IN DERIVED CLASSES DEFULAT MUST BE ""

	case -51:
		return QString("Selected entity does not have any suitable scalar field or RGB. Intensity scalar field or RGB are needed for computing SIFT");
	case -52:
		return QString("Wrong Parameters. One or more parameters cannot be accepted");
	case -53:
		return QString("SIFT keypoint extraction does not returned any point. Try relaxing your parameters");
	}

	return BaseFilter::getErrorMessage(errorCode);
}

template int estimateSIFT<pcl::PointXYZI, pcl::PointXYZ> (	const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
															pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud,
															int nr_octaves, float min_scale, int nr_scales_per_octave,
															float min_contrast);

template int estimateSIFT<pcl::PointXYZRGB, pcl::PointXYZ> (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
															pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud,
															int nr_octaves, float min_scale, int nr_scales_per_octave,
															float min_contrast);
