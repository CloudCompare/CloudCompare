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

// Error codes
static constexpr int NoSForRGB = -51;
static constexpr int EmptyOutput = -52;

//! Extract SIFT keypoints
/** if only the point cloud is given PCL default parameters are used (that are not really good, so please give parameters)
	\note Different types can be passed as input for this function:
		- PointXYZI
		- PointNormal
		- PointXYZRGB
	\note If a PointType with a scale field is passed as output type, scales will be returned together with the return cloud
**/
template <typename PointInT, typename PointOutT>
static int EstimateSIFT(const typename pcl::PointCloud<PointInT>::Ptr in_cloud,
						typename pcl::PointCloud<PointOutT>& out_cloud,
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

	keypoint_detector.compute(out_cloud);
	return BaseFilter::Success;
}

ExtractSIFT::ExtractSIFT()
	: BaseFilter(FilterDescription(	"Extract SIFT",
									"Extract SIFT Keypoints",
									"Extract SIFT keypoints for clouds with intensity/RGB or any scalar field",
									":/toolbar/PclUtils/icons/sift.png"))
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
}

bool ExtractSIFT::checkSelected() const
{
	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (!cloud)
	{
		return false;
	}

	//we need at least one scalar field or RGB colors
	return cloud->hasScalarFields() || cloud->hasColors();
}

int ExtractSIFT::getParametersFromDialog()
{
	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (!cloud)
	{
		assert(false);
		return InvalidInput;
	}

	//do we have scalar fields?
	unsigned fieldCount = cloud->getNumberOfScalarFields();
	QStringList fieldNames;
	fieldNames.reserve(fieldCount);
	for (unsigned i = 0; i < fieldCount; i++)
	{
		fieldNames << QString::fromStdString(cloud->getScalarFieldName(i));
	}

	//do we have rgb fields?
	if (cloud->hasColors())
	{
		//add rgb field
		fieldNames << "rgb";
	}

	if (fieldNames.empty())
	{
		//no scalar field or RGB?
		assert(false);
		return NoSForRGB;
	}

	//initialize the dialog object
	SIFTExtractDlg dialog(m_app ? m_app->getMainWindow() : nullptr);

	//update the combo box
	dialog.updateComboBox(fieldNames);

	if (!dialog.exec())
	{
		return CancelledByUser;
	}

	//get the parameters from the dialog
	m_nr_octaves = dialog.nrOctaves->value();
	m_min_scale = static_cast<float>(dialog.minScale->value());
	m_nr_scales_per_octave = dialog.scalesPerOctave->value();
	m_use_min_contrast = dialog.useMinContrast->checkState();
	m_min_contrast = m_use_min_contrast ? static_cast<float>(dialog.minContrast->value()) : 0.0f;
	m_field_to_use = dialog.intensityCombo->currentText();

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
	m_field_to_use_no_space = fieldname.toStdString(); //DGM: warning, toStdString doesn't preserve "local" characters

	return checkParameters();
}

int ExtractSIFT::checkParameters()
{
	if ((m_nr_octaves > 0) && (m_min_scale > 0) && (m_nr_scales_per_octave > 0))
	{
		if (m_use_min_contrast)
		{
			if (m_min_contrast > 0)
			{
				return Success;
			}
			else
			{
				return InvalidParameters;
			}
		}
		else
		{
			return Success;
		}
	}
	else
	{
		return InvalidParameters;
	}
}

int ExtractSIFT::compute()
{
	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (!cloud)
	{
		return InvalidInput;
	}

	QStringList scalarFields;
	if (m_mode == SCALAR_FIELD)
	{
		scalarFields << m_field_to_use;
	}
	
	PCLCloud::Ptr sm_cloud = cc2smReader(cloud).getAsSM(true, false, m_mode == RGB, scalarFields);
	if (!sm_cloud)
	{
		return NotEnoughMemory;
	}

	//Now change the name of the field to use to a standard name, only if in OTHER_FIELD mode
	if (m_mode == SCALAR_FIELD)
	{
		int field_index = pcl::getFieldIndex(*sm_cloud, m_field_to_use_no_space);
		sm_cloud->fields[field_index].name = "intensity"; //we always use intensity as name... even if it is curvature or another field.
	}

	//Now do the actual computation
	pcl::PointCloud<pcl::PointXYZ> out_cloud;
	if (m_mode == SCALAR_FIELD)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
		FROM_PCL_CLOUD(*sm_cloud, *cloud_i);
		EstimateSIFT<pcl::PointXYZI, pcl::PointXYZ>(cloud_i, out_cloud, m_nr_octaves, m_min_scale, m_nr_scales_per_octave, m_min_contrast );
	}
	else if (m_mode == RGB)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
		FROM_PCL_CLOUD(*sm_cloud, *cloud_rgb);
		EstimateSIFT<pcl::PointXYZRGB, pcl::PointXYZ>(cloud_rgb, out_cloud, m_nr_octaves, m_min_scale, m_nr_scales_per_octave, m_min_contrast );
	}

	PCLCloud out_cloud_sm;
	TO_PCL_CLOUD(out_cloud, out_cloud_sm);

	if (out_cloud_sm.height * out_cloud_sm.width == 0)
	{
		//cloud is empty
		return EmptyOutput;
	}

	ccPointCloud* out_cloud_cc = pcl2cc::Convert(out_cloud_sm);
	if (!out_cloud_cc)
	{
		//conversion failed (not enough memory?)
		return NotEnoughMemory;
	}

	std::stringstream name;
	if (m_mode == RGB)
		name << "SIFT Keypoints_" << m_nr_octaves << "_" << "rgb" << "_" << m_min_scale << "_" << m_nr_scales_per_octave << "_" << m_min_contrast;
	else
		name << "SIFT Keypoints_" << m_nr_octaves << "_" << m_field_to_use_no_space  << "_" << m_min_scale << "_" << m_nr_scales_per_octave << "_" << m_min_contrast;

	out_cloud_cc->setName(name.str().c_str());
	out_cloud_cc->setDisplay(cloud->getDisplay());
	//copy global shift & scale
	out_cloud_cc->copyGlobalShiftAndScale(*cloud);

	if (cloud->getParent())
	{
		cloud->getParent()->addChild(out_cloud_cc);
	}

	Q_EMIT newEntity(out_cloud_cc);

	return Success;
}

QString ExtractSIFT::getErrorMessage(int errorCode) const
{
	switch (errorCode)
	{
	case NoSForRGB:
		return tr("Selected entity does not have any suitable scalar field or RGB. Intensity scalar field or RGB are needed for computing SIFT");
	case EmptyOutput:
		return tr("SIFT keypoint extraction did not return any point. Try relaxing your parameters");
	default:
		//see below
		break;
	}

	return BaseFilter::getErrorMessage(errorCode);
}
