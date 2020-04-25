//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccGeomFeaturesDlg.h"

//Qt
#include <QPushButton>
#include <QDialogButtonBox>

ccGeomFeaturesDlg::ccGeomFeaturesDlg(QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::GeomFeaturesDialog()
{
	setupUi(this);

	connect(buttonBox->button(QDialogButtonBox::Reset), &QPushButton::clicked, this, &ccGeomFeaturesDlg::reset);

	try
	{
		m_options.push_back(Option(roughnessCheckBox, CCCoreLib::GeometricalAnalysisTools::Roughness, 0));
		m_options.push_back(Option(firstOrderMomentCheckBox, CCCoreLib::GeometricalAnalysisTools::MomentOrder1, 0));
		m_options.push_back(Option(curvMeanCheckBox, CCCoreLib::GeometricalAnalysisTools::Curvature, CCCoreLib::Neighbourhood::MEAN_CURV));
		m_options.push_back(Option(curvGaussCheckBox, CCCoreLib::GeometricalAnalysisTools::Curvature, CCCoreLib::Neighbourhood::GAUSSIAN_CURV));
		m_options.push_back(Option(curvNCRCheckBox, CCCoreLib::GeometricalAnalysisTools::Curvature, CCCoreLib::Neighbourhood::NORMAL_CHANGE_RATE));
		m_options.push_back(Option(densityKnnCheckBox, CCCoreLib::GeometricalAnalysisTools::LocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_KNN));
		m_options.push_back(Option(densitySurfCheckBox, CCCoreLib::GeometricalAnalysisTools::LocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_2D));
		m_options.push_back(Option(densityVolCheckBox, CCCoreLib::GeometricalAnalysisTools::LocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_3D));
		m_options.push_back(Option(eigSumCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValuesSum));
		m_options.push_back(Option(eigOmnivarianceCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Omnivariance));
		m_options.push_back(Option(eigenentropyCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenEntropy));
		m_options.push_back(Option(eigAnisotropyCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Anisotropy));
		m_options.push_back(Option(eigPlanarityBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Planarity));
		m_options.push_back(Option(eigLinearityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Linearity));
		m_options.push_back(Option(eigPCA1CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::PCA1));
		m_options.push_back(Option(eigPCA2CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::PCA2));
		m_options.push_back(Option(eigSurfaceVarCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::SurfaceVariation));
		m_options.push_back(Option(eigSphericityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Sphericity));
		m_options.push_back(Option(eigVerticalityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Verticality));
		m_options.push_back(Option(eigenvalue1CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValue1));
		m_options.push_back(Option(eigenvalue2CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValue2));
		m_options.push_back(Option(eigenvalue3CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValue3));
	}
	catch (...)
	{
	}
	m_options.shrink_to_fit();
}

void ccGeomFeaturesDlg::setSelectedFeatures(const ccLibAlgorithms::GeomCharacteristicSet& features)
{
	reset();

	for (const ccLibAlgorithms::GeomCharacteristic& f : features)
	{
		//find the corresponding checkbox
		for (const Option& opt : m_options)
		{
			if (opt.charac == f.charac && opt.subOption == f.subOption)
			{
				opt.checkBox->setChecked(true);
				break;
			}
		}
	}
}

bool ccGeomFeaturesDlg::getSelectedFeatures(ccLibAlgorithms::GeomCharacteristicSet& features) const
{
	features.clear();
	
	try
	{
		//test each check-box and add the corresponding feature descriptor if necessary
		for (const Option& opt : m_options)
		{
			assert(opt.checkBox);
			if (opt.checkBox && opt.checkBox->isChecked())
				features.push_back(opt);
		}
		features.shrink_to_fit();
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
	return true;
}

double ccGeomFeaturesDlg::getRadius() const
{
	return radiusDoubleSpinBox->value();
}

void ccGeomFeaturesDlg::setRadius(double r)
{
	radiusDoubleSpinBox->setValue(r);
}

void ccGeomFeaturesDlg::reset()
{
	for (const Option& opt : m_options)
		opt.checkBox->setChecked(false);
}
