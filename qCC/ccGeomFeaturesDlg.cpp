// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccGeomFeaturesDlg.h"

// Qt
#include <QDialogButtonBox>
#include <QPushButton>

ccGeomFeaturesDlg::ccGeomFeaturesDlg(QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , Ui::GeomFeaturesDialog()
{
	setupUi(this);

	connect(buttonBox->button(QDialogButtonBox::Reset), &QPushButton::clicked, this, &ccGeomFeaturesDlg::reset);

	try
	{
		addOption(Option(roughnessCheckBox, CCCoreLib::GeometricalAnalysisTools::Roughness, 0));
		addOption(Option(firstOrderMomentCheckBox, CCCoreLib::GeometricalAnalysisTools::MomentOrder1, 0));
		addOption(Option(curvMeanCheckBox, CCCoreLib::GeometricalAnalysisTools::Curvature, CCCoreLib::Neighbourhood::MEAN_CURV));
		addOption(Option(curvGaussCheckBox, CCCoreLib::GeometricalAnalysisTools::Curvature, CCCoreLib::Neighbourhood::GAUSSIAN_CURV));
		addOption(Option(curvNCRCheckBox, CCCoreLib::GeometricalAnalysisTools::Curvature, CCCoreLib::Neighbourhood::NORMAL_CHANGE_RATE));
		addOption(Option(densityKnnCheckBox, CCCoreLib::GeometricalAnalysisTools::LocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_KNN));
		addOption(Option(densitySurfCheckBox, CCCoreLib::GeometricalAnalysisTools::LocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_2D));
		addOption(Option(densityVolCheckBox, CCCoreLib::GeometricalAnalysisTools::LocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_3D));
		addOption(Option(densityKnnApproxCheckBox, CCCoreLib::GeometricalAnalysisTools::ApproxLocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_KNN));
		addOption(Option(densitySurfApproxCheckBox, CCCoreLib::GeometricalAnalysisTools::ApproxLocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_2D));
		addOption(Option(densityVolApproxCheckBox, CCCoreLib::GeometricalAnalysisTools::ApproxLocalDensity, CCCoreLib::GeometricalAnalysisTools::DENSITY_3D));
		addOption(Option(eigSumCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValuesSum));
		addOption(Option(eigOmnivarianceCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Omnivariance));
		addOption(Option(eigenentropyCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenEntropy));
		addOption(Option(eigAnisotropyCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Anisotropy));
		addOption(Option(eigPlanarityBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Planarity));
		addOption(Option(eigLinearityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Linearity));
		addOption(Option(eigPCA1CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::PCA1));
		addOption(Option(eigPCA2CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::PCA2));
		addOption(Option(eigSurfaceVarCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::SurfaceVariation));
		addOption(Option(eigSphericityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Sphericity));
		addOption(Option(eigVerticalityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::Verticality));
		addOption(Option(eigenvalue1CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValue1));
		addOption(Option(eigenvalue2CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValue2));
		addOption(Option(eigenvalue3CheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::EigenValue3));
		addOption(Option(degreeOfPlanarityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::DegreeOfPlanarity));
		addOption(Option(degreeOfLinearityCheckBox, CCCoreLib::GeometricalAnalysisTools::Feature, CCCoreLib::Neighbourhood::DegreeOfLinearity));
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccGeomFeaturesDlg] Not enough memory!");
	}
}

void ccGeomFeaturesDlg::addOption(const Option& option)
{
	auto it = std::find(m_options.begin(), m_options.end(), option);
	if (it != m_options.end())
	{
		ccLog::Warning(QString("Internal error: duplicate option for characteristic '%1'").arg(static_cast<int>(option.gc.charac)));
		assert(false);
		return;
	}
	if (option.checkBox == nullptr)
	{
		ccLog::Warning(QString("Internal error: null checkbox for characteristic '%1'").arg(static_cast<int>(option.gc.charac)));
		assert(false);
		return;
	}
	m_options.push_back(option);
}

void ccGeomFeaturesDlg::setUpDirection(const CCVector3& upDir)
{
	upDirXDoubleSpinBox->setValue(upDir.x);
	upDirYDoubleSpinBox->setValue(upDir.y);
	upDirZDoubleSpinBox->setValue(upDir.z);
	upDirGroupBox->setChecked(true);
}

CCVector3* ccGeomFeaturesDlg::getUpDirection() const
{
	if (roughnessCheckBox->isChecked() && upDirGroupBox->isChecked())
	{
		static CCVector3 UpDirection(0, 0, 1);
		UpDirection.x = static_cast<PointCoordinateType>(upDirXDoubleSpinBox->value());
		UpDirection.y = static_cast<PointCoordinateType>(upDirYDoubleSpinBox->value());
		UpDirection.z = static_cast<PointCoordinateType>(upDirZDoubleSpinBox->value());
		return &UpDirection;
	}
	return nullptr;
}

void ccGeomFeaturesDlg::setSelectedFeatures(const ccLibAlgorithms::GeomCharacteristicWithSubOptionSet& features)
{
	reset();

	for (const auto& f : features)
	{
		// find the corresponding checkbox
		auto it = std::find(m_options.begin(), m_options.end(), Option(nullptr, f.charac, f.subOption));
		if (it != m_options.end())
		{
			it->checkBox->setChecked(true);
		}
	}
}

bool ccGeomFeaturesDlg::getSelectedFeatures(ccLibAlgorithms::GeomCharacteristicWithSubOptionSet& features) const
{
	features.clear();

	try
	{
		// test each check-box and add the corresponding feature descriptor if necessary
		for (const Option& opt : m_options)
		{
			assert(opt.checkBox);
			if (opt.checkBox && opt.checkBox->isChecked())
			{
				features.push_back(opt.gc);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccGeomFeaturesDlg::getSelectedFeatures] Not enough memory!");
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
	{
		assert(opt.checkBox);
		opt.checkBox->setChecked(false);
	}
}
