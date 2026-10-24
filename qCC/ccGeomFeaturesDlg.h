#pragma once

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
// #                   COPYRIGHT: Daniel Girardeau-Montaut                  #
// #                                                                        #
// ##########################################################################

// Local
#include "ccLibAlgorithms.h"

// Ui
#include <ui_geomFeaturesDlg.h>

// Qt
#include <QDialog>

//! Dialog for computing the density of a point clouds
class ccGeomFeaturesDlg : public QDialog
    , public Ui::GeomFeaturesDialog
{
  public:
	//! Default constructor
	explicit ccGeomFeaturesDlg(QWidget* parent = nullptr);

	//! Sets selected features
	void setSelectedFeatures(const ccLibAlgorithms::GeomCharacteristicWithSubOptionSet& features);
	//! Returns selected features
	bool getSelectedFeatures(ccLibAlgorithms::GeomCharacteristicWithSubOptionSet& features) const;
	//! Sets the default kernel radius (for 'precise' mode only)
	void setRadius(double r);
	//! Returns	the kernel radius (for 'precise' mode only)
	double getRadius() const;

	//! Sets the 'up direction' (and enables the group at the same time)
	void setUpDirection(const CCVector3& upDir);
	//! Returns the 'up direction' if any is defined (nullptr otherwise)
	CCVector3* getUpDirection() const;

	//! Reset the whole dialog
	void reset();

  protected:
	//! Composite structure for a geometric characteristic option and its corresponding checkbox
	struct Option
	{
		//! Default constructor
		Option(QCheckBox* cb = nullptr, CCCoreLib::GeometricalAnalysisTools::GeomCharacteristic c = CCCoreLib::GeometricalAnalysisTools::Feature, int option = 0)
		    : gc(c, option)
		    , checkBox(cb)
		{
		}

		//! Equality operator
		bool operator==(const Option& other) const
		{
			return gc == other.gc;
		}

		ccLibAlgorithms::GeomCharacteristicWithSubOption gc;                 //!< Geometric characteristic (with sub option)
		QCheckBox*                                       checkBox = nullptr; //!< Corresponding checkbox
	};

	//! Add option
	void addOption(const Option& option);

	//! List of all available options
	std::vector<Option> m_options;
};
