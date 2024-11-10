#pragma once

// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                   COPYRIGHT: Thomas Montaigu                           #
// #                                                                        #
// ##########################################################################

#include "LasExtraScalarField.h"
#include "ui_extra_scarlar_field_card.h"

class LasExtraScalarFieldCard : public QWidget
    , public Ui::ExtraScalarFieldCard
{
	Q_OBJECT

  public:
	explicit LasExtraScalarFieldCard(QWidget* parent = nullptr);

	/// Fills the card info from the given input field
	///
	/// A card filled using this method is considered as being non-default
	void fillFrom(const LasExtraScalarField& field);

	void fillAsDefault(const std::string& sfName);

	void reset();

	LasExtraScalarField::DataType dataType() const;

	/// Fills the `field` with the info the card contains
	///
	/// This includes linking the scalar field from the pointCloud to to the field.
	/// Returns `true` on success, `false` on failure.
	bool fillField(LasExtraScalarField& field, const ccPointCloud& pointCloud) const;

	inline bool isDefault() const
	{
		return !unlockModificationsButton->isHidden();
	};

	/// Returns true if this card maps the field with the name
	bool mapsFieldWithName(const std::string& sfName) const;

  private:
	/// Struct to aggregate together the user input related
	/// to one dimension of an extra scalar field definition.
	struct ScalarFieldUserInputs
	{
		QComboBox*      scalarFieldComboBox{nullptr};
		QDoubleSpinBox* scaleSpinBox{nullptr};
		QDoubleSpinBox* offsetSpinBox{nullptr};
	};

	void onNumberOfElementsSelected(unsigned numberOfElements);
	void onRadioButton1Selected(bool);
	void onRadioButton2Selected(bool);
	void onRadioButton3Selected(bool);
	void onUnlockModifications();

	void onToggleAdvancedOptionsClicked();

  private:
	ScalarFieldUserInputs m_scalarFieldsUserInputs[LasExtraScalarField::MAX_DIM_SIZE];

	LasExtraScalarField::DimensionSize dimensionSize() const;
};
