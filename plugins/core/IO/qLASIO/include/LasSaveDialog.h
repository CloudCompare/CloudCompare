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

#include "LasDetails.h"
#include "LasExtraScalarField.h"
#include "LasScalarField.h"
#include "ui_lassavedialog.h"

// Qt
#include <QDialog>

// CCCoreLib
#include <CCGeom.h>

class QStringListModel;
class ccScalarField;
class ccPointCloud;

class MappingLabel;
class LasExtraScalarFieldCard;

/// This dialog is responsible for presenting to the user
/// the different options when saving a point cloud to a LAS/LAZ file.
class LasSaveDialog : public QDialog
    , public Ui::LASSaveDialog
{
	Q_OBJECT

  public:
	/// Constructor
	explicit LasSaveDialog(ccPointCloud* cloud, QWidget* parent = nullptr);

	/// Set the file version and the point format
	void setVersionAndPointFormat(const LasDetails::LasVersion versionAndFmt);
	/// Set scale that would offer the user the best precision
	void setOptimalScale(const CCVector3d& scale, bool autoCheck = false);
	/// Set the scale that was used in the original file
	/// to save comes from.
	void setOriginalScale(const CCVector3d& scale, bool canUseScale, bool autoCheck = true);
	/// Set the extra LAS scalar fields saved from the original file.
	void setExtraScalarFields(const std::vector<LasExtraScalarField>& extraScalarFields);

	/// Offsets
	enum Offset
	{
		GLOBAL_SHIFT,
		ORIGN_LAS_OFFSET,
		MIN_BB_CORNER,
		CUSTOM_LAS_OFFSET
	};

	/// Sets the available offsets
	void setOffsets(const QMap<Offset, CCVector3d>& availableOffsets, Offset selectedOffsetType);

	/// Returns the point format currently selected
	uint8_t selectedPointFormat() const;
	/// Returns the version currently selected
	void selectedVersion(uint8_t& versionMajor, uint8_t& versionMinor) const;
	/// Returns the currently selected scale
	CCVector3d chosenScale() const;
	/// Returns whether the user wants to save RGB
	bool shouldSaveRGB() const;
	/// Returns whether the user wants to save the Waveforms
	bool shouldSaveWaveform() const;
	/// Returns whether the user wants to save normals as extra las scalar field
	bool shouldSaveNormalsAsExtraScalarField() const;
	/// Returns the chosen offset
	CCVector3d chosenOffset(Offset& offsetType) const;
	/// Returns the vector of LAS scalar fields the user wants to save.
	///
	/// Each LAS scalar fields is mapped to an existing point cloud's ccScalarField.
	/// The mapping is done by us and the user.
	std::vector<LasScalarField>      fieldsToSave() const;
	std::vector<LasExtraScalarField> extraFieldsToSave() const;

  public Q_SLOTS:
	void                     handleSelectedVersionChange(const QString&);
	void                     handleSelectedPointFormatChange(int index);
	void                     handleComboBoxChange(int index);
	void                     handleCustomScaleButtontoggled(bool checked);
	LasExtraScalarFieldCard* addExtraScalarFieldCard();

  private:
	LasExtraScalarFieldCard* createCard() const;
	/// This will scan the the point cloud scalar fields
	/// and create a default scalar field extra card if the field is
	/// neither selected as a standard field nor as an extra field.
	void assignLeftoverScalarFieldsAsExtra();
	void unassignDefaultFields();
	bool shouldAutomaticallyAssignLeftoverSFsAsExtra() const;

  private:
	ccPointCloud* m_cloud{nullptr};
	/// Model that contains the list of scalar fields names
	QStringListModel* m_scalarFieldsNamesModel{nullptr};
	/// Model that contains the list of data type names the user can choose
	/// for its extra scalar fields
	QStringListModel* m_extraFieldsDataTypesModel{nullptr};
	CCVector3d        m_optimalScale;
	CCVector3d        m_originalScale;
	/// Contains the mapping from a LAS field name to a combo box
	/// where the user (or us) selected the scalar field to use
	std::vector<std::pair<MappingLabel*, QComboBox*>> m_scalarFieldMapping;
	/// Output offsets that the user can choose
	QMap<Offset, CCVector3d> outputOffsets;
};
