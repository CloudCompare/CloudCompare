#pragma once

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
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
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################

#include <QDialog>

#include <CCGeom.h>

#include "LasDetails.h"
#include "ui_lassavedialog.h"

class QStringListModel;
class ccScalarField;
class ccPointCloud;

class MappingLabel;

/// This dialog is responsible for presenting to the user
/// the different options when saving a point cloud to a LAS/LAZ file.
class LasSaveDialog : public QDialog, public Ui::LASSaveDialog
{
    Q_OBJECT

  public:
    explicit LasSaveDialog(ccPointCloud *cloud, QWidget *parent = nullptr);

    void setVersionAndPointFormat(const QString &version, unsigned int pointFormat);
    void setOptimalScale(const CCVector3d &optimalScale);
    void setSavedScale(const CCVector3d &savedScale);
    void setExtraScalarFields(const std::vector<LasExtraScalarField> &extraScalarFields);

    unsigned int selectedPointFormat() const;
    unsigned int selectedVersionMinor() const;
    CCVector3d chosenScale() const;
    bool shouldSaveRGB() const;
    bool shouldSaveWaveform() const;

    std::vector<LasScalarField> fieldsToSave() const;

  public Q_SLOTS:
    void handleSelectedVersionChange(const QString &);
    void handleSelectedPointFormatChange(int index);
    void handleComboBoxChange(int index);

  private:
    ccPointCloud *m_cloud{nullptr};
    QStringListModel *m_comboBoxModel{nullptr};
    /// Contains the mapping from a LAS field name to a combo box
    /// where the user (or us) selected the scalar field to use
    std::vector<std::pair<MappingLabel *, QComboBox *>> m_scalarFieldMapping;
};
