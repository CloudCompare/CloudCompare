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

#ifndef LASSAVEDIALOG_H
#define LASSAVEDIALOG_H

#include <QDialog>

#include <CCGeom.h>

#include "LasFields.h"
#include "ui_lassavedialog.h"

class QStringListModel;
class ccScalarField;
class ccPointCloud;

class ScalarFieldRow;
class ExtraScalarFieldForm;
class LasScalarFieldForm;

class LasSaveDialog : public QDialog, public Ui::LASSaveDialog {
  Q_OBJECT
public:
  explicit LasSaveDialog(ccPointCloud *cloud, QWidget *parent = nullptr);

  void setVersionAndPointFormat(const QString &version,
                                unsigned int pointFormat);
  void setSavedScale(const CCVector3d &savedScale);
  void
  setExtraScalarFields(const std::vector<SavedExtraField> &savedExtraFields);

  unsigned int selectedPointFormat() const;
  unsigned int selectedVersionMinor() const;
  CCVector3d chosenScale() const;
  bool shouldSaveColors() const;
  //    bool shouldSaveWaveform() const;

  LasField::Vector fieldsToSave() const;

public Q_SLOTS:
  void handleSelectedVersionChange(const QString &);
  void handleSelectedPointFormatChange(int index);

private:
  void setOptimalScale(const CCVector3d &optimalScale);

private:
  ccPointCloud *m_cloud{nullptr};
  QStringListModel *m_comboBoxModel{nullptr};
  std::vector<SavedExtraField> m_savedExtraFields;
  ExtraScalarFieldForm *m_ebForm;
  LasScalarFieldForm *m_scalarFieldForm;

  friend ScalarFieldRow;
};

#endif // LASSAVEDIALOG_H
